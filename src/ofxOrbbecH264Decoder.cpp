#include "ofxOrbbecH264Decoder.h"

#ifdef TARGET_OSX

namespace {
    // ── H.264 NAL unit types (spec Table 7-1) ─────────────────────────────
    constexpr uint8_t kH264_Slice = 1;    // non-IDR coded slice
    constexpr uint8_t kH264_IDR   = 5;    // IDR coded slice (keyframe)
    constexpr uint8_t kH264_SPS   = 7;
    constexpr uint8_t kH264_PPS   = 8;

    // ── H.265 NAL unit types (spec Table 7-1) ─────────────────────────────
    // VCL NALs are 0..31. IRAP (keyframe) pictures are 16..23.
    constexpr uint8_t kH265_VCL_MAX   = 31;
    constexpr uint8_t kH265_IRAP_MIN  = 16;   // BLA_W_LP
    constexpr uint8_t kH265_IRAP_MAX  = 23;   // RSV_IRAP_VCL23
    constexpr uint8_t kH265_VPS       = 32;
    constexpr uint8_t kH265_SPS       = 33;
    constexpr uint8_t kH265_PPS       = 34;
}

//--------------------------------------------------------------
ofxOrbbecH264Decoder::ofxOrbbecH264Decoder() {
    mNals.reserve(16);
    mAvcc.reserve(512 * 1024);
}

//--------------------------------------------------------------
ofxOrbbecH264Decoder::~ofxOrbbecH264Decoder() {
    destroySession();
}

//--------------------------------------------------------------
void ofxOrbbecH264Decoder::clear() {
    destroySession();
    mVPS.clear();
    mSPS.clear();
    mPPS.clear();
    mCodec        = Codec::None;
    mbGotKeyframe = false;
    mbFrameNew    = false;
    mWidth = mHeight = 0;
}

//--------------------------------------------------------------
void ofxOrbbecH264Decoder::destroySession() {
    if (mSession) {
        VTDecompressionSessionWaitForAsynchronousFrames(mSession);
        VTDecompressionSessionInvalidate(mSession);
        CFRelease(mSession);
        mSession = nullptr;
    }
    if (mFormatDesc) {
        CFRelease(mFormatDesc);
        mFormatDesc = nullptr;
    }
}

//--------------------------------------------------------------
void ofxOrbbecH264Decoder::parseAnnexB(const uint8_t * d, size_t size, Codec codec, std::vector<Nal> & out) {
    out.clear();
    if (size < 4) return;

    size_t i        = 0;
    size_t nalStart = 0;
    bool   inNal    = false;

    auto pushNal = [&](size_t end) {
        // Trim trailing_zero_8bits, which also removes the leading zero of a
        // following 4-byte start code.
        while (end > nalStart && d[end - 1] == 0x00) end--;
        if (end > nalStart) {
            Nal n;
            n.offset = nalStart;
            n.size   = end - nalStart;
            // H.264 header is 1 byte, type in the low 5 bits.
            // H.265 header is 2 bytes, type in bits 1..6 of the first byte.
            n.type = (codec == Codec::H265) ? uint8_t((d[nalStart] >> 1) & 0x3F)
                                            : uint8_t(d[nalStart] & 0x1F);
            out.push_back(n);
        }
    };

    while (i + 2 < size) {
        if (d[i] == 0x00 && d[i + 1] == 0x00 && d[i + 2] == 0x01) {
            if (inNal) pushNal(i);
            i += 3;
            nalStart = i;
            inNal    = true;
            continue;
        }
        i++;
    }

    if (inNal && nalStart < size) pushNal(size);
}

//--------------------------------------------------------------
bool ofxOrbbecH264Decoder::createSession() {
    destroySession();

    OSStatus status = noErr;

    if (mCodec == Codec::H264) {
        if (mSPS.empty() || mPPS.empty()) return false;

        const uint8_t * paramSets[2]  = { mSPS.data(), mPPS.data() };
        const size_t    paramSizes[2] = { mSPS.size(), mPPS.size() };

        status = CMVideoFormatDescriptionCreateFromH264ParameterSets(
            kCFAllocatorDefault, 2, paramSets, paramSizes,
            4,                  // 4-byte AVCC length prefixes
            &mFormatDesc);

    } else if (mCodec == Codec::H265) {
        if (mVPS.empty() || mSPS.empty() || mPPS.empty()) return false;

        const uint8_t * paramSets[3]  = { mVPS.data(), mSPS.data(), mPPS.data() };
        const size_t    paramSizes[3] = { mVPS.size(), mSPS.size(), mPPS.size() };

        // __builtin_available is Clang's C/C++ equivalent of Objective-C's
        // @available. Plain @available only parses in .m / .mm files.
        if (__builtin_available(macOS 10.13, *)) {
            status = CMVideoFormatDescriptionCreateFromHEVCParameterSets(
                kCFAllocatorDefault, 3, paramSets, paramSizes,
                4,              // 4-byte HVCC length prefixes
                nullptr,        // no extensions
                &mFormatDesc);
        } else {
            ofLogError("ofxOrbbecH264Decoder") << "H.265 requires macOS 10.13 or later";
            return false;
        }
    } else {
        return false;
    }

    if (status != noErr || !mFormatDesc) {
        ofLogError("ofxOrbbecH264Decoder") << "format description creation failed: " << status;
        mFormatDesc = nullptr;
        return false;
    }

    CMVideoDimensions dims = CMVideoFormatDescriptionGetDimensions(mFormatDesc);
    mWidth  = dims.width;
    mHeight = dims.height;

    // Ask VideoToolbox for packed BGRA so we skip doing YUV conversion ourselves.
    CFMutableDictionaryRef attrs = CFDictionaryCreateMutable(
        kCFAllocatorDefault, 4, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);

    int32_t     pixFmt    = kCVPixelFormatType_32BGRA;
    CFNumberRef pixFmtRef = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &pixFmt);
    CFDictionarySetValue(attrs, kCVPixelBufferPixelFormatTypeKey, pixFmtRef);
    CFRelease(pixFmtRef);

    CFNumberRef wRef = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &dims.width);
    CFNumberRef hRef = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &dims.height);
    CFDictionarySetValue(attrs, kCVPixelBufferWidthKey,  wRef);
    CFDictionarySetValue(attrs, kCVPixelBufferHeightKey, hRef);
    CFRelease(wRef);
    CFRelease(hRef);

    // Empty IOSurface dictionary keeps buffers IOSurface-backed, which matters
    // if you later want a zero-copy path to a texture.
    CFDictionaryRef ioSurfaceProps = CFDictionaryCreate(
        kCFAllocatorDefault, nullptr, nullptr, 0,
        &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    CFDictionarySetValue(attrs, kCVPixelBufferIOSurfacePropertiesKey, ioSurfaceProps);
    CFRelease(ioSurfaceProps);

    VTDecompressionOutputCallbackRecord cbRecord;
    cbRecord.decompressionOutputCallback = &ofxOrbbecH264Decoder::outputCallback;
    cbRecord.decompressionOutputRefCon   = this;

    status = VTDecompressionSessionCreate(
        kCFAllocatorDefault, mFormatDesc,
        nullptr,        // decoder specification — let the system pick hardware
        attrs, &cbRecord, &mSession);

    CFRelease(attrs);

    if (status != noErr || !mSession) {
        ofLogError("ofxOrbbecH264Decoder") << "VTDecompressionSessionCreate failed: " << status;
        mSession = nullptr;
        CFRelease(mFormatDesc);
        mFormatDesc = nullptr;
        return false;
    }

    ofLogNotice("ofxOrbbecH264Decoder")
        << (mCodec == Codec::H265 ? "H.265" : "H.264")
        << " session created " << mWidth << "x" << mHeight;
    return true;
}

//--------------------------------------------------------------
bool ofxOrbbecH264Decoder::decode(const uint8_t * data, size_t numBytes, bool bH264) {
    mbFrameNew = false;

    if (!data || numBytes < 4) return false;

    const Codec wanted = bH264 ? Codec::H264 : Codec::H265;
    if (wanted != mCodec) {
        // Stream switched codec (or this is the first frame) — start clean.
        clear();
        mCodec = wanted;
    }

    parseAnnexB(data, numBytes, mCodec, mNals);
    if (mNals.empty()) return false;

    // ── Collect parameter sets and look for a keyframe ────────────────────
    bool bParamsChanged = false;
    bool bHasKeyframe   = false;

    auto capture = [&](std::vector<uint8_t> & dst, const Nal & n) {
        std::vector<uint8_t> v(data + n.offset, data + n.offset + n.size);
        if (v != dst) { dst = std::move(v); bParamsChanged = true; }
    };

    for (const auto & n : mNals) {
        if (mCodec == Codec::H264) {
            if      (n.type == kH264_SPS) capture(mSPS, n);
            else if (n.type == kH264_PPS) capture(mPPS, n);
            else if (n.type == kH264_IDR) bHasKeyframe = true;
        } else {
            if      (n.type == kH265_VPS) capture(mVPS, n);
            else if (n.type == kH265_SPS) capture(mSPS, n);
            else if (n.type == kH265_PPS) capture(mPPS, n);
            else if (n.type >= kH265_IRAP_MIN && n.type <= kH265_IRAP_MAX) bHasKeyframe = true;
        }
    }

    // Resolution or encoder config changed mid-stream — rebuild.
    if (bParamsChanged) {
        mbGotKeyframe = false;
        if (!createSession()) return false;
    }

    if (!mSession) {
        if (!createSession()) return false;   // no-op until parameter sets arrive
    }

    // Feeding non-keyframes before the first IRAP/IDR produces garbage or errors.
    if (!mbGotKeyframe) {
        if (!bHasKeyframe) return false;
        mbGotKeyframe = true;
    }

    // ── Annex-B -> length-prefixed (AVCC / HVCC) ──────────────────────────
    mAvcc.clear();
    for (const auto & n : mNals) {
        const bool bIsVCL = (mCodec == Codec::H264)
            ? (n.type >= kH264_Slice && n.type <= kH264_IDR)
            : (n.type <= kH265_VCL_MAX);
        if (!bIsVCL) continue;

        const uint32_t len = static_cast<uint32_t>(n.size);
        const uint8_t  hdr[4] = {
            static_cast<uint8_t>((len >> 24) & 0xFF),
            static_cast<uint8_t>((len >> 16) & 0xFF),
            static_cast<uint8_t>((len >>  8) & 0xFF),
            static_cast<uint8_t>( len        & 0xFF)
        };
        mAvcc.insert(mAvcc.end(), hdr, hdr + 4);
        mAvcc.insert(mAvcc.end(), data + n.offset, data + n.offset + n.size);
    }

    if (mAvcc.empty()) return false;   // parameter-set-only access unit

    // ── Wrap in a CMSampleBuffer ──────────────────────────────────────────
    CMBlockBufferRef blockBuffer = nullptr;
    OSStatus status = CMBlockBufferCreateWithMemoryBlock(
        kCFAllocatorDefault, nullptr, mAvcc.size(), kCFAllocatorDefault,
        nullptr, 0, mAvcc.size(), kCMBlockBufferAssureMemoryNowFlag, &blockBuffer);

    if (status != kCMBlockBufferNoErr) {
        ofLogError("ofxOrbbecH264Decoder") << "CMBlockBufferCreateWithMemoryBlock failed: " << status;
        return false;
    }

    status = CMBlockBufferReplaceDataBytes(mAvcc.data(), blockBuffer, 0, mAvcc.size());
    if (status != kCMBlockBufferNoErr) {
        ofLogError("ofxOrbbecH264Decoder") << "CMBlockBufferReplaceDataBytes failed: " << status;
        CFRelease(blockBuffer);
        return false;
    }

    CMSampleBufferRef sampleBuffer = nullptr;
    const size_t sampleSize = mAvcc.size();
    status = CMSampleBufferCreateReady(
        kCFAllocatorDefault, blockBuffer, mFormatDesc,
        1,          // numSamples
        0,          // numSampleTimingEntries — decode-only, no timing needed
        nullptr,
        1,          // numSampleSizeEntries
        &sampleSize,
        &sampleBuffer);

    CFRelease(blockBuffer);

    if (status != noErr) {
        ofLogError("ofxOrbbecH264Decoder") << "CMSampleBufferCreateReady failed: " << status;
        return false;
    }

    // ── Decode ────────────────────────────────────────────────────────────
    // Flags of 0 means synchronous: outputCallback fires before this returns.
    VTDecodeFrameFlags decodeFlags = 0;
    VTDecodeInfoFlags  infoFlags   = 0;

    status = VTDecompressionSessionDecodeFrame(mSession, sampleBuffer, decodeFlags, nullptr, &infoFlags);
    CFRelease(sampleBuffer);

    if (status != noErr) {
        ofLogWarning("ofxOrbbecH264Decoder") << "VTDecompressionSessionDecodeFrame failed: " << status;
        if (status == kVTInvalidSessionErr || status == kVTVideoDecoderMalfunctionErr) {
            // Session went bad (often after a display/GPU change). Rebuild on
            // the next keyframe.
            destroySession();
            mbGotKeyframe = false;
        }
        return false;
    }

    VTDecompressionSessionWaitForAsynchronousFrames(mSession);
    return mbFrameNew;
}

//--------------------------------------------------------------
void ofxOrbbecH264Decoder::outputCallback(void * decompressionOutputRefCon,
                                          void * sourceFrameRefCon,
                                          OSStatus status,
                                          VTDecodeInfoFlags infoFlags,
                                          CVImageBufferRef imageBuffer,
                                          CMTime presentationTimeStamp,
                                          CMTime presentationDuration) {
    (void)sourceFrameRefCon;
    (void)presentationTimeStamp;
    (void)presentationDuration;

    if (status != noErr) {
        ofLogWarning("ofxOrbbecH264Decoder") << "decompression callback status: " << status;
        return;
    }
    if (infoFlags & kVTDecodeInfo_FrameDropped) return;
    if (!imageBuffer) return;

    auto * self = static_cast<ofxOrbbecH264Decoder *>(decompressionOutputRefCon);
    if (self) self->handleDecodedImage(imageBuffer);
}

//--------------------------------------------------------------
void ofxOrbbecH264Decoder::handleDecodedImage(CVImageBufferRef imageBuffer) {
    CVPixelBufferLockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);

    const size_t w      = CVPixelBufferGetWidth(imageBuffer);
    const size_t h      = CVPixelBufferGetHeight(imageBuffer);
    const size_t stride = CVPixelBufferGetBytesPerRow(imageBuffer);
    const auto * src    = static_cast<const uint8_t *>(CVPixelBufferGetBaseAddress(imageBuffer));

    if (!src || w == 0 || h == 0) {
        CVPixelBufferUnlockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mMutex);

        if (!mPixels.isAllocated() ||
            mPixels.getWidth()  != static_cast<size_t>(w) ||
            mPixels.getHeight() != static_cast<size_t>(h) ||
            mPixels.getPixelFormat() != OF_PIXELS_RGB) {
            mPixels.allocate(w, h, OF_PIXELS_RGB);
        }

        // VideoToolbox hands us BGRA. Repack to tightly packed RGB888,
        // dropping the alpha the stream never had.
        uint8_t * dst = mPixels.getData();
        for (size_t y = 0; y < h; y++) {
            const uint8_t * s = src + y * stride;
            uint8_t *       d = dst + y * w * 3;
            for (size_t x = 0; x < w; x++) {
                d[0] = s[2];   // R
                d[1] = s[1];   // G
                d[2] = s[0];   // B
                s += 4;
                d += 3;
            }
        }
    }

    CVPixelBufferUnlockBaseAddress(imageBuffer, kCVPixelBufferLock_ReadOnly);
    mbFrameNew = true;
}

#endif // TARGET_OSX
