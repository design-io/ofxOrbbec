#pragma once

//
//  ofxOrbbecH264Decoder.h
//
//  Hardware-accelerated H.264 / H.265 decoding for Orbbec network-mode colour
//  streams on macOS, using VideoToolbox.
//
//  The Femto Mega only publishes compressed colour profiles when connected over
//  Ethernet. This class takes the raw Annex-B bytes of an OB_FORMAT_H264 or
//  OB_FORMAT_H265 frame and produces an ofPixels.
//
//  Link against: VideoToolbox.framework, CoreMedia.framework, CoreVideo.framework
//

#include "ofPixels.h"
#include "ofLog.h"

#ifdef TARGET_OSX

#include <VideoToolbox/VideoToolbox.h>
#include <CoreMedia/CoreMedia.h>
#include <CoreVideo/CoreVideo.h>

#include <cstdint>
#include <mutex>
#include <vector>

class ofxOrbbecH264Decoder {
public:
    ofxOrbbecH264Decoder();
    ~ofxOrbbecH264Decoder();

    // Owns a VideoToolbox session — not copyable, not movable.
    ofxOrbbecH264Decoder(const ofxOrbbecH264Decoder &) = delete;
    ofxOrbbecH264Decoder & operator=(const ofxOrbbecH264Decoder &) = delete;

    /// Feed one Annex-B access unit. Pass bH264 = true for OB_FORMAT_H264,
    /// false for OB_FORMAT_H265 — matching the addon's decodeH26XFrame() shape.
    ///
    /// Returns true when a new decoded picture is ready in getPixels().
    /// Returns false — without it being an error — while waiting for the first
    /// keyframe, or if the access unit held only parameter sets.
    bool decode(const uint8_t * data, size_t numBytes, bool bH264);

    bool isFrameNew() const { return mbFrameNew; }

    ofPixels & getPixels() { return mPixels; }
    const ofPixels & getPixels() const { return mPixels; }

    int getWidth() const { return mWidth; }
    int getHeight() const { return mHeight; }

    /// getPixels() is always OF_PIXELS_RGB (3-byte, tightly packed) — matching
    /// the rest of the addon and what the Orbbec point-cloud helpers expect.
    /// The decoded stream has no alpha, so there is nothing to preserve.

    /// True once parameter sets have arrived and a session exists.
    bool isReady() const { return mSession != nullptr; }

    void clear();

private:
    enum class Codec { None, H264, H265 };

    struct Nal {
        size_t  offset;   // into the caller's buffer, past the start code
        size_t  size;
        uint8_t type;
    };

    static void parseAnnexB(const uint8_t * d, size_t size, Codec codec, std::vector<Nal> & out);

    bool createSession();
    void destroySession();

    static void outputCallback(void * decompressionOutputRefCon,
                               void * sourceFrameRefCon,
                               OSStatus status,
                               VTDecodeInfoFlags infoFlags,
                               CVImageBufferRef imageBuffer,
                               CMTime presentationTimeStamp,
                               CMTime presentationDuration);

    void handleDecodedImage(CVImageBufferRef imageBuffer);

    VTDecompressionSessionRef   mSession    = nullptr;
    CMVideoFormatDescriptionRef mFormatDesc = nullptr;
    Codec                       mCodec      = Codec::None;

    // H.264 uses SPS + PPS. H.265 additionally needs VPS.
    std::vector<uint8_t> mVPS;
    std::vector<uint8_t> mSPS;
    std::vector<uint8_t> mPPS;

    // Reused across frames to keep the decode path allocation-free.
    std::vector<uint8_t> mAvcc;
    std::vector<Nal>     mNals;

    ofPixels   mPixels;
    std::mutex mMutex;

    int  mWidth  = 0;
    int  mHeight = 0;
    bool mbFrameNew    = false;
    bool mbGotKeyframe = false;
};

#endif // TARGET_OSX
