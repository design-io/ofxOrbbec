
#include "ofxOrbbecCamera.h"
#include <libobsensor/hpp/Utils.hpp>

using namespace std;

//-------------------------------------------------------------
//std::vector < std::shared_ptr<ob::DeviceInfo> > ofxOrbbecCamera::getDeviceList(bool bNetIncDevices){
std::vector< std::shared_ptr<ofxOrbbec::DeviceInfo> > ofxOrbbecCamera::getDeviceList(bool bNetIncDevices) {
//    std::vector<std::shared_ptr<ob::DeviceInfo> > dInfo;
	std::vector< std::shared_ptr<ofxOrbbec::DeviceInfo> > retDeviceInfos;

    auto tCtx = make_shared<ob::Context>(); //ofxOrbbecCamera::getContext();
    if( bNetIncDevices ){
		tCtx->enableNetDeviceEnumeration(true); 
	}else{
		tCtx->enableNetDeviceEnumeration(false); 
    }

    // Query the list of connected devices
	auto devList = tCtx->queryDeviceList();
    
    // Get the number of connected devices
    int devCount = devList->deviceCount();

    // traverse the device list and create a pipe
    for(int i = 0; i < devCount; i++) {
        auto dev  = devList->getDevice(i);
        auto info = dev->getDeviceInfo();

        ofLogNotice("ofxOrbbecCamera::getDeviceList()") << "["<< i <<"] device is " << info->name() << " serial: " << info->serialNumber() << std::endl; 
		
		auto dinfo = std::make_shared<ofxOrbbec::DeviceInfo>();
		dinfo->_name = info->name();
		dinfo->_pid = info->pid();
		dinfo->_vid = info->vid();
		dinfo->_uid = info->uid();
		dinfo->_firmwareVersion = info->firmwareVersion();
//		dinfo->_usbType = info->usbType();
		dinfo->_connectionType = info->connectionType();
		dinfo->_ipAddress = info->ipAddress();
		dinfo->_hardwareVersion = info->hardwareVersion();
		dinfo->_supportedMinSdkVersion = info->supportedMinSdkVersion();
		dinfo->_deviceType = info->deviceType();
		dinfo->_serialNumber = info->serialNumber();
		
		retDeviceInfos.push_back(dinfo);
//        dInfo.push_back(info);
    }

    return retDeviceInfos;
}

//Class functions
//-------------------------------------------------------------
ofxOrbbecCamera::~ofxOrbbecCamera(){
    close();
    #if defined(OFXORBBEC_DECODE_H264_H265) && !defined(OFXORBBEC_MACOS_DECODE_VIDEOTOOLBOX)
        if(bInitOneTime){
            avcodec_close(codecContext264);
            av_free(codecContext264);

            avcodec_close(codecContext265);
            av_free(codecContext265);
            bInitOneTime = false; 
        }
    #endif 
}

//-------------------------------------------------------------
void ofxOrbbecCamera::close(){
    clear();
}

//-------------------------------------------------------------
void ofxOrbbecCamera::clear(){

    if( isThreadRunning() ){
        waitForThread(true, 2000);
    }

    // Stop the dedicated point cloud thread (after capture thread so no new signals arrive)
    if( mPointCloudThreadRunning.load() ){
        mPointCloudThreadRunning.store(false);
        mPointCloudCV.notify_one();  // wake it from its wait
        if( mPointCloudThread.joinable() ){
            mPointCloudThread.join();
        }
        ofLogNotice("ofxOrbbecCamera") << "Joining point cloud thread to main thread";
    }
    mPointCloudNewData = false;
    mPCDepthFrame.reset();
    // mPCColorFrame.reset();

    try{
		if( mPipe ){
			mPipe->stop();
			mPipe.reset();
			//pointCloud.reset();
        }
    }catch(ob::Error &e) {
        std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    }

    mCurrentSettings = ofxOrbbec::Settings();
    bNewFrameColor = bNewFrameDepth = bNewFrameIR = false;
    mInternalColorFrameNo.store(0);
    mInternalDepthFrameNo.store(0);
    mExtColorFrameNo.store(0);
    mExtDepthFrameNo.store(0);
    mDepthPixels.clear();
    mDepthPixelsBack.clear();
    mColorPixels.clear();
    mColorPixelsBack.clear();
    mDepthPixelsF.clear();
    mDepthPixelsFBack.clear();
    mPointCloudMesh.clear();
    mPointCloudMeshLocal.clear();
    mPointCloudPts.clear();
    mPointCloudPtsLocal.clear();

    mPipe.reset();
    ctxLocal.reset();
    bConnected = false;
    mTimeSinceFrame = 0.0;
}

//-------------------------------------------------------------
bool ofxOrbbecCamera::open(ofxOrbbec::Settings aSettings){
    clear(); 
	
	if( !aSettings.logFilePath.empty() ) {
//		ob::Context::setLoggerToFile(OB_LOG_SEVERITY_ERROR, "log.txt");
		ob::Context::setLoggerToFile(OB_LOG_SEVERITY_ERROR, aSettings.logFilePath.string().c_str() );
	}
	ob::Context::setLoggerToConsole(aSettings.logLevel);
	
	ctxLocal = make_shared<ob::Context>();
    auto tCtx = ctxLocal;

    std::shared_ptr<ob::Device> device;

    //need depth frames for point cloud
    if( aSettings.bPointCloud && !aSettings.bDepth ){
        aSettings.bDepth = true; 
    }

    mCurrentSettings = aSettings; 

    if( aSettings.ip != ""){
        try{
            device = tCtx->createNetDevice(aSettings.ip.c_str(), 8090);
        }catch(ob::Error &e) {
            std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
        }
        if(!device){
            return false; 
        }
    }else{

        // Query the list of connected devices
         auto devList = tCtx->queryDeviceList();
        
        // Get the number of connected devices
        int devCount = devList->deviceCount();

        bool openWithSerial = aSettings.deviceSerial != "";

        // traverse the device list and create a pipe
        for(int i = 0; i < devCount; i++) {
            // Get the device and create the pipeline
            auto dev  = devList->getDevice(i);
            auto info = dev->getDeviceInfo();

            std::cout << "["<< i <<"] device is " << info->name() << " serial: " << info->serialNumber() << std::endl; 

            if( openWithSerial ){
                string serialStr(info->serialNumber()); 
                if( aSettings.deviceSerial == serialStr ){
                    device = dev;
                    break; 
                }
            }else{
                if( aSettings.deviceID == i ){
                    device = dev; 
                    break; 
                }
            }
        }

    }    

    if( device ){
        // pass in device to create pipeline
        mPipe = std::make_shared<ob::Pipeline>(device);

        auto info = device->getDeviceInfo();
        ofLogNotice() << "Firmware: " << info->firmwareVersion();
        ofLogNotice() << "SDK version: " << ob::Version::getMajor() << "." 
                    << ob::Version::getMinor() << "."
                    << ob::Version::getPatch();
    
        if( mPipe ){

             // Create Config for configuring Pipeline work
            std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

			shared_ptr<ob::StreamProfile> depthProfile;
			shared_ptr<ob::StreamProfile> colorProfile;

            if( aSettings.bColor ){
				
				auto getValidColorProfile = [](std::shared_ptr<ob::StreamProfileList> alist) -> std::shared_ptr<ob::VideoStreamProfile> {
					for(int i = 0; i < alist->count(); i++) {
						try {
							auto p = alist->getProfile(i)->as<ob::VideoStreamProfile>();
							return p;
						}
						catch (...) {
							continue;
						}
					}
					return std::shared_ptr<ob::VideoStreamProfile>();
				};
				
                // Get the color camera configuration list
                auto colorProfileList = mPipe->getStreamProfileList(OB_SENSOR_COLOR);
				
				if (colorProfileList->count() == 0) {
					ofLogError("ofxOrbbecCamera::open") << "no color profiles available!";
					clear();
					return false;
				}
				
//				for(int i = 0; i < colorProfileList->count(); i++) {
//					auto vp = colorProfileList->getProfile(i)->as<ob::VideoStreamProfile>();
//					ofLogNotice("ofxOrbbecCamera") << "Color Profile[" << i << "]: "
//					<< vp->width() << "x" << vp->height()
//					<< " @ " << vp->fps() << "fps"
//					<< " format: " << vp->format();
//				}

                if( mCurrentSettings.colorFrameSize.requestWidth > 0){
                    try {
                        auto requestType = mCurrentSettings.colorFrameSize;

                        // Find the corresponding profile according to the specified format
                        colorProfile = colorProfileList->getVideoStreamProfile(requestType.requestWidth, requestType.requestHeight, requestType.format, requestType.frameRate);
                        // ofLogNotice("ofxOrbbecCamera") << "enabling color stream — format: " 
                        //     << colorProfile->format()
                        //     << " " << colorProfile->as<ob::VideoStreamProfile>()->width()
                        //     << "x" << colorProfile->as<ob::VideoStreamProfile>()->height();
                    }
                    catch(ob::Error &e) {
                        ofLogWarning("ofxOrbbecCamera::open") << " couldn't open color with requested dimensions / format - using default "; 
//                        colorProfile = colorProfileList->getProfile(0);
						colorProfile = getValidColorProfile(colorProfileList);
                    }

                } else {
//                    colorProfile = colorProfileList->getProfile(0);
					colorProfile = getValidColorProfile(colorProfileList);
                }

                if( colorProfile ) {
                    auto videoProfile = colorProfile->as<ob::VideoStreamProfile>();
                    ofLogNotice("ofxOrbbecCamera") << "Color Profile: "
                        << videoProfile->width() << "x" << videoProfile->height()
                        << " @ " << videoProfile->fps() << "fps"
                        << " format: " << videoProfile->format();
					
					// enable color stream
					config->enableStream(colorProfile);
				} else {
					ofLogError("ofxOrbbecCamera::open") << "no usable color profile";
					clear();
					return false;
				}
            }

            if( aSettings.bDepth ){
				
				auto pickDepthFrameForFps = [](shared_ptr<ob::StreamProfile> aColorProfile, shared_ptr<ob::StreamProfileList> aDepthList ) -> shared_ptr<ob::StreamProfile> {
					int targetFPS = OB_FPS_ANY;
					if (aColorProfile) {
						targetFPS = aColorProfile->as<ob::VideoStreamProfile>()->fps();
					}
					
					try {
						auto tdepthProfile = aDepthList->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_ANY, targetFPS);
						ofLogNotice("ofxOrbbecCamera")
						<< "open: no requested depth frame size, matched depth profile to color at "
						<< targetFPS << "fps.";
						return tdepthProfile;
					} catch (ob::Error &e) {
						ofLogWarning("ofxOrbbecCamera")
						<< "open: no depth profile at " << targetFPS
						<< "fps, falling back to first available.";
						return aDepthList->getProfile(0);
					}
					return std::shared_ptr<ob::StreamProfile>();
				};

                if( colorProfile && ((aSettings.bAlignDepthToColor || aSettings.bPointCloudRGB) && aSettings.bColor) ) {
                    OBAlignMode alignMode = ALIGN_D2C_HW_MODE;
                    // Get depth profiles that are compatible with HW alignment to your color profile
                    auto depthProfileList = mPipe->getD2CDepthProfileList(colorProfile, alignMode);
                    // if( depthProfileList->count() > 0 ) {
                    //     ofLogNotice("ofxOrbbecCamera") << "Setting align mode to: ALIGN_D2C_HW_MODE";
                    //     ofLogNotice("ofxOrbbecCamera") << "D2C compatible depth profiles: " << depthProfileList->count();
                    //     for(int i = 0; i < depthProfileList->count(); i++) {
                    //         auto vp = depthProfileList->getProfile(i)->as<ob::VideoStreamProfile>();
                    //         ofLogNotice("ofxOrbbecCamera") << "D2C Depth Profile[" << i << "]: "
                    //             << vp->width() << "x" << vp->height()
                    //             << " @ " << vp->fps() << "fps"
                    //             << " format: " << vp->format();
                    //     }
                    // }

                    if(depthProfileList->count() < 1) {
                        alignMode = ALIGN_D2C_SW_MODE;
                        depthProfileList = mPipe->getD2CDepthProfileList(colorProfile, alignMode);
                        if( depthProfileList->count() > 0 ) {
                            ofLogNotice("ofxOrbbecCamera") << "Setting align mode to: ALIGN_D2C_SW_MODE";
                        }
                    }
                    if( depthProfileList->count() < 1 ) {
                        ofLogNotice("ofxOrbbecCamera") << "Setting align mode to: ALIGN_DISABLE";
                        alignMode = ALIGN_DISABLE;
                    }

                    if(depthProfileList->count() > 0) {
                        //device->setBoolProperty(OB_PROP_DEPTH_SOFT_FILTER_BOOL, false);
                        if(mCurrentSettings.depthFrameSize.requestWidth > 0) {
                            try {
                                auto requestType = mCurrentSettings.depthFrameSize;
                                depthProfile = depthProfileList->getVideoStreamProfile(
                                    requestType.requestWidth,
                                    requestType.requestHeight,
                                    requestType.format,
                                    requestType.frameRate
                                );
                            }
                            catch(ob::Error &e) {
                                ofLogWarning("ofxOrbbecCamera::open") 
                                    << "couldn't open depth with requested dimensions - using first compatible D2C profile";
                                depthProfile = depthProfileList->getProfile(0);
                            }
                        } else {
                            depthProfile = depthProfileList->getProfile(0);
                        }

                        config->setAlignMode(alignMode);
                    } else {
                        config->setAlignMode(ALIGN_DISABLE);
                    }
					
					
					ofLogNotice("ofxOrbbecCamera") << "align mode set to: " << alignMode;

                } else {
                    config->setAlignMode(ALIGN_DISABLE);
                }


                if(!depthProfile) {
                    ofLogNotice("ofxOrbbecCamera") << "open: Going to grab the depth stream from Sensor depth";
                    // Get the depth camera configuration list
                    auto depthProfileList = mPipe->getStreamProfileList(OB_SENSOR_DEPTH);

                    if( mCurrentSettings.depthFrameSize.requestWidth > 0){
                        try {
                            auto requestType = mCurrentSettings.depthFrameSize;

                            // Find the corresponding profile according to the specified format
                            depthProfile = depthProfileList->getVideoStreamProfile(requestType.requestWidth, requestType.requestHeight, requestType.format, requestType.frameRate);
                        }
                        catch(ob::Error &e) {
                            ofLogWarning("ofxOrbbecCamera::open") << " couldn't open depth with requested dimensions / format - using default "; 
							depthProfile = pickDepthFrameForFps(colorProfile, depthProfileList);
                        }

                    }else{
						depthProfile = pickDepthFrameForFps(colorProfile, depthProfileList);
                    }
                }

                if( depthProfile ) {
                    auto videoProfile = depthProfile->as<ob::VideoStreamProfile>();
                    ofLogNotice("ofxOrbbecCamera") << "Depth Profile: "
                        << videoProfile->width() << "x" << videoProfile->height()
                        << " @ " << videoProfile->fps() << "fps"
                        << " format: " << videoProfile->format();
					
					// enable depth stream
					config->enableStream(depthProfile);
                }
                
                
            }

			
			std::shared_ptr<ob::StreamProfile> irProfile;
			if (aSettings.bIR) {
				auto irProfileList = mPipe->getStreamProfileList(OB_SENSOR_IR);
				
				if (depthProfile) {
					// IR shares the ToF capture with depth — must match exactly.
					auto dvp = depthProfile->as<ob::VideoStreamProfile>();
					try {
						irProfile = irProfileList->getVideoStreamProfile(dvp->width(), dvp->height(), OB_FORMAT_ANY, dvp->fps());
					} catch (ob::Error &e) {
						ofLogWarning("ofxOrbbecCamera") << "no IR profile matching depth "
						<< dvp->width() << "x" << dvp->height() << "@" << dvp->fps();
					}
				}
				
				if (!irProfile && irProfileList->count() > 0) {
					irProfile = irProfileList->getProfile(0);
				}
				
				if (irProfile) {
					auto ivp = irProfile->as<ob::VideoStreamProfile>();
					ofLogNotice("ofxOrbbecCamera") << "IR Profile: " << ivp->width() << "x"
					<< ivp->height() << " @" << ivp->fps() << "fps format: " << ivp->format();
					config->enableStream(irProfile);
				} else {
					ofLogError("ofxOrbbecCamera") << "IR requested but no usable profile";
				}
			}
			

            // if( aSettings.bPointCloud || aSettings.bAlignDepthToColor){
            //     device->setBoolProperty(OB_PROP_DEPTH_SOFT_FILTER_BOOL, false);
            //     if( (aSettings.bColor && aSettings.bPointCloudRGB) || (aSettings.bColor && aSettings.bAlignDepthToColor) ){
                    
			// 		// Try find supported depth to color align hardware mode profile
			// 		auto depthProfileList = mPipe->getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
			// 		if(depthProfileList->count() > 0) {
            //             ofLogNotice("ofxOrbbecCamera") << "Setting align mode to: ALIGN_D2C_HW_MODE";
			// 			config->setAlignMode(ALIGN_D2C_HW_MODE);
			// 		}
			// 		else {
			// 			// Try find supported depth to color align software mode profile
			// 			auto depthProfileList = mPipe->getD2CDepthProfileList(colorProfile, ALIGN_D2C_SW_MODE);
			// 			if(depthProfileList->count() > 0) {
            //                 ofLogNotice("ofxOrbbecCamera") << "Setting align mode to: ALIGN_D2C_SW_MODE";
			// 				config->setAlignMode(ALIGN_D2C_SW_MODE);
			// 			}else{
            //                 ofLogNotice("ofxOrbbecCamera") << "Setting align mode to: disabled";
			// 				config->setAlignMode(ALIGN_DISABLE);
			// 			}
			// 		}
                    
            //     }else{
            //         config->setAlignMode(ALIGN_DISABLE);
			// 	}
            // }
			
//			if( aSettings.bAlignDepthToColor ) {
//				// Set alignment mode: align depth → color if OB_STREAM_COLOR
//				// Tell SDK to align DEPTH to COLOR (so depth map matches RGB resolution and viewpoint)
//				mObAlignToColor = std::make_shared<ob::Align>(OB_STREAM_COLOR);
//			}
            
			
            // Pass in the configuration and start the pipeline
			try {

                if(aSettings.bResetCameraClock) {
                    ofLogNotice("ofxOrbbecCamera") << "Attempting to reset camera clock.";
                    device->setBoolProperty(OB_PROP_TIMER_RESET_ENABLE_BOOL, true);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    device->setBoolProperty(OB_PROP_TIMER_RESET_SIGNAL_BOOL, true);
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                }

				mPipe->start(config);
                mPipe->enableFrameSync();
			} catch(ob::Error &e) {
				ofLogError("ofxOrbbecCamera") << "ERROR STARTING camera: "
				<< e.getMessage()
				<< " | function: " << e.getName()
				<< " | args: "     << e.getArgs()
				<< " | type: "     << e.getExceptionType();
				clear();
				return false;
			} catch(std::exception &e) {
				ofLogError("ofxOrbbecCamera") << "ERROR STARTING camera: " << e.what();
				clear();
				return false;
			} catch(...) {
				ofLogError("ofxOrbbecCamera") << "ERROR STARTING camera";
				clear();
				return false;
			}

            // Pre-allocate front and back pixel buffers at the correct resolution
            // so that setFromPixels() never triggers a heap allocation mid-capture.
            if( aSettings.bDepth && depthProfile ){
                auto vsp = depthProfile->as<ob::VideoStreamProfile>();
                mDepthPixels.allocate(vsp->width(), vsp->height(), OF_IMAGE_GRAYSCALE);
                mDepthPixelsBack.allocate(vsp->width(), vsp->height(), OF_IMAGE_GRAYSCALE);
                mDepthPixelsF.allocate(vsp->width(), vsp->height(), OF_IMAGE_GRAYSCALE);
                mDepthPixelsFBack.allocate(vsp->width(), vsp->height(), OF_IMAGE_GRAYSCALE);
            }
            if( aSettings.bColor && colorProfile ){
                auto vsp = colorProfile->as<ob::VideoStreamProfile>();
                mColorPixels.allocate(vsp->width(), vsp->height(), OF_IMAGE_COLOR);
                mColorPixelsBack.allocate(vsp->width(), vsp->height(), OF_IMAGE_COLOR);
            }
            if( aSettings.bPointCloud || aSettings.bPointCloudRGB ){
                mPointCloudMesh.setMode(OF_PRIMITIVE_POINTS);
                mPointCloudMeshLocal.setMode(OF_PRIMITIVE_POINTS);
            }
			
			if (aSettings.bIR && irProfile) {
				auto ivp = irProfile->as<ob::VideoStreamProfile>();
				mIRPixels.allocate(ivp->width(), ivp->height(), OF_IMAGE_GRAYSCALE);
				mIRPixelsBack.allocate(ivp->width(), ivp->height(), OF_IMAGE_GRAYSCALE);
			}

            if( aSettings.bPointCloud || aSettings.bPointCloudRGB ){
                auto cameraParam = mPipe->getCameraParam();

                //pointCloud = std::make_shared<ob::PointCloudFilter>();
                //pointCloud->setCameraParam(cameraParam);
                
                if( aSettings.bPointCloudRGB ){
                    //pointCloud->setCreatePointFormat(OB_FORMAT_RGB_POINT);
					auto param = mPipe->getCalibrationParam(config);
					
					ofLogNotice("ofxOrbbecCamera") << "rgb intrinsic: "
					<< param.intrinsics[OB_SENSOR_COLOR].width << "x" << param.intrinsics[OB_SENSOR_COLOR].height
					<< " fx:" << param.intrinsics[OB_SENSOR_COLOR].fx
					<< " cx:" << param.intrinsics[OB_SENSOR_COLOR].cx;
										
					auto     vsp            = colorProfile->as<ob::VideoStreamProfile>();
					uint32_t colorWidth     = vsp->width();
					uint32_t colorHeight    = vsp->height();
					uint32_t tableSize = colorWidth * colorHeight * 2;// * sizeof(float);
					xyTableData.resize(tableSize);

                    ofLogNotice("ofxOrbbecCamera") << "setting up point cloud tables: " << colorWidth << " x " << colorHeight;
					
					if(!ob::CoordinateTransformHelper::transformationInitXYTables(param, OB_SENSOR_COLOR, &xyTableData[0], &tableSize, &xyTables)) {
						ofLogError() << " couldn't init xyTables for depth " << endl;
					}
					
                }else{
                    //pointCloud->setCreatePointFormat(OB_FORMAT_POINT);
                    
					auto param = mPipe->getCalibrationParam(config);
					auto     vsp            = depthProfile->as<ob::VideoStreamProfile>();
					uint32_t depthWidth     = vsp->width();
					uint32_t depthHeight    = vsp->height();
					uint32_t tableSize = depthWidth * depthHeight * 2 * sizeof(float);
					xyTableData.resize(tableSize);

                    ofLogNotice("ofxOrbbecCamera") << "setting up point cloud tables: " << depthWidth << " x " << depthHeight;
					
					if(!ob::CoordinateTransformHelper::transformationInitXYTables(param, OB_SENSOR_DEPTH, &xyTableData[0], &tableSize, &xyTables)) {
						ofLogError() << " couldn't init xyTables for depth " << endl;
					}

                }
            }

            ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_ERROR);
            bConnected = true;

            // Start the dedicated point cloud thread before the capture thread
            if( aSettings.bPointCloud || aSettings.bPointCloudRGB ){
                mPointCloudThreadRunning.store(true);
                ofLogNotice("ofxOrbbecCamera") << "Starting the point cloud thread!";
                mPointCloudThread = std::thread(&ofxOrbbecCamera::pointCloudThreadFunc, this);
            }

            startThread();

        } else {
			clear();
            return false;
        }

    }

    return true; 
}

//-------------------------------------------------------------
void ofxOrbbecCamera::setPropertyInt(int propEnum, int value){
	if( mPipe && mPipe->getDevice() ){
		mPipe->getDevice()->setIntProperty((OBPropertyID)propEnum, value);
	}
}

//-------------------------------------------------------------
bool ofxOrbbecCamera::isConnected(){
	if( mPipe && mPipe->getDevice() ){
        return bConnected; 
	}
	return false;
}

//-------------------------------------------------------------
ofPixels ofxOrbbecCamera::getDepthPixels(){
    std::lock_guard<std::mutex> lock(mFrameMutex);
    mExtDepthFrameNo = mInternalDepthFrameNo.load();
    return mDepthPixels;
}

//-------------------------------------------------------------
ofFloatPixels ofxOrbbecCamera::getDepthPixelsF(){
    std::lock_guard<std::mutex> lock(mFrameMutex);
    mExtDepthFrameNo = mInternalDepthFrameNo.load();
    return mDepthPixelsF;
}

//-------------------------------------------------------------
ofPixels ofxOrbbecCamera::getColorPixels(){
    std::lock_guard<std::mutex> lock(mFrameMutex);
    mExtColorFrameNo = mInternalColorFrameNo.load();
    return mColorPixels;
}

//-------------------------------------------------------------
ofPixels ofxOrbbecCamera::getIRPixels() {
	std::lock_guard<std::mutex> lock(mFrameMutex);
	mExtIRFrameNo = mInternalIRFrameNo.load();
	return mIRPixels;
}

//-------------------------------------------------------------
vector<glm::vec3> ofxOrbbecCamera::getPointCloud(){
    std::lock_guard<std::mutex> lock(mFrameMutex);
    mExtDepthFrameNo = mInternalDepthFrameNo.load();
    return mPointCloudPtsLocal;
}

//-------------------------------------------------------------
ofMesh ofxOrbbecCamera::getPointCloudMesh(){
    std::lock_guard<std::mutex> lock(mFrameMutex);
    mExtDepthFrameNo = mInternalDepthFrameNo.load();
    return mPointCloudMeshLocal;
}

//-------------------------------------------------------------
int ofxOrbbecCamera::getMinDistance() {
	return mMinDistanceMM.load();
}

//-------------------------------------------------------------
void ofxOrbbecCamera::setMinDistance( int adistInMM ) {
	mMinDistanceMM = adistInMM;
}

//-------------------------------------------------------------
int ofxOrbbecCamera::getMaxDistance() {
	return mMaxDistanceMM.load();
}

//-------------------------------------------------------------
void ofxOrbbecCamera::setMaxDistance( int adistInMM ) {
	mMaxDistanceMM = adistInMM;
}

//-------------------------------------------------------------
int ofxOrbbecCamera::getMaxIRValue() {
	return mIRMaxValue.load();
}

//-------------------------------------------------------------
void ofxOrbbecCamera::setMaxIRValue( int aMaxIR ) {
	mIRMaxValue = aMaxIR;
}

//-------------------------------------------------------------
void ofxOrbbecCamera::update(){
    if( mPipe ){
        bNewFrameDepth = bNewFrameColor = bNewFrameIR = false; 
        if( mInternalDepthFrameNo > mExtDepthFrameNo ){
            bNewFrameDepth = true; 
//            bNewFrameIR = true; 
        }
		if( mInternalIRFrameNo > mExtIRFrameNo ) {
			bNewFrameIR = true;
		}
        if( mInternalColorFrameNo > mExtColorFrameNo ){
            bNewFrameColor = true; 
        }
        if( bNewFrameColor || bNewFrameDepth || bNewFrameIR ){
            mTimeSinceFrame = 0; 
            bConnected = true; 
        }else{
            mTimeSinceFrame += ofClamp(ofGetLastFrameTime(), 1.0/250.0, 1.0/5.0); 
        }
        if( bConnected && mTimeSinceFrame > 5.0 ){
            bConnected = false; 
        }
    }
}

//-------------------------------------------------------------
void ofxOrbbecCamera::threadedFunction(){
    while(isThreadRunning()){
        if( mPipe ){
            auto frameSet = mPipe->waitForFrames(100);
            if(frameSet) {

                bool bGotDepth = false;
                bool bGotColor = false;
				bool bGotIR = false;

                // ── Phase 1: decode both frames into back buffers ─────────────────
                if( mCurrentSettings.bDepth ){
                    auto depthFrame = frameSet->getFrame(OB_FRAME_DEPTH);
                    if(depthFrame) {
                        mDepthPixelsBack = processFrame(depthFrame);
                        bGotDepth = true;
                    }
                }

                if( mCurrentSettings.bColor ){
                    auto colorFrame = frameSet->getFrame(OB_FRAME_COLOR);
                    if(colorFrame) {
                        mColorPixelsBack = processFrame(colorFrame);
                        if(mColorPixelsBack.getWidth()){
                            bGotColor = true;
                        }
                    }
                }
				
				if (mCurrentSettings.bIR) {
					auto irFrame = frameSet->getFrame(OB_FRAME_IR);
					if (irFrame) {
						mIRPixelsBack = processIRFrame(irFrame);
						bGotIR = mIRPixelsBack.getWidth() > 0;
					}
				}

                // ── Phase 2: immediately swap pixels to front ─────────────────────
                // Main thread gets fresh color/depth NOW — no waiting for point cloud.
                {
                    std::lock_guard<std::mutex> lock(mFrameMutex);
                    if(bGotDepth) {
                        std::swap(mDepthPixels, mDepthPixelsBack);
                        std::swap(mDepthPixelsF, mDepthPixelsFBack);
                        mInternalDepthFrameNo++;
                    }
                    if(bGotColor) {
                        std::swap(mColorPixels, mColorPixelsBack);
                        mInternalColorFrameNo++;
                    }
					if(bGotIR) {
						std::swap(mIRPixels, mIRPixelsBack);
						mInternalIRFrameNo++;
					}
                }

                // ── Phase 3: hand off to the dedicated point cloud thread ─────────
                // The PC thread runs independently — if it's still busy with the
                // previous frame the capture loop keeps going at full frame rate.
                bool bNeedPC = false;
                if( bGotDepth && mCurrentSettings.bPointCloud && !mCurrentSettings.bPointCloudRGB ){
                    bNeedPC = (frameSet->depthFrame() != nullptr);
                }
                // if( bGotColor && mCurrentSettings.bPointCloudRGB ){
                if( mCurrentSettings.bPointCloudRGB ){
                    // bNeedPC = (frameSet->depthFrame() != nullptr && frameSet->colorFrame() != nullptr
                    //            && mColorPixels.isAllocated() && mColorPixels.getWidth() > 0);
                    bNeedPC = (frameSet->depthFrame() != nullptr && mColorPixels.isAllocated() && mColorPixels.getWidth() > 0);
                }

                if( bNeedPC ){
                    std::lock_guard<std::mutex> lk(mPointCloudInputMutex);
                    mPCDepthFrame = frameSet->depthFrame();
                    //mPCColorFrame = (mCurrentSettings.bPointCloudRGB) ? frameSet->colorFrame() : nullptr;
                    mPointCloudNewData = true;
                    mPointCloudCV.notify_one();
                }
            }
        }
    }
}

//-------------------------------------------------------------
void ofxOrbbecCamera::pointCloudThreadFunc(){
    // Persistent local buffers — reuse allocations across iterations
    std::vector<uint8_t> localDepthData;
    ofPixels localColorPixels;
    
    while(mPointCloudThreadRunning.load()){
        std::shared_ptr<ob::DepthFrame> depthFrame;
        //std::shared_ptr<ob::ColorFrame> colorFrame;

        // Wait for the capture thread to hand us a new frameset
        {
            std::unique_lock<std::mutex> lk(mPointCloudInputMutex);
            mPointCloudCV.wait(lk, [this]{
                return mPointCloudNewData || !mPointCloudThreadRunning.load();
            });
            if(!mPointCloudThreadRunning.load()) break;
            depthFrame = mPCDepthFrame;
            //colorFrame = mPCColorFrame;
            mPCDepthFrame.reset();
            //mPCColorFrame.reset();
            mPointCloudNewData = false;
        }

        // bool bRGB = (colorFrame != nullptr);
        bool bRGB = mCurrentSettings.bPointCloudRGB && mColorPixels.isAllocated();

        // ── Extract raw data we need, then release SDK frames immediately ────
        // The Orbbec SDK has a finite internal frame buffer pool.  Holding onto
        // frame shared_ptrs during the long point-cloud build (~100-300ms) can
        // starve the SDK and stall waitForFrames on the capture thread.

        // 1) Copy raw depth bytes (only reallocates on first frame / resolution change)
        size_t depthBytes = depthFrame->dataSize();
        int depthW = depthFrame->width();
        int depthH = depthFrame->height();
        localDepthData.resize(depthBytes);
        memcpy(localDepthData.data(), depthFrame->data(), depthBytes);

        // 2) Snapshot the decoded colour pixels (RGB path only).
        //    mColorPixels is the front buffer written by the capture thread under
        //    mFrameMutex.  A brief lock here is fine — it only blocks main-thread
        //    getters / capture-thread Phase-2 for the duration of a memcpy.
        if( bRGB ){
            std::lock_guard<std::mutex> lock(mFrameMutex);
            localColorPixels = mColorPixels;
        }

        // 3) Release the SDK frames — buffer pool is now free for the capture thread
        depthFrame.reset();
        //colorFrame.reset();

        // ── Build the point cloud from local copies ──────────────────────────
        bool bBuilt = false;
        try {
            pointCloudToMesh(localDepthData.data(), depthW, depthH,
                             bRGB ? &localColorPixels : nullptr);
            bBuilt = true;
        }
        catch(std::exception &e) {
            std::cout << "Point cloud build failed: " << e.what() << std::endl;
        }

        // Swap the finished mesh to the front buffers
        if( bBuilt ){
            std::lock_guard<std::mutex> lock(mFrameMutex);
            std::swap(mPointCloudMesh, mPointCloudMeshLocal);
            std::swap(mPointCloudPts, mPointCloudPtsLocal);
        }
    }
}

//-------------------------------------------------------------
bool ofxOrbbecCamera::isFrameNew(){
    return bNewFrameColor || bNewFrameDepth || bNewFrameIR;
}

//-------------------------------------------------------------
bool ofxOrbbecCamera::isFrameNewDepth(){
    return bNewFrameDepth;
}

//-------------------------------------------------------------
bool ofxOrbbecCamera::isFrameNewColor(){
    return bNewFrameColor;
}

//-------------------------------------------------------------
bool ofxOrbbecCamera::isFrameNewIR() {
	return bNewFrameIR;
}


#if defined(OFXORBBEC_DECODE_H264_H265) && !defined(OFXORBBEC_MACOS_DECODE_VIDEOTOOLBOX)
//-------------------------------------------------------------
void ofxOrbbecCamera::initH26XCodecs(){
    if( !bInitOneTime ){
		// Initialize FFmpeg libraries
	#if LIBAVCODEC_VERSION_MAJOR < 58
		// FFmpeg < 4.0
		av_register_all();
		avcodec_register_all();
	#endif
		bInitOneTime = true;

		// Allocate an AVCodecContext and set its codec
		codec264 = avcodec_find_decoder(AV_CODEC_ID_H264);
        if( !codec264 ) { ofLogError("ofxOrbbecCamera") << "H264 decoder not found!"; }
        codecContext264 = avcodec_alloc_context3(codec264);
        if( !codecContext264 ) { ofLogError("ofxOrbbecCamera") << "H264 context alloc failed!"; }
        // These are often needed for stream-based H264 (no container)
        codecContext264->flags2 |= AV_CODEC_FLAG2_CHUNKS;
        codecContext264->thread_count = 1; // disable threading — common Linux crash source
        // avcodec_open2(codecContext264, codec264, NULL);
        int ret = avcodec_open2(codecContext264, codec264, NULL);
        if( ret < 0 ) {
            char errbuf[256];
            av_strerror(ret, errbuf, sizeof(errbuf));
            ofLogError("ofxOrbbecCamera") << "H264 avcodec_open2 failed: " << errbuf;
            return;
        }
        ofLogNotice("ofxOrbbecCamera") << "H264 decoder initialized ok";
        

        // // Allocate an AVCodecContext and set its codec
        // codec265 = avcodec_find_decoder(AV_CODEC_ID_H265);
        // codecContext265 = avcodec_alloc_context3(codec265);
        // avcodec_open2(codecContext265, codec265, NULL);
        codec265 = avcodec_find_decoder(AV_CODEC_ID_H265);
        if( !codec265 ) { ofLogError("ofxOrbbecCamera") << "H265 decoder not found!"; return; }
        
        codecContext265 = avcodec_alloc_context3(codec265);
        if( !codecContext265 ) { ofLogError("ofxOrbbecCamera") << "H265 context alloc failed!"; return; }
        
        codecContext265->flags2 |= AV_CODEC_FLAG2_CHUNKS;
        codecContext265->thread_count = 1;
        
        ret = avcodec_open2(codecContext265, codec265, NULL);
        if( ret < 0 ) {
            char errbuf[256];
            av_strerror(ret, errbuf, sizeof(errbuf));
            ofLogError("ofxOrbbecCamera") << "H265 avcodec_open2 failed: " << errbuf;
            return;
        }
        ofLogNotice("ofxOrbbecCamera") << "H265 decoder initialized ok";

        // Suppress "Could not find ref with POC" spam — these are handled via
        // decode_error_flags in decodeH26XFrame and don't need to fill the console.
        av_log_set_level(AV_LOG_FATAL);
    }
}

//-------------------------------------------------------------
ofPixels ofxOrbbecCamera::decodeH26XFrame(uint8_t * myData, int dataSize, bool bH264){
    initH26XCodecs();
    
    AVPacket packet; 
    av_init_packet(&packet);
    packet.data = myData;
    packet.size = dataSize;

    // Allocate an AVFrame for decoded data
    AVFrame* frame = av_frame_alloc();
    
    ofPixels pix;

    auto codecContext = codecContext264;
    if( !bH264 ){
        codecContext = codecContext265; 
    }

    int ret = avcodec_send_packet(codecContext, &packet);
    if (ret < 0) {
        // Flush decoder so it can resync on the next IDR keyframe
        avcodec_flush_buffers(codecContext);
        av_frame_free(&frame);
        return pix;
    }

    int frameDecoded = avcodec_receive_frame(codecContext, frame);

    if( frameDecoded == 0 ){

        // AV_DECODE_ERROR_MISSING_REFERENCE means the decoder received inter-frames
        // (P/B frames) before it had the keyframe they reference — causes gray output.
        // Flush the context so it discards stale state and resyncs on the next IDR.
        if( frame->decode_error_flags & FF_DECODE_ERROR_MISSING_REFERENCE ){
            avcodec_flush_buffers(codecContext);
            av_frame_free(&frame);
            return pix;
        }

        // Allocate an AVFrame for RGB data
        AVFrame* rgbFrame = av_frame_alloc();

        rgbFrame->format = AV_PIX_FMT_RGB24; 
        rgbFrame->width = codecContext->width; 
        rgbFrame->height = codecContext->height; 

        av_frame_get_buffer(rgbFrame, 0);

        // Create a sws context for RGB conversion
        swsContext = sws_getCachedContext(swsContext, codecContext->width, codecContext->height, codecContext->pix_fmt,
            codecContext->width, codecContext->height, (AVPixelFormat)rgbFrame->format, SWS_BILINEAR, NULL, NULL, NULL);
        
        // Convert the decoded frame to RGB
        int result = sws_scale(swsContext, frame->data, frame->linesize, 0, frame->height, rgbFrame->data, rgbFrame->linesize);
        pix.setFromPixels((unsigned char * )rgbFrame->data[0], codecContext->width, codecContext->height, 3); 

        av_frame_free(&rgbFrame);
    }

    // Clean up and free allocated memory
    av_frame_free(&frame);

    return pix; 
}

#endif 

//-------------------------------------------------------------
ofPixels ofxOrbbecCamera::processFrame(shared_ptr<ob::Frame> frame){

    ofPixels pix; 
    cv::Mat imuMat;
    cv::Mat rstMat;

    try{
        
        if( !frame ){
            return pix; 
        }

        if(frame->type() == OB_FRAME_COLOR) {
            auto videoFrame = frame->as<ob::VideoFrame>();
            switch(videoFrame->format()) {
            case OB_FORMAT_H264:

                #if defined(OFXORBBEC_DECODE_H264_H265)
					#if defined(OFXORBBEC_MACOS_DECODE_VIDEOTOOLBOX)
					if(mVTDecoder.decode((uint8_t*)videoFrame->data(), videoFrame->dataSize(), true)){
						pix = mVTDecoder.getPixels();
					}
					#else
                    pix = decodeH26XFrame((uint8_t*)videoFrame->data(), videoFrame->dataSize(), true);
					#endif
                #else
                    ofLogError("ofxOrbbecCamera::processFrame") << " h264 / h265 not enabled. Define OFXORBBEC_DECODE_H264_H265 or set color format to OB_FORMAT_RGB " << endl;
                #endif  

            break; 
            case OB_FORMAT_H265:

                #ifdef OFXORBBEC_DECODE_H264_H265
					#if defined(OFXORBBEC_MACOS_DECODE_VIDEOTOOLBOX)
					if(mVTDecoder.decode((uint8_t*)videoFrame->data(), videoFrame->dataSize(), false )){
						pix = mVTDecoder.getPixels();
					}
					#else
                    pix = decodeH26XFrame((uint8_t*)videoFrame->data(), videoFrame->dataSize(), false);
					#endif
                #else
                    ofLogError("ofxOrbbecCamera::processFrame") << " h264 / h265 not enabled. Define OFXORBBEC_DECODE_H264_H265 or set color format to OB_FORMAT_RGB " << endl;
                #endif 

            break; 
            case OB_FORMAT_MJPG: {

				#if !defined(TARGET_OSX) && !defined(TARGET_WIN32)
                cv::Mat rawMat(1, videoFrame->dataSize(), CV_8UC1, videoFrame->data());
                rstMat = cv::imdecode(rawMat, 1);
                cv::cvtColor(rstMat, rstMat, cv::COLOR_BGR2RGB);
				#else
                ofLogError("ofxOrbbecCamera::processFrame") << " MJPG not supported - set color format to OB_FORMAT_RGB " << endl;
				#endif

            } break;
            case OB_FORMAT_NV21: {
                cv::Mat rawMat(videoFrame->height() * 3 / 2, videoFrame->width(), CV_8UC1, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2RGB_NV21);
            } break;
            case OB_FORMAT_YUYV:
            case OB_FORMAT_YUY2: {
                cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC2, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2RGB);
            } break;
            case OB_FORMAT_RGB: {
                cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC3, videoFrame->data());
                rstMat = rawMat;
            } break;
            case OB_FORMAT_UYVY: {
                cv::Mat rawMat(videoFrame->height(), videoFrame->width(), CV_8UC2, videoFrame->data());
                cv::cvtColor(rawMat, rstMat, cv::COLOR_YUV2RGB_UYVY);
            } break;
            default:
                break;
            }
            if(!rstMat.empty()) {
                pix.setFromPixels(rstMat.ptr(), rstMat.cols, rstMat.rows, rstMat.channels());
            }
        }
        else if(frame->type() == OB_FRAME_DEPTH) {
            auto videoFrame = frame->as<ob::VideoFrame>();
			if(videoFrame->format() == OB_FORMAT_Y16) {
				cv::Mat cvtMat;
				cv::Mat rawMat = cv::Mat(videoFrame->height(), videoFrame->width(), CV_16UC1, videoFrame->data());
				// depth frame pixel value multiply scale to get distance in millimeter
//				float scale = videoFrame->as<ob::DepthFrame>()->getValueScale();
				
				// threshold to 5.46m
//                cv::threshold(rawMat, cvtMat, 5460.0f / scale, 0, cv::THRESH_TRUNC);
//                cvtMat.convertTo(cvtMat, CV_8UC1, scale * 0.05);
//				rstMat = cvtMat;//cv::applyColorMap(cvtMat, rstMat, cv::COLORMAP_JET);
				
				// build a mask: keep pixels with depth in [near, far] and depth != 0
				const uint16_t near_mm = mMinDistanceMM.load();   // 0.0 m
				const uint16_t far_mm  = mMaxDistanceMM.load();  // 5.46m
				
				cv::Mat depthF;
				rawMat.convertTo(depthF, CV_32F); // in mm as float
				
				cv::Mat norm = (float(far_mm) - depthF) / float(far_mm - near_mm);  // near -> ~1, far -> 0
				
				// Clamp to [0,1]
				cv::max(norm, 0.0, norm);
				cv::min(norm, 1.0, norm);
				
				// Make invalid depths (0) also 0 in the normalized output (optional)
				norm.setTo(0.0f, rawMat == 0);
				
				// ensure contiguous buffer for ofFloatPixels
				if (!norm.isContinuous()) {
					norm = norm.clone();
				}
				
				if( !norm.empty() ) {
					mDepthPixelsFBack.setFromPixels(norm.ptr<float>(0), norm.cols, norm.rows, norm.channels());
				}
				
				norm.convertTo(rstMat, CV_8UC1, 255.0); // 0..255
			}
				
            if(!rstMat.empty()) {
                pix.setFromPixels(rstMat.ptr(), rstMat.cols, rstMat.rows, rstMat.channels());
            }
        }
        else if(frame->type() == OB_FRAME_IR || frame->type() == OB_FRAME_IR_LEFT || frame->type() == OB_FRAME_IR_RIGHT) {
            auto videoFrame = frame->as<ob::VideoFrame>();
            if(videoFrame->format() == OB_FORMAT_Y16) {
                cv::Mat cvtMat;
                cv::Mat rawMat = cv::Mat(videoFrame->height(), videoFrame->width(), CV_16UC1, videoFrame->data());
                rawMat.convertTo(cvtMat, CV_8UC1, 1.0 / 16.0f);
                rstMat = rawMat;//cv::cvtColor(cvtMat, rstMat, cv::COLOR_GRAY2RGB);
            }
            else if(videoFrame->format() == OB_FORMAT_Y8) {
                cv::Mat rawMat = cv::Mat(videoFrame->height(), videoFrame->width(), CV_8UC1, videoFrame->data());
                rstMat = rawMat;//cv::cvtColor(rawMat * 2, rstMat, cv::COLOR_GRAY2RGB);
            }
            else if(videoFrame->format() == OB_FORMAT_MJPG) {
				#if !defined(TARGET_OSX) && !defined(TARGET_WIN32)
                cv::Mat rawMat(1, videoFrame->dataSize(), CV_8UC1, videoFrame->data());
                rstMat = cv::imdecode(rawMat, 1);
                rstMat = rawMat;//cv::cvtColor(rstMat * 2, rstMat, cv::COLOR_GRAY2RGB);
				#else
                ofLogError("ofxOrbbecCamera::processFrame") << " MJPG not supported - set IR format to OB_FORMAT_Y16 or OB_FORMAT_Y8 " << endl;
				#endif
            }
            if(!rstMat.empty()) {
                pix.setFromPixels(rstMat.ptr(), rstMat.cols, rstMat.rows, rstMat.channels());
            }
        }
    } catch(const cv::Exception& ex) {
        ofLogError("processFrame") << " OB_FORMAT not supported " << std::endl; 
    }
    return pix; 
}

//-------------------------------------------------------------
ofPixels ofxOrbbecCamera::processIRFrame(std::shared_ptr<ob::Frame> frame) {
	auto vf = frame->as<ob::VideoFrame>();
	const int w = vf->width(), h = vf->height();
	
	ofPixels out;
	out.allocate(w, h, OF_IMAGE_GRAYSCALE);
	
	const uint16_t * src = (const uint16_t *)vf->data();
	uint8_t * dst = out.getData();
	const int n = w * h;
	
	// Fixed-range mapping. mIRMaxValue defaults to ~1000 for indoor scenes;
	const float scale = 255.f / float(std::max(1, mIRMaxValue.load()));
	for (int i = 0; i < n; i++) {
		dst[i] = (uint8_t)std::min(255.f, src[i] * scale);
	}
	return out;
}

//-------------------------------------------------------------
void ofxOrbbecCamera::pointCloudToMesh(void* depthData, int depthWidth, int depthHeight, ofPixels* colorPixels){
    if( !depthData ) return;

    bool bRGB = (colorPixels != nullptr && colorPixels->isAllocated());

    int numPoints = 0;
    uint32_t pointcloudSize = 0;

    if(bRGB){
        numPoints = colorPixels->getWidth() * colorPixels->getHeight();
        pointcloudSize = numPoints * sizeof(OBColorPoint);
    }else{
        numPoints = depthWidth * depthHeight;
        pointcloudSize = numPoints * sizeof(OBPoint);
    }

    if( mPointcloudData.size() != pointcloudSize){
        mPointcloudData.resize(pointcloudSize);
    }

    if( numPoints < 10 ) {
        ofLogWarning("ofxOrbbecCamera") << "number of points to calculate too low, skipping point cloud";
        return;
    }

    if( xyTables.xTable == nullptr || xyTables.yTable == nullptr ) {
        ofLogWarning("ofxOrbbecCamera") << "xyTables not initialized, skipping point cloud";
        return;
    }

    // Direct-write into the mesh's internal vectors — eliminates the full dealloc+realloc
    // of `mPointCloudMesh = ofMesh()`, the per-element push_backs, and the addVertices /
    // addColors copy passes (~60MB of redundant memory writes per 1080p frame).
    // resize() only reallocates on the first frame or a resolution change; subsequent
    // frames reuse the existing heap allocation, so steady-state overhead is near-zero.
    // Mesh mode (OF_PRIMITIVE_POINTS) was set in open() and persists across swaps.
    auto& meshVerts  = mPointCloudMesh.getVertices();
    auto& meshColors = mPointCloudMesh.getColors();
    meshVerts.resize(numPoints);

    // Fast byte→float colour conversion: one multiply instead of constructing an
    // ofColor (uint8) and then converting it to ofFloatColor implicitly.
    constexpr float inv255 = 1.f / 255.f;

    if( bRGB ){
		
        meshColors.resize(numPoints);

        OBColorPoint *point = (OBColorPoint *)&mPointcloudData[0];
        ob::CoordinateTransformHelper::transformationDepthToRGBDPointCloud(
            &xyTables, depthData, colorPixels->getData(), point);

        point = (OBColorPoint *)&mPointcloudData[0];

        for(int i = 0; i < numPoints; i++) {
            meshVerts[i]  = glm::vec3(point->x, -point->y, -point->z);
            meshColors[i] = ofFloatColor(point->r * inv255, point->g * inv255, point->b * inv255, 1.f);
            point++;
        }

    }else{
        OBPoint *point = (OBPoint *)&mPointcloudData[0];
        ob::CoordinateTransformHelper::transformationDepthToPointCloud(
            &xyTables, depthData, point);

        point = (OBPoint *)&mPointcloudData[0];

        for(int i = 0; i < numPoints; i++) {
            meshVerts[i] = glm::vec3(point->x, -point->y, -point->z);
            point++;
        }
    }

    // One vector copy for the getPointCloud() API
    mPointCloudPts = meshVerts;
    // Swap to front buffers (mPointCloudMeshLocal / mPointCloudPtsLocal) is
    // handled in pointCloudThreadFunc under mFrameMutex.
}
