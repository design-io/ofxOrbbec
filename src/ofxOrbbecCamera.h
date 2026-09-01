//#include "ofMain.h"

#include "ofPixels.h"
#include "ofMesh.h"
#include "ofThread.h"

#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"
#include <opencv2/opencv.hpp>
#include <mutex>
#include <atomic>
#include <thread>
#include <condition_variable>


//If you have ffmpeg / libavcodec included in your project uncomment below 
//You can easily get the required libs from ofxFFmpegRTSP addon ( if you add it to your project )
#define OFXORBBEC_DECODE_H264_H265
#ifdef TARGET_OSX
// comment out to use ffmpeg decoding on macOS.
// must have the ffmpeg libs linked.
#define OFXORBBEC_MACOS_DECODE_VIDEOTOOLBOX
#endif

// this allows us to decode the color video streams from Femto Mega over IP connection 
#ifdef OFXORBBEC_DECODE_H264_H265
#ifdef OFXORBBEC_MACOS_DECODE_VIDEOTOOLBOX
#include "ofxOrbbecH264Decoder.h"
#else
    extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
    #include <libavutil/imgutils.h>
    }
#endif
#endif

namespace ofxOrbbec{

struct Settings{

    struct FrameType{
        int requestWidth = 0; //needed
        int requestHeight = 0;//usually not needed, width is enough 
        OBFormat format = OB_FORMAT_UNKNOWN; //DEPTH: OB_FORMAT_Y16 COLOR: OB_FORMAT_RGB or OB_FORMAT_MJPG 
        int frameRate = 30;
    };

    std::string ip = "";
    int deviceID = 0;
    std::string deviceSerial = "";

    FrameType depthFrameSize;
    FrameType colorFrameSize; 
    
    bool bColor = false;
    bool bDepth = false; 
    bool bPointCloud = false; 
    bool bPointCloudRGB = false;
	bool bAlignDepthToColor = false;
	bool bIR = false;
	
    bool bResetCameraClock = false;
	
	std::filesystem::path logFilePath;
	OBLogSeverity logLevel = OB_LOG_SEVERITY_ERROR;
};

// copying structure of ob::DeviceInfo so that we can return a device list
// without storing a context in the class.
// the ob device list owns the devices and they are unreliable when the ob::device list goes out of scope
// specifically for getDeviceList
// https://orbbec.github.io/OrbbecSDK/doc/api/English/classob_1_1DeviceInfo.html
struct DeviceInfo {
	
	std::string& name() {
		return _name;
	}
	
	int& pid() {return _pid;}
	int& vid() { return _vid; }
	std::string& uid() { return _uid; }
	std::string& serialNumber() { return _serialNumber; }
	
	std::string& firmwareVersion() { return _firmwareVersion;}
	std::string& hardwareVersion() { return _hardwareVersion;}
	std::string& supportedMinSdkVersion() { return _supportedMinSdkVersion;}
	
//	std::string& usbType() { return _usbType;}
	std::string& connectionType() { return _connectionType;}
	
	std::string& ipAddress() {return _ipAddress;}
	OBDeviceType& deviceType() { return _deviceType; }
	
	std::string _name;
	int _pid =0;
	int _vid = 0;
	std::string _uid;
	std::string _serialNumber;
	std::string _firmwareVersion;
	std::string _hardwareVersion;
	std::string _supportedMinSdkVersion;
//	std::string _usbType; // deprecated
	std::string _connectionType;
	std::string _ipAddress;
	OBDeviceType _deviceType;
};

};


class ofxOrbbecCamera : public ofThread{
    public:

        ofxOrbbecCamera() = default; 
		ofxOrbbecCamera( const ofxOrbbecCamera & A) = delete;
        ~ofxOrbbecCamera();

        bool open(ofxOrbbec::Settings aSettings);
        bool isConnected();
        void close();
        void update();

//        static std::vector < std::shared_ptr<ob::DeviceInfo> > getDeviceList(bool bIncludeNetworkDevices);
		static std::vector< std::shared_ptr<ofxOrbbec::DeviceInfo> > getDeviceList(bool bIncludeNetworkDevices);

        //any frame
        bool isFrameNew();
        bool isFrameNewDepth();
        bool isFrameNewColor();
        bool isFrameNewIR();

        ofPixels getDepthPixels();
        ofFloatPixels getDepthPixelsF(); 
        ofPixels getColorPixels();
		ofPixels getIRPixels(); 
        
        std::vector <glm::vec3> getPointCloud(); 
        ofMesh getPointCloudMesh();
	
	/// \brief Get the minimum distance the camera is tracking in millimeters
	int getMinDistance();
	/// \brief Set the minimum distance the camera is tracking in millimeters
	void setMinDistance( int adistInMM );
	
	/// \brief Get the maximum distance the camera is tracking in millimeters
	int getMaxDistance();
	/// \brief Set the maximum distance the camera is tracking in millimeters
	void setMaxDistance( int adistInMM );
	
	/// \brief Get the maximum IR value for mapping IR pixels.
	int getMaxIRValue();
	/// \brief Set the maximum IR value for mapping IR pixel values.
	void setMaxIRValue( int aMaxIR );

	/// \brief: Set properties like OB_PROP_COLOR_GAIN_INT / OB_PROP_COLOR_AUTO_EXPOSURE_BOOL  etc 
	void setPropertyInt(int propEnum, int value);

    protected:
        void threadedFunction() override;
        void pointCloudThreadFunc();
        void clear();

        ofPixels processFrame(std::shared_ptr<ob::Frame> frame);
	ofPixels processIRFrame(std::shared_ptr<ob::Frame> frame);
		void pointCloudToMesh(void* depthData, int depthWidth, int depthHeight, ofPixels* colorPixels = nullptr);

        ofxOrbbec::Settings mCurrentSettings;
        
        bool bNewFrameColor, bNewFrameDepth, bNewFrameIR = false; 
        
        std::atomic<unsigned int> mInternalDepthFrameNo{0};
        std::atomic<unsigned int> mInternalColorFrameNo{0};
	std::atomic<unsigned int> mInternalIRFrameNo{0};
	
        std::atomic<unsigned int> mExtDepthFrameNo{0};
        std::atomic<unsigned int> mExtColorFrameNo{0};
	std::atomic<unsigned int> mExtIRFrameNo{0};

        // Front buffers — read by main thread via getters
        ofPixels mDepthPixels, mColorPixels;
        ofFloatPixels mDepthPixelsF;
	ofPixels mIRPixels;
        ofMesh mPointCloudMeshLocal;
		std::vector<glm::vec3> mPointCloudPtsLocal;

        // Back buffers — written by capture thread, swapped to front under mFrameMutex
        ofPixels mDepthPixelsBack, mColorPixelsBack;
	ofPixels mIRPixelsBack;
        ofFloatPixels mDepthPixelsFBack;
        ofMesh mPointCloudMesh;         // working buffer for point cloud build
		std::vector<glm::vec3> mPointCloudPts;

        std::mutex mFrameMutex;

        // Dedicated point cloud thread — decouples the slow point cloud build
        // from the fast capture loop so color/depth frames arrive at full rate.
        std::thread mPointCloudThread;
        std::mutex mPointCloudInputMutex;
        std::condition_variable mPointCloudCV;
        std::atomic<bool> mPointCloudThreadRunning{false};
        bool mPointCloudNewData = false;
        std::shared_ptr<ob::DepthFrame> mPCDepthFrame;
        //std::shared_ptr<ob::ColorFrame> mPCColorFrame;

		std::shared_ptr <ob::Pipeline> mPipe;
   		//std::shared_ptr <ob::PointCloudFilter> pointCloud;
   		std::shared_ptr <ob::Context> ctxLocal;

        #ifdef OFXORBBEC_DECODE_H264_H265
	
		#if defined(OFXORBBEC_MACOS_DECODE_VIDEOTOOLBOX)
			ofxOrbbecH264Decoder mVTDecoder;
		#else

            bool bInitOneTime = false;

			const AVCodec * codec264 = nullptr;
			AVCodecContext * codecContext264 = nullptr;

			const AVCodec * codec265 = nullptr;
			AVCodecContext* codecContext265 = nullptr;

            SwsContext* swsContext = nullptr;

            void initH26XCodecs();
            ofPixels decodeH26XFrame(uint8_t * myData, int dataSize, bool bH264);
		#endif

        #endif
        
        OBXYTables xyTables;
        std::vector <float> xyTableData;
        std::vector <uint8_t> mPointcloudData;
        bool bConnected = false;
        float mTimeSinceFrame = 0;
	
	std::atomic<int> mMinDistanceMM = 0;
	std::atomic<int> mMaxDistanceMM = 5460;
	std::atomic<int> mIRMaxValue = 1000;
	
//	std::shared_ptr<ob::Align> mObAlignToColor;

};
