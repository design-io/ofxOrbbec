#include "ofMain.h"

#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"
#include <opencv2/opencv.hpp>


//If you have ffmpeg / libavcodec included in your project uncomment below 
//You can easily get the required libs from ofxFFmpegRTSP addon ( if you add it to your project )
#define OFXORBBEC_DECODE_H264_H265

// this allows us to decode the color video streams from Femto Mega over IP connection 
#ifdef OFXORBBEC_DECODE_H264_H265
    extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
    #include <libavutil/imgutils.h>
    }
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
};

};


class ofxOrbbecCamera : public ofThread{
    public:

        ofxOrbbecCamera() = default; 
        ofxOrbbecCamera( const ofxOrbbecCamera & A) = default; 
        ~ofxOrbbecCamera();

        bool open(ofxOrbbec::Settings aSettings);
        bool isConnected();
        void close();
        void update();

        static std::vector < std::shared_ptr<ob::DeviceInfo> > getDeviceList(); 

        //any frame
        bool isFrameNew();
        bool isFrameNewDepth();
        bool isFrameNewColor();
        bool isFrameNewIR();

        ofPixels getDepthPixels();
        ofFloatPixels getDepthPixelsF(); 
        ofPixels getColorPixels(); 
        
        std::vector <glm::vec3> getPointCloud(); 
        ofMesh getPointCloudMesh();

    protected:
        void threadedFunction() override; 
        void clear(); 
        
        ofPixels processFrame(shared_ptr<ob::Frame> frame);
		void pointCloudToMesh(shared_ptr<ob::DepthFrame> depthFrame, shared_ptr<ob::ColorFrame> colorFrame = shared_ptr<ob::ColorFrame>() );

        ofxOrbbec::Settings mCurrentSettings;
        
        bool bNewFrameColor, bNewFrameDepth, bNewFrameIR = false; 
        
        unsigned int mInternalDepthFrameNo = 0;
		unsigned int mInternalColorFrameNo = 0;
		unsigned int mExtDepthFrameNo = 0;
		unsigned int mExtColorFrameNo = 0;

        ofPixels mDepthPixels, mColorPixels; 
        ofFloatPixels mDepthPixelsF;

        ofMesh mPointCloudMesh; 
        ofMesh mPointCloudMeshLocal;
        vector <glm::vec3> mPointCloudPts;
        vector <glm::vec3> mPointCloudPtsLocal;

		std::shared_ptr <ob::Pipeline> mPipe;
   		std::shared_ptr <ob::PointCloudFilter> pointCloud;
   		std::shared_ptr <ob::Context> ctxLocal;

        #ifdef OFXORBBEC_DECODE_H264_H265 

            bool bInitOneTime = false; 

            AVCodec* codec264 = nullptr;
            AVCodecContext* codecContext264 = nullptr;

            AVCodec* codec265 = nullptr;
            AVCodecContext* codecContext265 = nullptr;

            SwsContext* swsContext = nullptr;

            void initH26XCodecs();
            ofPixels decodeH26XFrame(uint8_t * myData, int dataSize, bool bH264);

        #endif
        
        OBXYTables xyTables;
        vector <float> xyTableData;
        vector <uint8_t> mPointcloudData;

};
