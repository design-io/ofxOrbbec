#include "ofMain.h"

#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"
#include <opencv2/opencv.hpp>

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


class ofxOrbbecCamera{
    public:

        bool open(ofxOrbbec::Settings aSettings);
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
        void clear(); 
        
        static std::shared_ptr<ob::Context> & getContext(); 
		static std::shared_ptr <ob::Context> ctx;

        ofPixels processFrame(shared_ptr<ob::Frame> frame);
        void pointCloudToMesh(shared_ptr<ob::Frame> frame, bool bRGB = false);
        
        ofxOrbbec::Settings mCurrentSettings;
        bool bNewFrameColor, bNewFrameDepth, bNewFrameIR = false; 

        ofPixels mDepthPixels, mColorPixels; 
        ofFloatPixels mDepthPixelsF;

        ofMesh mPointCloudMesh; 
        vector <glm::vec3> mPointCloudPts;

		std::shared_ptr <ob::Pipeline> mPipe;
   		std::shared_ptr <ob::PointCloudFilter> pointCloud;
};
