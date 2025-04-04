#pragma once

#include "ofMain.h"
#include "ofxOrbbecCamera.h"

class ofApp : public ofBaseApp{

	public:
		void setup() override;
		void update() override;
		void draw() override;
		void exit() override;

		ofPixels processFrame(std::shared_ptr<ob::Frame> frame);

		void keyPressed(int key) override;
		void keyReleased(int key) override;
		void mouseMoved(int x, int y) override;
		void mouseDragged(int x, int y, int button) override;
		void mousePressed(int x, int y, int button) override;
		void mouseReleased(int x, int y, int button) override;
		void mouseEntered(int x, int y) override;
		void mouseExited(int x, int y) override;
		void windowResized(int w, int h) override;
		void dragEvent(ofDragInfo dragInfo) override;
		void gotMessage(ofMessage msg) override;
		
		ofMesh mPointCloudMesh;

		ofxOrbbecCamera orbbecCam; 
		ofxOrbbec::Settings settings; 

		ofTexture outputTex;
		ofTexture outputTexDepth;

		ofEasyCam mCam;
};
