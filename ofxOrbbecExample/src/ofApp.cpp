#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

    ofSetLogLevel(OF_LOG_NOTICE);
	
//	auto deviceInfo = ofxOrbbecCamera::getDeviceList(true);
	// more verbose output
//	settings.logLevel = OB_LOG_SEVERITY_DEBUG;

	settings.bColor = true; 
    settings.bDepth = true; 
    settings.bPointCloud = true;
//	settings.bIR = true;
    //settings.depthFrameSize.requestWidth = 640; //For Femto: 512 is WFOV binned, 640 is NFOV, 320 is NFOV binned
//	settings.depthFrameSize.requestWidth = 512;
	
    settings.colorFrameSize.requestWidth = 1280;
//	settings.colorFrameSize.requestHeight = 720;
//    settings.bPointCloudRGB = true;
//	settings.bAlignDepthToColor = true;
    settings.ip = "192.168.50.70";
	// if connecting via IP and on macOS
#if defined(TARGET_OSX)
	settings.colorFrameSize.format = OB_FORMAT_H264;
#else
	settings.colorFrameSize.format = OB_FORMAT_MJPG;
#endif
	
    orbbecCam.open(settings);
    
}

//--------------------------------------------------------------
void ofApp::update(){

    orbbecCam.update();
    
    if( orbbecCam.isFrameNewColor() ){
        auto pix = orbbecCam.getColorPixels();
        outputTex.loadData(pix);
    }

    if( orbbecCam.isFrameNewDepth() ){
        auto depthPix = orbbecCam.getDepthPixels();
        outputTexDepth.loadData(depthPix);

        mPointCloudMesh = orbbecCam.getPointCloudMesh();
    }
	
	if( orbbecCam.isFrameNewIR() ) {
		auto irPix = orbbecCam.getIRPixels();
		outputTexIR.loadData(irPix);
	}

}

//--------------------------------------------------------------
void ofApp::exit(){
    orbbecCam.close();
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(10); 

    if(!ofGetMousePressed()){
        ofSetColor(255, 255);
		float ty = 0;
		if( outputTex.isAllocated() ) {
			outputTex.draw(0,0, outputTex.getWidth()/4, outputTex.getHeight()/4);
			ty += outputTex.getHeight()/4;
		}
		if( outputTexDepth.isAllocated() ) {
			outputTexDepth.draw(0, ty, outputTexDepth.getWidth()/2, outputTexDepth.getHeight()/2);
			ty += outputTexDepth.getHeight()/2;
		}
		if(outputTexIR.isAllocated() ) {
			outputTexIR.draw(0, ty, outputTexIR.getWidth()/2, outputTexIR.getHeight()/2);
			ty += outputTexIR.getHeight()/2;
		}
    }

    ofSetColor(255);
    ofEnableDepthTest();
    mCam.begin();
        ofPushMatrix();
        ofTranslate(0, -300, 1000);
        mPointCloudMesh.draw(); 
        ofPopMatrix();
    mCam.end();
    ofDisableDepthTest();
	
	std::stringstream ss;
	if( settings.bDepth ) {
		ss << "Depth: max distance(up/down): " << orbbecCam.getMaxDistance() << endl;
	}
	if( settings.bIR ) {
		ss << "IR: max value(left/right): " << orbbecCam.getMaxIRValue() << endl;
	}
	if( ss.str().size() > 0 ) {
		ofDrawBitmapStringHighlight(ss.str(), 512, 60 );
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if( key == 'c' ){
        orbbecCam.close();
	} else if( key == OF_KEY_UP ) {
		orbbecCam.setMaxDistance( orbbecCam.getMaxDistance() + 10 );
	} else if( key == OF_KEY_DOWN ) {
		orbbecCam.setMaxDistance( std::max(orbbecCam.getMaxDistance()- 10, 100) );
	} else if( key == OF_KEY_LEFT ) {
		orbbecCam.setMaxIRValue( std::max(orbbecCam.getMaxIRValue() - 50, 50) );
	} else if( key == OF_KEY_RIGHT ) {
		orbbecCam.setMaxIRValue( orbbecCam.getMaxIRValue() + 50 );
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
