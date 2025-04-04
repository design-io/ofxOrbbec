#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

    ofSetLogLevel(OF_LOG_NOTICE);
    auto deviceInfo = ofxOrbbecCamera::getDeviceList(true); 

    settings.bColor = true; 
    settings.bDepth = true; 
    settings.bPointCloud = true; 
    //settings.depthFrameSize.format = OB_FORMAT_Y16;
    //settings.depthFrameSize.requestWidth = 640; //For Femto: 512 is WFOV binned, 640 is NFOV, 320 is NFOV binned 
    settings.colorFrameSize.format = OB_FORMAT_MJPG; 
    settings.colorFrameSize.requestWidth = 1280;
    //settings.bPointCloudRGB = true; 
    //settings.ip = "192.168.50.70";
    
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
        outputTex.draw(0,0, outputTex.getWidth()/4, outputTex.getHeight()/4);
        outputTexDepth.draw(0, outputTex.getHeight()/4, outputTexDepth.getWidth()/2, outputTexDepth.getHeight()/2);
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
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if( key == 'c' ){
        orbbecCam.close();
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
