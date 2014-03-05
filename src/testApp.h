#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxSyphon.h"

class testApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
		
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	
	ofxKinect kinect;
	
//	opencv stuff for kinect tracking
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
//    ofxCvGrayscaleImage blurredLines; //greyImage blurred
	
	ofxCvContourFinder contourFinder;
	
    
    
//    gui
    ofParameter<bool> bDrawOutlines;
    ofParameter<bool> bSetRegistration;
    ofParameter<bool> bKinectFlip;
	
	ofParameter<int> nearThreshold;
	ofParameter<int> farThreshold;
	ofParameter<int> blurAmnt;
    
	ofParameter<int> angle;
    
    ofxButton setAngle;
    ofxButton setRegistration;
    
    ofxPanel gui;
    int offsetY;
    
    
    ofxSyphonServer syphonServer;
};
