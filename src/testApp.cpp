#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 70;
	bDrawOutlines = true;
    
	ofSetFrameRate(60);
	
    
    
//    set up gui

    gui.setup("Interface"); // most of the time you don't need a name but don't forget to call setup
    gui.add(bDrawOutlines.set("Outline Mode", true));
    gui.add(bKinectFlip.set("Flip Kinect", false));
    gui.add(bSetRegistration.set("Calibrate With RGB", false));
    gui.add(setRegistration.setup("Set RGB Cal"));
    gui.add(nearThreshold.set("Near Threshold", 230, 0, 255));
    gui.add(farThreshold.set("Far Threshold", 70, 0, 255));
    gui.add(blurAmnt.set("Blur Amount", 5, 0, 30));
    gui.add(angle.set("Tilt Angle", 0, -30, 30));
    gui.add(setAngle.setup("Set Angle"));
    
    offsetY = 135;
    
    
	// zero the tilt on startup
	kinect.setCameraTiltAngle(angle);
	
    
//    set server name
    syphonServer.setName("Kinect");
}

//--------------------------------------------------------------
void testApp::update() {
	
	ofBackground(100, 100, 100);
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		if(bKinectFlip){
            grayImage.rotate(180, 320, 240);
        }
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
        grayThreshNear = grayImage;
        grayThreshFar = grayImage;
        grayThreshNear.threshold(nearThreshold, true);
        grayThreshFar.threshold(farThreshold);
        cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		
        grayImage.blur(blurAmnt);
        
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
        
	}
	
}

//--------------------------------------------------------------
void testApp::draw() {

    ofClear(0,0,0);

//    first draw kinect image to syphon
    if(!bDrawOutlines){
        grayImage.draw(0, 0, ofGetWidth(), ofGetHeight());
    }
    if(bDrawOutlines){
        contourFinder.draw(0, 0, ofGetWidth(), ofGetHeight());
    }
    
    syphonServer.publishScreen();
    
    
//    then render GUI
    ofClear(50, 50, 50);
	ofSetColor(255, 255, 255);
	

    // monitors
    kinect.drawDepth(10, 10 + offsetY, 400, 300);
    kinect.draw(420, 10 + offsetY, 400, 300);
    
    grayImage.draw(10, 320 + offsetY, 400, 300);
    contourFinder.draw(420, 320 + offsetY, 400, 300);
	
	// draw instructions
	ofSetColor(255, 255, 255);

    gui.draw();
    
    if (setAngle) {
        kinect.setCameraTiltAngle(angle);
    }
    if (setRegistration) {
        kinect.setRegistration(bSetRegistration);
    }
}


//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
