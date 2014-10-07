#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(false);
    
	//kinect.init();
	kinect.init(true); // shows infrared instead of RGB video image
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
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
	contourFinder.setMinAreaRadius(1);
	contourFinder.setMaxAreaRadius(15);
	contourFinder.setFindHoles(true);
	contourFinder.setTargetColor(0);
	contourFinder.setThreshold(200);
	// wait for half a frame before forgetting something
	contourFinder.getTracker().setPersistence(15);
	// an object can move up to 32 pixels per frame
	contourFinder.getTracker().setMaximumDistance(32);
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	//kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
}

//--------------------------------------------------------------
void ofApp::update() {
	
	ofBackground(100, 100, 100);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
//		float threshold = ofMap(mouseX, 0, ofGetWidth(), 0, 255);
//		contourFinder.setThreshold(threshold);
		bool bFindHoles = false;
		if( bFindHoles ) {
			ofImage eroded;
			ofxCv::erode(kinect.getDepthPixelsRef(), eroded, 3);
			contourFinder.findContours(eroded);
		} else {
			ofImage blurred;
			ofxCv::blur(kinect.getPixelsRef(), blurred, 3);
			contourFinder.findContours(blurred);
		}
	}
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void ofApp::draw() {
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	} else {
		// draw from the live kinect
		ofSetColor(255, 128);
		kinect.drawDepth(0, 0);
		kinect.draw(0, 0);
		ofPushStyle();
		ofSetLineWidth(2);
		contourFinder.draw();
		
		ofxCv::RectTracker& tracker = contourFinder.getTracker();
		for(int i = 0; i < contourFinder.size(); i++) {
			cv::RotatedRect ellipse = contourFinder.getFitEllipse(i);
			ofVec2f ellipseCenter = ofxCv::toOf(ellipse.center);
			ofVec2f ellipseSize = ofxCv::toOf(ellipse.size);
			
			ofPushMatrix();
			ofSetColor(ofColor::violet, 200);
			ofTranslate(ellipseCenter.x, ellipseCenter.y);
			ofRotate(ellipse.angle);
			ofEllipse(0, 0, ellipseSize.x, ellipseSize.y);
			ofPopMatrix();
			
			ofSetColor(ofColor::brown, 200);
			ofPoint center = ofxCv::toOf(contourFinder.getCenter(i));
			ofPushMatrix();
			ofTranslate(center.x, center.y);
			int label = contourFinder.getLabel(i);
			string msg = ofToString(label) + ":" + ofToString(tracker.getAge(label));
			ofDrawBitmapString(msg, 0, 0);
			ofVec2f velocity = ofxCv::toOf(contourFinder.getVelocity(i));
			ofScale(5, 5);
			ofLine(0, 0, velocity.x, velocity.y);
			ofPopMatrix();
		}
		ofPopStyle();

#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	}
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
        
    if(kinect.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
		<< "motor / led / accel controls are not currently supported" << endl << endl;
    }
    
	reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.size()
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;
    if(kinect.hasCamTiltControl()) {
    	reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }
    
	ofDrawBitmapString(reportStream.str(), 20, 652);
	ofDrawBitmapString("FPS: " + ofToString(ofGetFrameRate()), 20, 50);

}

void ofApp::drawPointCloud() {
	
	int w = 640;
	int h = 480;
	ofMesh mesh, markers, markersPlane;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	markers.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
	int step = 2;
	
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getDepthPixelsRef().getColor(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	ofxCv::RectTracker& tracker = contourFinder.getTracker();
	for(int i = 0; i < contourFinder.size(); i++) {
		ofRectangle rect = ofxCv::toOf(contourFinder.getBoundingRect(i));
		float centerDistance = kinect.getDistanceAt(rect.getTopLeft()) +
		kinect.getDistanceAt(rect.getTopRight()) +
		kinect.getDistanceAt(rect.getBottomLeft()) +
		kinect.getDistanceAt(rect.getBottomRight());
		int count = 4;
		if( kinect.getDistanceAt(rect.getTopLeft()) <= 0.f ) count--;
		if( kinect.getDistanceAt(rect.getTopRight()) <= 0.f ) count--;
		if( kinect.getDistanceAt(rect.getBottomLeft()) <= 0.f ) count--;
		if( kinect.getDistanceAt(rect.getBottomRight()) <= 0.f ) count--;
		if( count == 0 ) continue;
		centerDistance /= count;
		float threshold = 1.f;
		
		ofVec3f marker(kinect.getWorldCoordinateAt(rect.getCenter().x, rect.getCenter().y, centerDistance));
		markers.addVertex(marker);
		markers.addColor(ofColor::violet);
		markersPlane.addVertex(ofxCv::toOf(contourFinder.getCenter(i)));
	}
	
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
	mesh.drawVertices();
	//markers.drawVertices();
	for( int i = 0; i < markers.getNumVertices(); i++ ) {
		ofSetColor(markers.getColor(i));
		ofDrawSphere(markers.getVertex(i), 10);
	}
	//markers.draw();
	
	ofDisableDepthTest();
	ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::exit() {
	//kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
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
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{}
