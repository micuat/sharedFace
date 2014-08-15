#include "ofApp.h"

#define NUM_SETS 4

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	video.loadMovie(ofToDataPath("a/video.mov"));
	ofSetWindowShape(video.getWidth(), video.getHeight());
	
	MAX_FRAME = video.getTotalNumFrames();
	
	points.resize(NUM_SETS);
	pointsSpeed.resize(NUM_SETS);
	for( int i = 0; i < points.size(); i++ ) {
		points.at(i).resize(MAX_FRAME);
		pointsSpeed.at(i).resize(MAX_FRAME);
	}
	flags = vector<int>(MAX_FRAME, 0);
	pointsSmoothed = points;
	frame = 0;
	
	bPlayback = false;
	
	// load data
	ofFile file;
	if( file.open(ofToDataPath("a/points.data"), ofFile::ReadOnly, false) ) {
		ofBuffer buf = file.readToBuffer();
		file.close();
		for( int i = 0; i < points.at(0).size() && !buf.isLastLine(); i++ ) {
			for( int j = 0; j < points.size() && !buf.isLastLine(); j++ ) {
				points.at(j).at(i).x = ofToFloat(buf.getNextLine());
				points.at(j).at(i).y = ofToFloat(buf.getNextLine());
				
				pointsSmoothed.at(j).at(i) = points.at(j).at(i);
				if( i - 2 >= 0 && i < points.at(0).size() ) {
					pointsSmoothed.at(j).at(i-1) = 0.25f * points.at(j).at(i-2) + 0.5f * points.at(j).at(i-1) + 0.25f * points.at(j).at(i);
				}
				if( i > 0 ) {
					pointsSpeed.at(j).at(i) = pointsSmoothed.at(j).at(i).x - pointsSmoothed.at(j).at(i-1).x;
				} else {
					pointsSpeed.at(j).at(i) = 0;
				}
			}
		}
	}
	
	ofSetFrameRate(10);
}

//--------------------------------------------------------------
void ofApp::update(){
	if( bPlayback ) {
		frame = (frame + 1) % MAX_FRAME;
	}
	video.setFrame(frame);
	video.update();
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(50);
	ofSetColor(255);
	video.draw(0, 0);
	
	ofSetLineWidth(1);

	for( int i = 0; i < points.size(); i++ ) {
		ofNoFill();
		ofColor c;
		if( i < NUM_SETS / 2 )
			c = ofColor::red;
		else
			c = ofColor::green;
		ofSetColor(c);
		ofCircle(points.at(i).at(frame), 10);
		
		c.a = 128;
		ofSetColor(c);
		if( frame > 0 ) {
			ofCircle(points.at(i).at(frame - 1), 10);
			ofLine(points.at(i).at(frame - 1), points.at(i).at(frame));
		}
		if( frame < MAX_FRAME - 1 ) {
			ofCircle(points.at(i).at(frame + 1), 10);
			ofLine(points.at(i).at(frame), points.at(i).at(frame + 1));
		}
		
		ofFill();
		ofCircle(pointsSmoothed.at(i).at(frame), 7);
	}
	ofSetColor(ofColor::red);
	ofSetLineWidth(1);
	ofLine(points.at(0).at(frame), points.at(1).at(frame));
	ofPoint midPoint0 = points.at(0).at(frame).getInterpolated(points.at(1).at(frame), 0.5);
	ofSetLineWidth(2);
	ofLine(midPoint0, midPoint0 + ofPoint((pointsSpeed.at(0).at(frame) + pointsSpeed.at(1).at(frame)) * 0.5, 0));
	ofSetColor(ofColor::green);
	ofSetLineWidth(1);
	ofLine(points.at(2).at(frame), points.at(3).at(frame));
	ofPoint midPoint1 = points.at(2).at(frame).getInterpolated(points.at(3).at(frame), 0.5);
	ofSetLineWidth(2);
	ofLine(midPoint1, midPoint1 + ofPoint((pointsSpeed.at(2).at(frame) + pointsSpeed.at(3).at(frame)) * 0.5, 0));
	
	ofSetColor(ofColor::whiteSmoke);
	ofSetLineWidth(3);
	ofLine(midPoint0.x, (midPoint0.y + midPoint1.y) * 0.5, midPoint1.x, (midPoint0.y + midPoint1.y) * 0.5);
	
	ofSetColor(ofColor::wheat);
	ofDrawBitmapString(ofToString(frame), 50, 50);
	
	ofSetLineWidth(2);
	ofTranslate(50, 450);
	float avgSpeed = 0.f;
	int count = 0;
	for( int i = 0; i < points.at(0).size(); i++ ) {
		float speed = (pointsSpeed.at(0).at(i) + pointsSpeed.at(1).at(i)) * 0.5f;
		if( frame == i ) {
			ofSetColor(255, 0, 0, 200);
		} else {
			ofSetColor(255, 128);
		}
		if( flags.at(i) > 0 ) {
			ofSetColor(255, 0, 0, 200);
			avgSpeed += abs(speed);
			count++;
		}
		ofLine(i * 3, 0, i * 3, speed);
	}
	avgSpeed /= count;
	ofTranslate(0, 50);
	for( int i = 0; i < points.at(0).size(); i++ ) {
		if( frame == i ) {
			ofSetColor(0, 255, 0, 200);
		} else {
			ofSetColor(255, 128);
		}
		ofLine(i * 3, 0, i * 3, (pointsSpeed.at(2).at(i) + pointsSpeed.at(3).at(i)) * 0.5f);
	}
	ofTranslate(0, 100);
	float avgDistance = 0.f;
	count = 0;
	for( int i = 0; i < points.at(0).size(); i++ ) {
		float distance = ofLerp(points.at(0).at(i).x, points.at(1).at(i).x, 0.5) - ofLerp(points.at(2).at(i).x, points.at(3).at(i).x, 0.5);
		if( flags.at(i) > 0 ) {
			ofSetColor(255, 0, 0, 200);
			avgDistance += abs(distance);
			count++;
		} else {
			ofSetColor(255, 128);
		}
		if( frame == i ) {
			ofSetColor(255, 250);
		}
		ofLine(i * 3, 0, i * 3, distance);
	}
	avgDistance /= count;
	ofDrawBitmapString("speed: " + ofToString(avgSpeed) + " distance: " + ofToString(avgDistance), 0, 50);
}

//--------------------------------------------------------------
void ofApp::exit(){
	ofFile file;
	file.open(ofToDataPath("a/points.data"), ofFile::WriteOnly, false);
	file.create();
	ofBuffer buf;
	for( int i = 0; i < points.at(0).size(); i++ ) {
		for( int j = 0; j < points.size(); j++ ) {
			buf.append(ofToString(points.at(j).at(i).x) + "\n");
			buf.append(ofToString(points.at(j).at(i).y) + "\n");
		}
	}
	file.writeFromBuffer(buf);
	file.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if( key == OF_KEY_RIGHT || key == ' ' ) {
		if( frame < MAX_FRAME - 1 ) {
			frame++;
		}
	}
	if( key == OF_KEY_LEFT ) {
		if( frame > 0 ) {
			frame--;
		}
	}
	
	if( key == 'p' ) {
		bPlayback = !bPlayback;
	}
	key -= '1';
	if( key >= 0 && key < NUM_SETS ) {
		points.at(key).at(frame) = ofPoint(mouseX, mouseY);
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
	flagPointer = (x - 50) / 3;
	if( flagPointer < 0 ) flagPointer = 0;
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
	int flagPointerEnd = (x - 50) / 3;
	
	if( flagPointer <= flagPointerEnd ) {
		for( int i = flagPointer; i < flagPointerEnd && i < flags.size(); i++ ) {
			flags.at(i) = 1;
		}
	}
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
