#pragma once

#include "ofMain.h"
#include "ofxCv.h"

class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int);
	
	ofImage loaded;
	ofPixels gray;
	ofImage edge;
	ofFbo fbo;
};
