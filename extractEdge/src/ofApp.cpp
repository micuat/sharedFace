#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

void ofApp::setup() {
	loaded.loadImage("/Users/naoto/Documents/Programs/ProcessingJsOsc/nodeClient/template/image.jpg");
	convertColor(loaded, gray, CV_RGB2GRAY);
	fbo.allocate(640, 480, GL_RGBA);
}

void ofApp::update() {
	Canny(gray, edge, mouseX, mouseY, 3);
	blur(edge, 3);
	invert(edge);
	edge.update();
	
	fbo.begin();
	ofBackground(0, 255);
	ofEnableBlendMode(OF_BLENDMODE_ADD);
	ofSetColor(255, 200);
	loaded.draw(0, 0);
	ofSetColor(255, 255-200);
	edge.draw(0, 0);
	ofDisableAlphaBlending();
	fbo.end();
}

void ofApp::draw() {
	ofBackground(0, 255);
	fbo.draw(0, 0);
}

void ofApp::keyPressed(int key) {
	ofSaveScreen("/Users/naoto/Documents/Programs/ProcessingJsOsc/nodeClient/web-export/image.jpg");
}

