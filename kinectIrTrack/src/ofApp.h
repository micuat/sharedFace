/*
 https://github.com/micuat/sharedFace
 
 Naoto Hieda <micuat@gmail.com> 2014
 */

#pragma once

#include "ofMain.h"

#include "ofxCv.h"
#include "ofxKinect.h"
#include "ofxDelaunay.h"
#include "ofxOsc.h"

#define NUM_MARKERS 4
#define PORT 57130
#define RES_MULT 2

class ofApp : public ofBaseApp {
public:
	void setup();
	void init();
	void update();
	void draw();
	void updatePointCloud(ofMesh&);
	bool findVec3fFromRect(ofRectangle&, ofVec3f&);
	void findVec3fFromFitting(vector<cv::Point>&, ofMesh&);
	vector<int> registerMarkers(ofMesh&, ofMesh&, vector<int>&, ofMesh&);
	void updateTargetUsingLabels(ofMesh&, vector<int>&, vector<int>&, ofMesh&);
	void updateMarkerKalmanFilter();
	ofMatrix4x4 findRigidTransformation(ofMesh&, ofMesh&);
	void updateModelKalmanFilter();
	void updateInitMesh();
	void updateReceiveOsc();
	void keyPressed(int);
	void mousePressed(int x, int y, int button);
	void mouseDragged(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void dragEvent(ofDragInfo);
	
	vector<string> rootDir;
	
private:
	int projector_width, projector_height;
	float projector_horizontal_center;
	ofMesh mesh;
	ofEasyCam cam;
	cv::Mat proIntrinsic, proExtrinsic;
	ofxCv::Intrinsics proCalibration;
	cv::Size proSize;
	
	enum CameraMode {EASYCAM_MODE, PRO_MODE, CAM_MODE};
	CameraMode cameraMode;
	
	ofxKinect kinect;
	ofxCv::ContourFinder contourFinder;
	ofMesh target, initTarget, markers, samples;
	ofMesh initMesh;
	ofImage image;
	ofMatrix4x4 modelMat;
	vector<int> registeredLabels;
	
	bool bReset;
	ofxOscReceiver receiver;
	vector<ofVec4f> lines;
	ofFbo drawImage;
	
	ofxCv::KalmanPosition kalmanPosition;
	ofxCv::KalmanEuler kalmanEuler;
	vector<ofxCv::KalmanPosition> kalmanMarkers;
	
	vector<ofImage> stamps;
	vector<ofImage> stampsBlink;
	vector<ofVec2f> stampCoord;
	
	bool drawPointCloud;
	int shaderMode;
	
	int jsMode;
	
	ofShader shader, shader2;
	
	bool pathLoaded;
};
