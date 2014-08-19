/*
 https://github.com/micuat/sharedFace
 
 Naoto Hieda <micuat@gmail.com> 2014
 */

#pragma once

#include "ofMain.h"

#include "ofxCv.h"
#include "ofxKinect.h"
#include "ofxDelaunay.h"

#define NUM_MARKERS 3

class ofApp : public ofBaseApp {
public:
	void setup();
	void init();
	void update();
	void draw();
	void updatePointCloud(ofMesh&);
	bool findVec3fFromRect(ofRectangle&, ofVec3f&);
	vector<int> registerMarkers(ofMesh&, ofMesh&, vector<int>&, ofMesh&);
	void updateTargetUsingLabels(ofMesh&, vector<int>&, vector<int>&, ofMesh&);
	void updateMarkerKalmanFilter();
	ofMatrix4x4 findRigidTransformation(ofMesh&, ofMesh&);
	void updateModelKalmanFilter();
	void updateInitMesh();
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
	ofMesh target, initTarget;
	ofMesh initMesh;
	ofImage image;
	ofMatrix4x4 modelMat;
	vector<int> registeredLabels;
	
	bool bReset;
	vector<ofVec4f> lines;
	ofFbo drawImage;
	
	ofxCv::KalmanPosition kalmanPosition;
	ofxCv::KalmanEuler kalmanEuler;
	vector<ofxCv::KalmanPosition> kalmanMarkers;
	
	bool pathLoaded;
};
