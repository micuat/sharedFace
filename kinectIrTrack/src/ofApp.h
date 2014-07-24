/*
 https://github.com/micuat/sharedFace
 
 Naoto Hieda <micuat@gmail.com> 2014
 */

#pragma once

#include "ofMain.h"

#include "ofxCv.h"
#include "ofxActiveScan.h"
#include "ofxKinect.h"

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
	ofMatrix4x4 findRigidTransformation(ofMesh&, ofMesh&);
	void updateKalmanFilter();
	void updateInitMesh();
	void keyPressed(int);
	void mousePressed(int x, int y, int button);
	void mouseDragged(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void dragEvent(ofDragInfo);
	
	vector<string> rootDir;
	
private:
	ofxActiveScan::Options options;
	ofMesh mesh;
	ofEasyCam cam;
	cv::Mat proIntrinsic, proExtrinsic;
	ofxCv::Intrinsics proCalibration;
	cv::Size proSize;
	
	enum CameraMode {EASYCAM_MODE, PRO_MODE, CAM_MODE};
	CameraMode cameraMode;
	
	int cw, ch;
	
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
	
	cv::KalmanFilter KF;
	cv::Mat_<float> measurement;
	
	bool pathLoaded;
};
