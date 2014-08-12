/*
 https://github.com/micuat/sharedFace
 
 Naoto Hieda <micuat@gmail.com> 2014
 */

#include "ofApp.h"

double getLongestDistance(ofMesh m) {
	int n = m.getNumVertices();
	double longestDistance = 0;
	for( int i = 0; i < n - 1; i++ ) {
		for( int j = i + 1; j < n; j++ ) {
			ofVec3f vi = m.getVertex(i);
			ofVec3f vj = m.getVertex(j);
			double distance = vi.distance(vj);
			
			if( distance > longestDistance ) {
				longestDistance = distance;
			}
		}
	}
	
	return longestDistance;
}

// entry point
void ofApp::setup() {
	if( rootDir.size() > 0 ) {
		init();
		pathLoaded = true;
	} else {
		pathLoaded = false;
	}
}

void ofApp::init() {
	ofSetLogLevel(OF_LOG_WARNING);
	
	// enable depth->video image calibration
	kinect.setRegistration(false);
    
	kinect.init();
	while( !kinect.isFrameNewVideo() ) {
		kinect.update();
		image.setFromPixels(kinect.getPixelsRef());
		ofSleepMillis(50);
	}
	kinect.close();
	
	kinect.init(true);
	
	kinect.open();		// opens first available kinect
	
	cameraMode = EASYCAM_MODE;
	
	cv::FileStorage fs(ofToDataPath(rootDir[0] + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> proSize.width;
	fs["proHeight"] >> proSize.height;
	
	cv::FileStorage cfs(ofToDataPath(rootDir[0] + "/calibration.yml"), cv::FileStorage::READ);
	cfs["proIntrinsic"] >> proIntrinsic;
	cfs["proExtrinsic"] >> proExtrinsic;
	
	cout << proIntrinsic << endl;
	cout << proExtrinsic << endl;
	
	// set parameters for projection
	proCalibration.setup(proIntrinsic, proSize);
	
	contourFinder.setMinAreaRadius(3);
	contourFinder.setMaxAreaRadius(15);
	contourFinder.setFindHoles(true);
	contourFinder.setTargetColor(0);
	contourFinder.setThreshold(128);
	// wait for half a frame before forgetting something
	contourFinder.getTracker().setPersistence(60);
	// an object can move up to 32 pixels per frame
	contourFinder.getTracker().setMaximumDistance(32);
	
	drawImage.allocate(kinect.width, kinect.height);
	
	// Kalman filter
	kalmanPosition.init(0.1, 0.1);
	kalmanEuler.init(0.1, 0.1);
	for( int i = 0; i < NUM_MARKERS; i++ ) {
		ofxCv::KalmanPosition kPos;
		kPos.init(0.01, 0.1);
		kalmanMarkers.push_back(kPos);
	}
	
	ofEnableDepthTest();
}

void ofApp::update() {
	kinect.update();
	if(kinect.isFrameNew()) {
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
		
		if( cameraMode == EASYCAM_MODE ) {
			updatePointCloud(mesh);
		}
		
		ofMesh markers, markersProjected;
		vector<int> markerLabels;
		ofxCv::RectTracker& tracker = contourFinder.getTracker();
		for(int i = 0; i < contourFinder.size(); i++) {
			ofRectangle rect = ofxCv::toOf(contourFinder.getBoundingRect(i));
			ofVec3f marker;
			if( findVec3fFromRect(rect, marker) ) { // skip if not enough neighbors
				markers.addVertex(marker);
				markersProjected.addVertex(kinect.sensorToColorCoordinate(rect.getCenter(), marker.z));
				markerLabels.push_back(contourFinder.getLabel(i));
			}
		}
		
		if( markers.getNumVertices() > 0 ) {
			bool bNeedReregister = false;
			const vector<unsigned int>& deadLabels = tracker.getDeadLabels();
			for( int i = 0; i < registeredLabels.size(); i++ ) {
				for( int j = 0; j < deadLabels.size(); j++ ) {
					if( registeredLabels.at(i) == deadLabels.at(j) ) {
						bNeedReregister = true;
					}
				}
			}
			for( int i = 0; i < registeredLabels.size(); i++ ) {
				for( int j = i + 1; j < registeredLabels.size(); j++ ) {
					if( registeredLabels.at(i) == registeredLabels.at(j) ) {
						bNeedReregister = true;
					}
				}
			}
			if( registeredLabels.empty() || bNeedReregister || bReset ) { // need to register
				registeredLabels = registerMarkers(markers, markersProjected, markerLabels, target);
			} else { // rely on tracker
				updateTargetUsingLabels(markers, markerLabels, registeredLabels, target);
			}
			
			if( initTarget.getNumVertices() == 0 || bReset ) {
				initTarget = target;
			}
			
			updateMarkerKalmanFilter();
			modelMat = findRigidTransformation(target, initTarget);
			updateModelKalmanFilter();
			
			if( initMesh.getNumVertices() == 0 || bReset ) {
				updateInitMesh();
				bReset = false;
			}
		}
	}
}

void ofApp::updatePointCloud(ofMesh& m) {
	int w = 640;
	int h = 480;
	m.clear();
	m.setMode(OF_PRIMITIVE_POINTS);
	glPointSize(2);
	int step = 2;
	
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			float distance = kinect.getDistanceAt(x, y);
			if( distance > 0 ) {
				ofPoint coord = kinect.sensorToColorCoordinate(ofPoint(x, y));
				m.addVertex(kinect.getWorldCoordinateAt(coord.x, coord.y, distance));
				m.addTexCoord(coord);
			}
		}
	}
}

bool ofApp::findVec3fFromRect(ofRectangle& rect, ofVec3f& v) {
	float centerDistance = kinect.getDistanceAt(rect.getTopLeft()) +
	kinect.getDistanceAt(rect.getTopRight()) +
	kinect.getDistanceAt(rect.getBottomLeft()) +
	kinect.getDistanceAt(rect.getBottomRight());
	int count = 4;
	if( kinect.getDistanceAt(rect.getTopLeft()) <= 0.f ) count--;
	if( kinect.getDistanceAt(rect.getTopRight()) <= 0.f ) count--;
	if( kinect.getDistanceAt(rect.getBottomLeft()) <= 0.f ) count--;
	if( kinect.getDistanceAt(rect.getBottomRight()) <= 0.f ) count--;
	
	if( count < 2 ) return false;
	
	centerDistance /= count;
	float threshold = 1.f;
	
	ofVec2f rectCenter = kinect.sensorToColorCoordinate(rect.getCenter(), centerDistance);
	v = ofVec3f(kinect.getWorldCoordinateAt(rectCenter.x, rectCenter.y, centerDistance));
	return true;
}

vector<int> ofApp::registerMarkers(ofMesh& markers, ofMesh& markersProjected, vector<int>& markerLabels, ofMesh& markersRegistered) {
	vector<int> labels;
	labels.resize(3, markerLabels.at(0));
	
	ofVec3f trMarker = markers.getVertex(0);
	ofVec3f blMarker = markers.getVertex(0);
	ofVec3f brMarker = markers.getVertex(0);
	float trDist = markersProjected.getVertex(0).distance(ofVec2f(kinect.width, 0));
	float blDist = markersProjected.getVertex(0).distance(ofVec2f(0, kinect.height));
	float brDist = markersProjected.getVertex(0).distance(ofVec2f(kinect.width, kinect.height));
	for( int i = 1; i < markers.getNumVertices(); i++ ) {
		ofVec2f p = markersProjected.getVertex(i);
		
		float curtrDist = p.distance(ofVec2f(kinect.width, 0));
		float curblDist = p.distance(ofVec2f(0, kinect.height));
		float curbrDist = p.distance(ofVec2f(kinect.width, kinect.height));
		if( curtrDist < trDist ) {
			trDist = curtrDist;
			trMarker = markers.getVertex(i);
			labels.at(0) = markerLabels.at(i);
		}
		if( curblDist < blDist ) {
			blDist = curblDist;
			blMarker = markers.getVertex(i);
			labels.at(1) = markerLabels.at(i);
		}
		if( curbrDist < brDist ) {
			brDist = curbrDist;
			brMarker = markers.getVertex(i);
			labels.at(2) = markerLabels.at(i);
		}
	}
	
	markersRegistered.clear();
	markersRegistered.addVertex(trMarker);
	markersRegistered.addVertex(blMarker);
	markersRegistered.addVertex(brMarker);
	return labels;
}

void ofApp::updateTargetUsingLabels(ofMesh& markers, vector<int>& curLabels, vector<int>& targetLabels, ofMesh& target) {
	target.clear();
	for( int i = 0; i < targetLabels.size(); i++ ) {
		for( int j = 0; j < curLabels.size(); j++ ) {
			if( targetLabels.at(i) == curLabels.at(j) ) {
				target.addVertex(markers.getVertex(j));
				break;
			}
		}
	}
}

void ofApp::updateMarkerKalmanFilter() {
	for( int i = 0; i < target.getNumVertices(); i++ ) {
		kalmanMarkers.at(i).update(target.getVertex(i));
		target.setVertex(i, kalmanMarkers.at(i).getEstimation());
	}
}

ofMatrix4x4 ofApp::findRigidTransformation(ofMesh& target, ofMesh& initTarget) {
	ofMatrix4x4 mat;
	
	// find rigid transformation
	// http://nghiaho.com/?page_id=671
	int n = target.getNumVertices();
	ofVec3f c0 = initTarget.getCentroid();
	ofVec3f cC = target.getCentroid();
	cv::Mat centroid0 = (cv::Mat1f(3, 1) << c0.x, c0.y, c0.z);
	cv::Mat centroidC = (cv::Mat1f(3, 1) << cC.x, cC.y, cC.z);
	
	
	cv::Mat H = cv::Mat_<float>::zeros(3, 3);
	for( int i = 0; i < n; i++ ) {
		cv::Mat p0 = (cv::Mat1f(3, 1) << initTarget.getVertex(i).x, initTarget.getVertex(i).y, initTarget.getVertex(i).z);
		cv::Mat pC = (cv::Mat1f(3, 1) << target.getVertex(i).x, target.getVertex(i).y, target.getVertex(i).z);
		p0 = p0 - centroid0;
		pC = pC - centroidC;
		H = H + p0 * pC.t();
	}
	
	// latency compensation
	float dt = 4.f;
	
	// predict centroid
	kalmanPosition.update(cC);
	cC = kalmanPosition.getEstimation();
	ofVec3f v = kalmanPosition.getVelocity();
	cC = cC + v * dt;
	
	// predict Euler angle
	cv::SVD svd(H);
	cv::Mat R = svd.vt.t() * svd.u.t();
	mat.set(R.at<float>(0, 0), R.at<float>(1, 0), R.at<float>(2, 0), 0,
			R.at<float>(0, 1), R.at<float>(1, 1), R.at<float>(2, 1), 0,
			R.at<float>(0, 2), R.at<float>(1, 2), R.at<float>(2, 2), 0,
			0, 0, 0, 1);
	kalmanEuler.update(mat.getRotate());
	ofVec3f pEuler = kalmanEuler.KalmanPosition_<float>::getEstimation();
	ofVec3f vEuler = kalmanEuler.KalmanPosition_<float>::getVelocity();
	ofQuaternion qPredicted;
	qPredicted.set(0, 0, 0, 1);
	qPredicted.makeRotate(pEuler.x + vEuler.x * dt, ofVec3f(1, 0, 0), pEuler.z + vEuler.z * dt, ofVec3f(0, 0, 1), pEuler.y + vEuler.y * dt, ofVec3f(0, 1, 0));
	
	// update model matrix
	mat.setRotate(qPredicted);
	ofVec3f T = cC - ofMatrix4x4::getTransposedOf(mat) * c0; // mat must be transposed for multiplication
	mat.setTranslation(T);
	return mat;
}

void ofApp::updateModelKalmanFilter() {
}

void ofApp::updateInitMesh() {
	double distanceProj = getLongestDistance(target);
	
	initMesh.clear();
	initMesh.setMode(OF_PRIMITIVE_POINTS);
	for( int i = 0; i < mesh.getNumVertices(); i++ ) {
		for( int j = 0; j < target.getNumVertices(); j++ ) {
			if( mesh.getVertex(i).distance(target.getVertex(j)) < distanceProj * 2.0 ) {
				initMesh.addVertex(mesh.getVertex(i));
				initMesh.addTexCoord(mesh.getTexCoord(i));
				break;
			}
		}
	}
}

void ofApp::draw() {
	if( pathLoaded ) {
		
		ofBackground(0);
		
		if(cameraMode == EASYCAM_MODE) {
			cam.begin();
			ofScale(1, -1, -1);
			ofTranslate(0, 0, -2);
		} else if(cameraMode == PRO_MODE) {
			ofSetupScreenPerspective(proSize.width, proSize.height);
			proCalibration.loadProjectionMatrix(0.0001, 100000000.0);
			cv::Mat m = proExtrinsic;
			cv::Mat extrinsics = (cv::Mat1d(4,4) <<
								  m.at<double>(0,0), m.at<double>(0,1), m.at<double>(0,2), m.at<double>(0,3),
								  m.at<double>(1,0), m.at<double>(1,1), m.at<double>(1,2), m.at<double>(1,3),
								  m.at<double>(2,0), m.at<double>(2,1), m.at<double>(2,2), m.at<double>(2,3),
								  0, 0, 0, 1);
			extrinsics = extrinsics.t();
			glMultMatrixd((GLdouble*) extrinsics.ptr(0, 0));
		} else if(cameraMode == CAM_MODE) {
			ofDisableDepthTest();
			ofEnableBlendMode(OF_BLENDMODE_ADD);
			
			drawImage.begin();
			ofBackground(0);
			ofPushStyle();
			ofSetColor(250);
			ofSetLineWidth(2);
			for( int i = 0; i < lines.size(); i++ ) {
				if( lines.at(i).z < 0 || lines.at(i).w < 0 ) continue;
				ofLine(lines.at(i).x, lines.at(i).y, lines.at(i).z, lines.at(i).w);
			}
			ofPopStyle();
			drawImage.end();
			
			drawImage.draw(0, 0);
			image.draw(0, 0);
			
			image.draw(image.getWidth(), 0);
			kinect.draw(image.getWidth(), 0);
			
			ofSetColor(255);
			ofDrawBitmapString(ofToString(ofGetFrameRate()) + " fps", 20, 20);
			
			ofDisableBlendMode();
			ofEnableDepthTest();
		}
		
		ofSetColor(255);
		if( cameraMode == EASYCAM_MODE )
			mesh.draw();
		glPointSize(3);
		target.drawWireframe();
		glMultMatrixf((GLfloat*)modelMat.getPtr());
		drawImage.getTextureReference().bind();
		initMesh.draw();
		drawImage.getTextureReference().unbind();
		
		if(cameraMode == EASYCAM_MODE) {
			cam.end();
		}
		
	}
}

void ofApp::keyPressed(int key) {
	if( pathLoaded ) {
		
		switch(key) {
			case '1': cameraMode = EASYCAM_MODE; break;
			case '2': cameraMode = PRO_MODE; break;
			case '3': cameraMode = CAM_MODE; break;
		}
		
	}
	
	if( key == 'f' ) {
		ofToggleFullscreen();
	}
	
	if( key == ' ' ) {
		bReset = true;
		image.setFromPixels(kinect.getPixelsRef());
	}
}

void ofApp::mousePressed(int x, int y, int button) {
	lines.push_back(ofVec4f(x, y, -1, -1));
}

void ofApp::mouseDragged(int x, int y, int button) {
	lines.at(lines.size() - 1).z = x;
	lines.at(lines.size() - 1).w = y;
	lines.push_back(ofVec4f(x, y, -1, -1));
}

void ofApp::mouseReleased(int x, int y, int button) {
	lines.at(lines.size() - 1).z = x;
	lines.at(lines.size() - 1).w = y;
}

void ofApp::dragEvent(ofDragInfo dragInfo){
	if( !pathLoaded ) {
		for( int i = 0 ; i < dragInfo.files.size() ; i++ ) {
			rootDir.push_back(dragInfo.files[i]);
		}
		init();
		pathLoaded = true;
	}
}
