/*
 https://github.com/micuat/sharedFace
 
 Naoto Hieda <micuat@gmail.com> 2014
 */

#include "ofApp.h"
#include "libfreenect_registration.h"
#include "freenect_internal.h"

using namespace ofxActiveScan;

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
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
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
	fs["proWidth"] >> options.projector_width;
	fs["proHeight"] >> options.projector_height;
	fs["vertical_center"] >> options.projector_horizontal_center;
	fs["nsamples"] >> options.nsamples;
	
	cv::FileStorage cfs(ofToDataPath(rootDir[0] + "/calibration.yml"), cv::FileStorage::READ);
	cfs["proIntrinsic"] >> proIntrinsic;
	cfs["proExtrinsic"] >> proExtrinsic;
	
	proSize.width = options.projector_width;
	proSize.height = options.projector_height;
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
	contourFinder.getTracker().setPersistence(15);
	// an object can move up to 32 pixels per frame
	contourFinder.getTracker().setMaximumDistance(32);
	
	drawImage.allocate(kinect.width, kinect.height);
	
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
			ofImage irShifted;
			irShifted.allocate(640, 480, OF_IMAGE_GRAYSCALE);
			for( int y = 0; y < 480; y++ ) {
				for( int x = 0; x < 640; x++ ) {
					irShifted.setColor(x, y, 0);
				}
			}
#define REG_X_VAL_SCALE 256 // "fixed-point" precision for double -> int32_t conversion
			
#define S2D_PIXEL_CONST 10
#define S2D_CONST_OFFSET 0.375
			
#define DEPTH_SENSOR_X_RES 1280
#define DEPTH_MIRROR_X 0
			
#define DEPTH_MAX_METRIC_VALUE FREENECT_DEPTH_MM_MAX_VALUE
#define DEPTH_NO_MM_VALUE      FREENECT_DEPTH_MM_NO_VALUE
#define DEPTH_MAX_RAW_VALUE    FREENECT_DEPTH_RAW_MAX_VALUE
#define DEPTH_NO_RAW_VALUE     FREENECT_DEPTH_RAW_NO_VALUE
			
#define DEPTH_X_OFFSET 1
#define DEPTH_Y_OFFSET 1
#define DEPTH_X_RES 640
#define DEPTH_Y_RES 480
			freenect_registration* reg = &(kinect.getDevice()->registration);
			uint32_t target_offset = DEPTH_Y_RES * reg->reg_pad_info.start_lines;
			for( int y = 0; y < 480; y++ ) {
				for( int x = 0; x < 640; x++ ) {
					uint16_t metric_depth = kinect.getDistanceAt(x, y);//kinect.getPixels()[x + y * DEPTH_X_RES];
					uint32_t reg_index = DEPTH_MIRROR_X ? ((y + 1) * DEPTH_X_RES - x - 1) : (y * DEPTH_X_RES + x);
					uint32_t nx = (reg->registration_table[reg_index][0] + reg->depth_to_rgb_shift[metric_depth]) / REG_X_VAL_SCALE;
					uint32_t ny =  reg->registration_table[reg_index][1];
					
					// ignore anything outside the image bounds
					if (nx >= DEPTH_X_RES) continue;
					
					// convert nx, ny to an index in the depth image array
					uint32_t target_index = (DEPTH_MIRROR_X ? ((ny + 1) * DEPTH_X_RES - nx - 1) : (ny * DEPTH_X_RES + nx)) - target_offset;
					
					// get the current value at the new location
					uint16_t current_depth = kinect.getRawDepthPixels()[target_index];
					
					irShifted.getPixels()[target_index] = kinect.getPixels()[x + y * DEPTH_X_RES];
				}
			}
			irShifted.update();
			ofImage blurred;
			ofxCv::blur(irShifted, blurred, 3);
			contourFinder.findContours(blurred);
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
			ofSetupScreenPerspective(options.projector_width, options.projector_height);
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
			
			ofDisableBlendMode();
			ofEnableDepthTest();
		}
		
		drawPointCloud();
		
		if(cameraMode == EASYCAM_MODE) {
			cam.end();
		}
		
	}
}


void ofApp::drawPointCloud() {
	
	int w = 640;
	int h = 480;
	mesh.clear();
	mesh.setMode(OF_PRIMITIVE_POINTS);
	glPointSize(2);
	int step = 2;
	
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0 ) {
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
				mesh.addTexCoord(ofVec2f(x, y));
			}
		}
	}
	
	ofMesh markers, markersPlane;
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
		if( count < 2 ) continue;
		centerDistance /= count;
		float threshold = 1.f;
		
		ofVec3f marker(kinect.getWorldCoordinateAt(rect.getCenter().x, rect.getCenter().y, centerDistance));
		markers.addVertex(marker);
		markers.addColor(ofColor::violet);
		markersPlane.addVertex(ofxCv::toOf(contourFinder.getCenter(i)));
	}
	
	if( markers.getNumVertices() > 0 ) {
		
		ofVec3f trMarker = markers.getVertex(0);
		ofVec3f blMarker = markers.getVertex(0);
		ofVec3f brMarker = markers.getVertex(0);
		float trDist = markersPlane.getVertex(0).distance(ofVec2f(kinect.width, 0));
		float blDist = markersPlane.getVertex(0).distance(ofVec2f(0, kinect.height));
		float brDist = markersPlane.getVertex(0).distance(ofVec2f(kinect.width, kinect.height));
		for( int i = 1; i < markers.getNumVertices(); i++ ) {
			ofVec2f p = markersPlane.getVertex(i);
			if( p.distance(ofVec2f(kinect.width, 0)) < trDist ) trMarker = markers.getVertex(i);
			if( p.distance(ofVec2f(0, kinect.height)) < blDist ) blMarker = markers.getVertex(i);
			if( p.distance(ofVec2f(kinect.width, kinect.height)) < brDist ) brMarker = markers.getVertex(i);
		}
		
		target.clear();
		target.setMode(OF_PRIMITIVE_TRIANGLES);
		target.addVertex(trMarker);
		target.addVertex(blMarker);
		target.addVertex(brMarker);
		target.addTriangle(0, 1, 2);
		target.addTexCoord(ofVec2f(500, 0));
		target.addTexCoord(ofVec2f(0, 500));
		target.addTexCoord(ofVec2f(500, 500));
		
		if( initTarget.getNumVertices() == 0 || bReset ) {
			initTarget = target;
		}
		
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
		cv::SVD svd(H);
		cv::Mat R = svd.vt.t() * svd.u.t();
		cv::Mat T = -R * centroid0 + centroidC;
		modelMat.set(R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), T.at<float>(0), R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), T.at<float>(1), R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), T.at<float>(2), 0, 0, 0, 1);
	}
	
	if( initMesh.getNumVertices() == 0 || bReset ) {
		double distanceProj = getLongestDistance(target);
		
		initMesh.clear();
		initMesh.setMode(OF_PRIMITIVE_POINTS);
		for( int i = 0; i < mesh.getNumVertices(); i++ ) {
			for( int j = 0; j < target.getNumVertices(); j++ ) {
				if( mesh.getVertex(i).distance(target.getVertex(j)) < distanceProj * 2.0 ) {
					initMesh.addVertex(mesh.getVertex(i));
//					ofColor col;
//					col.setHsb(ofMap(mesh.getVertex(i).y, -100, 100, 0, 255), 255, 255);
//					initMesh.addColor(col);
					initMesh.addTexCoord(mesh.getTexCoord(i));
					break;
				}
			}
		}
		bReset = false;
	}
	
	ofSetColor(255);
	if( cameraMode == EASYCAM_MODE )
		mesh.draw();
	glPointSize(3);
	glMultMatrixf((GLfloat*)ofMatrix4x4::getTransposedOf(modelMat).getPtr());
	drawImage.getTextureReference().bind();
	initMesh.draw();
	drawImage.getTextureReference().unbind();
//	ofSetColor(255);
//	image.bind();
//	target.draw();
//	image.unbind();
	

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
