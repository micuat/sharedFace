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
		rootDir.push_back("/Users/naoto/Documents/openFrameworks-0.8.3/addons/ofxActiveScan/141021miraikan/");
		init();
		pathLoaded = true;
		
//		pathLoaded = false;
	}
}

void ofApp::init() {
	ofSetLogLevel(OF_LOG_WARNING);
	
	receiver.setup(PORT);
	
	// acquire color image
	
	// enable depth->video image calibration
	kinect.setRegistration(false);
    
	kinect.init();
	while( !kinect.isFrameNewVideo() ) {
		kinect.update();
		image.setFromPixels(kinect.getPixelsRef());
		ofSleepMillis(500);
	}
	kinect.close();
	
	// save image for processing.js
	image.saveImage("/Users/naoto/Documents/Programs/ProcessingJsOsc/nodeClient/template/image.jpg");
	image.saveImage("/Users/naoto/Documents/Programs/ProcessingJsOsc/nodeClient/web-export/image.jpg");
	
	
	// re-open with IR mode
	kinect.init(true);
	
	kinect.open();		// opens first available kinect
	
	cameraMode = EASYCAM_MODE;
	
	
	// load calibration parameters
	cv::FileStorage fs(ofToDataPath(rootDir[0] + "/config.yml"), cv::FileStorage::READ);
	fs["proWidth"] >> proSize.width;
	fs["proHeight"] >> proSize.height;
	
	float lensDist;
	cv::FileStorage cfs(ofToDataPath(rootDir[0] + "/calibration.yml"), cv::FileStorage::READ);
	cfs["proIntrinsic"] >> proIntrinsic;
	cfs["proExtrinsic"] >> proExtrinsic;
	cfs["radialLensDistortion"] >> lensDist;
	
	cout << proIntrinsic << endl;
	cout << proExtrinsic << endl;
	
	// set parameters for projection
	proCalibration.setup(proIntrinsic, proSize);
	
	
	// tracker setup
	contourFinder.setMinAreaRadius(3);
	contourFinder.setMaxAreaRadius(15);
	contourFinder.setFindHoles(true);
	contourFinder.setTargetColor(0);
	contourFinder.setThreshold(200);
	// wait for half a frame before forgetting something
	contourFinder.getTracker().setPersistence(60);
	// an object can move up to 32 pixels per frame
	contourFinder.getTracker().setMaximumDistance(32);
	
	roi = cv::Rect(160, 0, 320, 480);
	
	drawImage.allocate(kinect.width * RES_MULT, kinect.height * RES_MULT);
	
	
	// Kalman filter
	kalmanPosition.init(30000.0, 30000.0);
	kalmanEuler.init(30000.0, 30000.0);
	for( int i = 0; i < NUM_MARKERS; i++ ) {
		ofxCv::KalmanPosition kPos;
		kPos.init(30000.0, 30000.0);
		kalmanMarkers.push_back(kPos);
	}
	
	// stamps
	ofImage stamp;
	stamp.loadImage(ofToDataPath("eye1r.png"));
	stamps.push_back(stamp);
	stamp.loadImage(ofToDataPath("eye1l.png"));
	stamps.push_back(stamp);
	stamp.loadImage(ofToDataPath("eye2r.png"));
	stamps.push_back(stamp);
	stamp.loadImage(ofToDataPath("eye2l.png"));
	stamps.push_back(stamp);
	stamp.loadImage(ofToDataPath("kizu.png"));
	stamps.push_back(stamp);
	stamp.loadImage(ofToDataPath("star.png"));
	stamps.push_back(stamp);
	stamp.loadImage(ofToDataPath("ball1.png"));
	stamps.push_back(stamp);
	stamps.push_back(stamp);
	stamp.loadImage(ofToDataPath("ball2.png"));
	stamps.push_back(stamp);
	stamp.loadImage(ofToDataPath("ball3.png"));
	stamps.push_back(stamp);
	stamps.push_back(stamp);
	
	stamp.loadImage(ofToDataPath("blinkr.png"));
	stampsBlink.push_back(stamp);
	stamp.loadImage(ofToDataPath("blinkl.png"));
	stampsBlink.push_back(stamp);

	stampCoord.resize(stamps.size());
	
	jsMode = 0; // pen or stamp; really need this?
	shaderMode = 0; // 0 for turn off, others for shader effects
	drawPointCloud = false;
	dynamicPen = false;
	
	// distortion shader
#define STRINGIFY(A) #A
	const char *src1v = STRINGIFY
   (
	uniform float dist;
	uniform vec2 ppoint;
	uniform float elapsedTime;
	void main(){
		
		gl_TexCoord[0] = gl_MultiTexCoord0;
		
		// projection as usual
		vec4 pos = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;
		gl_Position = pos;
		
		// xy with principal point origin
		vec2 shiftPos = pos.xy - ppoint;
		
		// lens distortion
		gl_Position.xy = shiftPos * (1.0 / (1.0 - dist * length(shiftPos))) + ppoint;
		
		vec4 col = gl_Color;
		gl_FrontColor = col;
		gl_TexCoord[0] = gl_MultiTexCoord0;
	}
	);
	
	const char *src1f = STRINGIFY
   (
	uniform float elapsedTime;
	uniform int shaderMode;
	uniform sampler2DRect texture1;
	//generate a random value from four points
	vec4 rand(vec2 A,vec2 B,vec2 C,vec2 D){ 
		
		vec2 s=vec2(12.9898,78.233); 
		vec4 tmp=vec4(dot(A,s),dot(B,s),dot(C,s),dot(D,s)); 
		
		return fract(sin(tmp) * 43758.5453)* 2.0 - 1.0; 
	} 
	
	//this is similar to a perlin noise function
	float noise(vec2 coord,float d){
		
		vec2 C[4]; 
		
		float d1 = 1.0/d;
		
		C[0]=floor(coord*d)*d1; 
		
		C[1]=C[0]+vec2(d1,0.0); 
		
		C[2]=C[0]+vec2(d1,d1); 
		
		C[3]=C[0]+vec2(0.0,d1);
		
		
		vec2 p=fract(coord*d); 
		
		vec2 q=1.0-p; 
		
		vec4 w=vec4(q.x*q.y,p.x*q.y,p.x*p.y,q.x*p.y); 
		
		return dot(vec4(rand(C[0],C[1],C[2],C[3])),w); 
	} 
	
	vec3 rgb2hsv(vec3 c) {
		vec4 K = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
		vec4 p = mix(vec4(c.bg, K.wz), vec4(c.gb, K.xy), step(c.b, c.g));
		vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));
		
		float d = q.x - min(q.w, q.y);
		float e = 1.0e-10;
		return vec3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);
	}
	
	vec3 hsv2rgb(vec3 c) {
		vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
		vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
		return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
	}
	void main() {
		vec2 pos = gl_TexCoord[0].st;
		
		vec4 col = texture2DRect(texture1, gl_TexCoord[0].st);
		vec4 col2 = vec4(0.0, 0.0, 0.0, 0.0);
		if( shaderMode == 1 ) {
			float val = fract(pos.x/20.0+elapsedTime*3.141592) * 0.8;
			float val2 = fract(pos.x/20.0-elapsedTime*3.141592) * 0.8;
			if( val > val2 ) {
				col2 = vec4(0, val, val, val * val);
			} else {
				col2 = vec4(val2, 0, 0, val2 * val2);
			}
		} else if( shaderMode == 2 ) {
			col2 = vec4(0.0, 1.0, 1.0, 1.0);
			col2.r += 0.10*noise(vec2(pos.x/1000.0 + elapsedTime/100.0, pos.y/10000.0 + elapsedTime/20.0), 200.0);
			col2.r += 0.05*noise(vec2(pos.x/100.0 - elapsedTime/100.0, pos.y/1000.0 + elapsedTime/50.0), 200.0);
			col2.rgb = hsv2rgb(col2.rgb);
		} else if( shaderMode == 3 ) {
			float val = fract(pos.y/20.0+elapsedTime*3.141592) * 0.8;
			float val2 = fract(pos.y/20.0-elapsedTime*3.141592) * 0.8;
			if( val > val2 ) {
				col2 = vec4(val, 0, val, val * val);
			} else {
				col2 = vec4(0, val2, 0, val2 * val2);
			}
		} else if( shaderMode == 4 ) {
			col2 = vec4(1.0, 0.0, 1.0, 1.0);
			col2.g += 0.10*noise(vec2(pos.x/1000.0 + elapsedTime/100.0, pos.y/10000.0 + elapsedTime/20.0), 200.0);
			col2.g += 0.05*noise(vec2(pos.x/100.0 - elapsedTime/100.0, pos.y/1000.0 + elapsedTime/50.0), 200.0);
			col2.rgb = hsv2rgb(col2.rgb);
		}
		if( col.r == 1.0 && col.g == 1.0 && col.b == 1.0 && shaderMode > 0 ) {
			col = col2;
		}
		gl_FragColor = col;
	}
	);
	const char *src2 = STRINGIFY
   (
	uniform float elapsedTime;
	
	//generate a random value from four points
	vec4 rand(vec2 A,vec2 B,vec2 C,vec2 D){ 
		
		vec2 s=vec2(12.9898,78.233); 
		vec4 tmp=vec4(dot(A,s),dot(B,s),dot(C,s),dot(D,s)); 
		
		return fract(sin(tmp) * 43758.5453)* 2.0 - 1.0; 
	} 
	
	//this is similar to a perlin noise function
	float noise(vec2 coord,float d){
		
		vec2 C[4]; 
		
		float d1 = 1.0/d;
		
		C[0]=floor(coord*d)*d1; 
		
		C[1]=C[0]+vec2(d1,0.0); 
		
		C[2]=C[0]+vec2(d1,d1); 
		
		C[3]=C[0]+vec2(0.0,d1);
		
		
		vec2 p=fract(coord*d); 
		
		vec2 q=1.0-p; 
		
		vec4 w=vec4(q.x*q.y,p.x*q.y,p.x*p.y,q.x*p.y); 
		
		return dot(vec4(rand(C[0],C[1],C[2],C[3])),w); 
	} 
	
	vec3 rgb2hsv(vec3 c) {
		vec4 K = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
		vec4 p = mix(vec4(c.bg, K.wz), vec4(c.gb, K.xy), step(c.b, c.g));
		vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));
		
		float d = q.x - min(q.w, q.y);
		float e = 1.0e-10;
		return vec3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);
	}
	
	vec3 hsv2rgb(vec3 c) {
		vec4 K = vec4(1.0, 2.0 / 3.0, 1.0 / 3.0, 3.0);
		vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
		return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
	}
	
	void main() {
		
		gl_TexCoord[0] = gl_MultiTexCoord0;
		
		//get our current vertex position so we can modify it
		vec4 pos = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;
		
		vec4 col = gl_Color;
		float n = noise( -elapsedTime*5.0 + vec2(sin(pos.z/100.0) + pos.y*100.0, 100.0), 1.0);
		gl_PointSize = n * 4.0 + 1.0;
		
		float th = 0.9;
		vec4 gridCol = vec4(0.0, n, 1.0, 1.0);
		vec4 decr = vec4(n*2.0, n*2.0, 0.7, 1.0);
		if( mix(0.0, 1.0, step(fract(pos.z*200.0+sin(elapsedTime*4.0)), 0.5)) < 1.0 ) {
			col = gridCol;
		} else {
			col = decr;
		}
		//finally set the pos to be that actual position rendered
		gl_Position = pos;
		
		vec3 hsb = rgb2hsv(col.xyz);
		hsb.x = fract(elapsedTime*0.1);
		col.xyz = hsv2rgb(hsb);
		
		gl_FrontColor = col;
		
	}
	);
	
	shader.setupShaderFromSource(GL_VERTEX_SHADER, src1v);
	shader.setupShaderFromSource(GL_FRAGMENT_SHADER, src1f);
	shader.linkProgram();
	
	shader.begin();
	shader.setUniform1f("dist", lensDist);
	shader.end();
	
	shader2.setupShaderFromSource(GL_VERTEX_SHADER, src2);
	shader2.linkProgram();
	
	
	ofEnableDepthTest();
}

void ofApp::update() {
	ofSetWindowTitle(ofToString(ofGetFrameRate()));
	
	kinect.update();
	if(kinect.isFrameNew()) {
		bool bFindHoles = false;
		if( bFindHoles ) {
			ofImage eroded;
			ofxCv::erode(kinect.getDepthPixelsRef(), eroded, 3);
			contourFinder.findContours(eroded);
		} else {
			ofImage blurred;
			ofxCv::erode(kinect.getPixelsRef(), blurred, 1);
			ofxCv::blur(blurred, 3);
//			ofxCv::blur(kinect.getPixelsRef(), blurred, 3);
			cv::Mat img = ofxCv::toCv(blurred);
			contourFinder.findContours(img(roi));
		}
		
		if( cameraMode == EASYCAM_MODE || drawPointCloud ) {
			updatePointCloud(mesh);
		}
		
		markers.clear();
		ofMesh markersProjected;
		vector<int> markerLabels;
		vector<cv::Point> sensorCoords;
		ofxCv::RectTracker& tracker = contourFinder.getTracker();
		if( contourFinder.size() == 4 ) {
			for(int i = 0; i < contourFinder.size(); i++) {
				sensorCoords.push_back(contourFinder.getCenter(i));
				sensorCoords.at(i).x += roi.x;
				sensorCoords.at(i).y += roi.y;
			}
			findVec3fFromFitting(sensorCoords, markers);
			for(int i = 0; i < contourFinder.size(); i++) {
				markersProjected.addVertex(kinect.sensorToColorCoordinate(ofxCv::toOf(sensorCoords.at(i)), markers.getVertex(i).z));
				markerLabels.push_back(contourFinder.getLabel(i));
			}
		} else {
			return;
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
				kalmanPosition.init(10000, 10000, true);
				kalmanEuler.init(10000, 10000, true);
				kalmanMarkers.clear();
				for( int i = 0; i < NUM_MARKERS; i++ ) {
					ofxCv::KalmanPosition kPos;
					kPos.init(10000, 10000);
					kalmanMarkers.push_back(kPos);
				}
				registeredLabels = registerMarkers(markers, markersProjected, markerLabels, target);
				bReset = false;
			} else { // rely on tracker
				updateTargetUsingLabels(markers, markerLabels, registeredLabels, target);
			}
			
			if( initTarget.getNumVertices() == 0 ) {
				initTarget = target;
			}
			
			updateMarkerKalmanFilter();
			modelMat = findRigidTransformation(target, initTarget);
			updateModelKalmanFilter();
			
			if( initMesh.getNumVertices() == 0 ) {
				updateInitMesh(markers, markersProjected);
			}
			
			updateReceiveOsc();
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
			if( distance > 0 && distance < 1500 ) {
				ofPoint coord = kinect.sensorToColorCoordinate(ofPoint(x, y));
				ofPoint p = kinect.getWorldCoordinateAt(coord.x, coord.y, distance);
				m.addVertex(p);
				m.addTexCoord(coord * RES_MULT);
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

void ofApp::findVec3fFromFitting(vector<cv::Point>& centers, ofMesh& markers) {
	vector<cv::Point> hull;
	vector<ofVec2f> centersOf;
	cv::Mat measurement = cv::Mat_<double>::zeros(0, 3); // rows 0, cols 3
	cv::Mat solution; // ax + by + cz = 1
	if( centers.size() == 4 ) {
		cv::convexHull(centers, hull, false );
		
		for( int i = 0; i < centers.size(); i++ ) {
			centersOf.push_back(ofxCv::toOf(centers.at(i)));
		}
		
		int count = 0;
		int dieCount = 0;
		int numSamples = 150;
		samples.clear();
		samples.setMode(OF_PRIMITIVE_POINTS);
		while( count < numSamples && dieCount < numSamples * 4 ) {
			dieCount++;
			float p0 = ofRandom(1.0);
			float p1 = ofRandom(1.0);
			
			ofVec2f v01 = centersOf.at(0).getInterpolated(centersOf.at(1), p0);
			ofVec2f v32 = centersOf.at(2).getInterpolated(centersOf.at(3), p0);
			ofVec2f sample = v01.getInterpolated(v32, p1);
			
			float dist = kinect.getDistanceAt(sample);
			if( dist > 0 ) {
				ofVec3f sample3d(sample.x, sample.y, dist);
				cv::Mat m = cv::Mat_<double>::zeros(1, 3);
				m.at<double>(0, 0) = sample3d.x;
				m.at<double>(0, 1) = sample3d.y;
				m.at<double>(0, 2) = sample3d.z;
				measurement.push_back(m);
				samples.addVertex(sample);
				count++;
			}
		}
		if( count < numSamples ) {
			for( int i = 0; i < centers.size(); i++ )
				markers.addVertex(centersOf.at(i));
			return;
		}
		cv::Mat oneMat = cv::Mat_<double>::ones(numSamples, 1);
		cv::Mat pinvMat = measurement.inv(cv::DECOMP_SVD);
		solution = pinvMat * oneMat;
		for( int i = 0; i < centers.size(); i++ ) {
			// z = (1 - ax - by) / c
			float z = (1.0 - solution.at<double>(0, 0) * centers.at(i).x - solution.at<double>(1, 0) * centers.at(i).y) / solution.at<double>(2, 0);
			ofVec3f v = kinect.sensorToColorCoordinate(ofxCv::toOf(centers.at(i)), z);
			markers.addVertex(kinect.getWorldCoordinateAt(v.x, v.y, z));
		}
	} else {
		for( int i = 0; i < centers.size(); i++ ) {
			markers.addVertex(ofVec3f());
		}
	}
}

vector<int> ofApp::registerMarkers(ofMesh& markers, ofMesh& markersProjected, vector<int>& markerLabels, ofMesh& markersRegistered) {
	vector<int> labels;
	labels.resize(NUM_MARKERS, markerLabels.at(0));
	
	ofVec3f tlMarker = markers.getVertex(0);
	ofVec3f trMarker = markers.getVertex(0);
	ofVec3f blMarker = markers.getVertex(0);
	ofVec3f brMarker = markers.getVertex(0);
	float tlDist = markersProjected.getVertex(0).distance(ofVec2f(0, 0));
	float trDist = markersProjected.getVertex(0).distance(ofVec2f(kinect.width, 0));
	float blDist = markersProjected.getVertex(0).distance(ofVec2f(0, kinect.height));
	float brDist = markersProjected.getVertex(0).distance(ofVec2f(kinect.width, kinect.height));
	for( int i = 1; i < markers.getNumVertices(); i++ ) {
		ofVec2f p = markersProjected.getVertex(i);
		
		float curtlDist = p.distance(ofVec2f(0, 0));
		float curtrDist = p.distance(ofVec2f(kinect.width, 0));
		float curblDist = p.distance(ofVec2f(0, kinect.height));
		float curbrDist = p.distance(ofVec2f(kinect.width, kinect.height));
		if( curtlDist < tlDist ) {
			tlDist = curtlDist;
			tlMarker = markers.getVertex(i);
			labels.at(0) = markerLabels.at(i);
		}
		if( curtrDist < trDist ) {
			trDist = curtrDist;
			trMarker = markers.getVertex(i);
			labels.at(1) = markerLabels.at(i);
		}
		if( curblDist < blDist ) {
			blDist = curblDist;
			blMarker = markers.getVertex(i);
			labels.at(2) = markerLabels.at(i);
		}
		if( curbrDist < brDist ) {
			brDist = curbrDist;
			brMarker = markers.getVertex(i);
			labels.at(3) = markerLabels.at(i);
		}
	}
	
	markersRegistered.clear();
	markersRegistered.addVertex(tlMarker);
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
	//float dt = 4.46f;
	float dt = 0;
	
	// predict centroid
	kalmanPosition.update(cC);
	cC = kalmanPosition.getEstimation();
	ofVec3f v = kalmanPosition.getVelocity();
	cC = cC + v * dt;
	
	dt = 0;
	
	// predict Euler angle
	cv::SVD svd(H);
	cv::Mat R = svd.vt.t() * svd.u.t();
	mat.set(R.at<float>(0, 0), R.at<float>(1, 0), R.at<float>(2, 0), 0,
			R.at<float>(0, 1), R.at<float>(1, 1), R.at<float>(2, 1), 0,
			R.at<float>(0, 2), R.at<float>(1, 2), R.at<float>(2, 2), 0,
			0, 0, 0, 1);
	kalmanEuler.update(mat.getRotate());
	ofQuaternion qPredicted = kalmanEuler.getEstimation();
	
	// update model matrix
	mat.setRotate(qPredicted);
	ofVec3f T = cC - ofMatrix4x4::getTransposedOf(mat) * c0; // mat must be transposed for multiplication
	mat.setTranslation(T);
	return mat;
}

void ofApp::updateModelKalmanFilter() {
}

void ofApp::updateInitMesh(ofMesh& ms, ofMesh& markersProjected) {
	double distanceProj = getLongestDistance(target);
	
	ofxDelaunay triangulation;
	
	initMesh.clear();
	initMesh.setMode(OF_PRIMITIVE_TRIANGLES);
	for( int i = 0; i < mesh.getNumVertices(); i++ ) {
		for( int j = 0; j < target.getNumVertices(); j++ ) {
			if( mesh.getVertex(i).distance(target.getVertex(j)) < distanceProj * 1.5 ) {
				initMesh.addVertex(mesh.getVertex(i));
				initMesh.addTexCoord(mesh.getTexCoord(i));
				triangulation.addPoint(mesh.getVertex(i));
				break;
			}
		}
	}
	if( initMesh.getNumVertices() > 0 ) {
//		for( int i = 0; i < target.getNumVertices(); i++ ) {
//			initMesh.addVertex(target.getVertex(i));
//			initMesh.addTexCoord(markersProjected.getVertex(i));
//			triangulation.addPoint(target.getVertex(i));
//		}
	}
	triangulation.triangulate();
	const ofMesh& tr = triangulation.triangleMesh;
	float thDistance = 100.0;
	// add only when the triangle side lengths are below threshold
	for( int i = 0; i < tr.getNumIndices(); i+=3 ) {
		if( tr.getVertex(tr.getIndex(i  )).distance(tr.getVertex(tr.getIndex(i+1))) > thDistance ) continue;
		if( tr.getVertex(tr.getIndex(i+1)).distance(tr.getVertex(tr.getIndex(i+2))) > thDistance ) continue;
		if( tr.getVertex(tr.getIndex(i+2)).distance(tr.getVertex(tr.getIndex(i  ))) > thDistance ) continue;
		initMesh.addIndex(tr.getIndex(i  ));
		initMesh.addIndex(tr.getIndex(i+1));
		initMesh.addIndex(tr.getIndex(i+2));
	}
}

void ofApp::updateReceiveOsc() {
	while(receiver.hasWaitingMessages()){
		// get the next message
		ofxOscMessage m;
		receiver.getNextMessage(&m);
		
		float x = m.getArgAsFloat(0);
		float y = m.getArgAsFloat(1);
		// check for mouse moved message
		if(m.getAddress() == "/pen/coord"){
			float i = m.getArgAsFloat(2);
			ofColor c(i==0?255:0, i==1?255:0, i==2?255:0);
			float weight = 10.0;
			if( (fatLines.end()-1)->size() < weight ) weight = (fatLines.end()-1)->size();
			(fatLines.end()-1)->add(ofVec2f(x, y), c, weight/7.5);
			(fatLines.end()-1)->update();
		}
		else if(m.getAddress() == "/pen/pressed"){
			float i = m.getArgAsFloat(2);
			ofColor c(i==0?255:0, i==1?255:0, i==2?255:0);
			ofxFatLine fatLine;
			fatLine.setFeather(0);
			fatLine.add(ofVec2f(x, y), c, 0);
			fatLine.update();
			fatLines.push_back(fatLine);
		}
		else if(m.getAddress() == "/pen/released"){
			float i = m.getArgAsFloat(2);
			ofColor c(i==0?255:0, i==1?255:0, i==2?255:0);
			(fatLines.end()-1)->add(ofVec2f(x, y), c, 0);
			float weight = 10.0;
			if( (fatLines.end()-1)->size() > weight ) {
				for( int i = 0; i < weight; i++ ) {
					(fatLines.end()-1)->updateWeight((fatLines.end()-1)->size() - i, i/7.5);
				}
			}
			(fatLines.end()-1)->update();
		}
		else if(m.getAddress() == "/pen/erase"){
			fatLines.clear();
			
			for( int i = 0; i < stampCoord.size(); i++ ) {
				stampCoord.at(i).x = 0;
				stampCoord.at(i).y = 0;
			}
		}
		else if(m.getAddress() == "/pen/undo"){
			if( fatLines.size() > 0 ) fatLines.pop_back();
		}
		else if(m.getAddress() == "/stamp/coord" || m.getAddress() == "/stamp/pressed"){
			int curStamp = m.getArgAsInt32(3);
			if( ofInRange(curStamp, 0, stampCoord.size()-1) ) {
				stampCoord.at(curStamp).x = x;
				stampCoord.at(curStamp).y = y;
			}
		}
		else if(m.getAddress() == "/mode/change"){
			int mode = m.getArgAsInt32(3);
			if( mode <= 1 ) {
				//shaderMode = 0;
				jsMode = mode;
			}
			else if( mode == 6 ) dynamicPen = !dynamicPen;
			else if( mode == 7 ) drawPointCloud = !drawPointCloud;
			else {
				int shaderModeToBe = mode - 2;
				shaderMode = shaderModeToBe; // new one
			}
			
		}
	}
}

void ofApp::draw() {
	if( pathLoaded ) {
		
		float curTime = ofGetElapsedTimef();
		
		ofBackground(0);
		
		// fbo texture rendering
		drawImage.begin();
		
		ofScale(RES_MULT, RES_MULT); // scale for hyper resolution
		ofBackground(255, 255);

		ofPushStyle();
		
		ofSetColor(255, 255);
		ofEnableAlphaBlending();
		ofDisableDepthTest();
		
		// stamps
		float curNoise = ofNoise(ofGetElapsedTimef());
		for( int i = 0; i < stamps.size(); i++ ) {
			ofImage* stamp;
			if( i <= 1 && curNoise > 0.6 ) stamp = &stampsBlink.at(i);
			else if( i <= 3 && curNoise > 0.6 ) stamp = &stampsBlink.at(i-2);
			else stamp = &stamps.at(i);
			ofPushMatrix();
			ofTranslate(stampCoord.at(i));
			float a = 0.6 * 0.33;
			ofScale(a, a, 1);
			stamp->draw(-stamp->getWidth()/2, -stamp->getHeight()/2);
			ofPopMatrix();
		}
		
		ofEnableDepthTest();
		
		// lines
		ofSetLineWidth(2 * RES_MULT);
		for( int i = fatLines.size() - 1; i >= 0; i-- ) {
			ofxFatLine& fl = fatLines.at(i);
			if( fl.getColor(0) == ofColor::black ) {
				ofxFatLine flColor = fl;
				for( int j = 0; j < flColor.size(); j++ ) {
					ofFloatColor c;
					float hue = j/32.0;
					if( true ) hue += 2.0 * ofGetElapsedTimef();
					c.setHsb(hue - (int)hue, 1.0, 1.0);
					flColor.updateColor(j, c);
				}
				flColor.update();
				flColor.draw();
			} else {
				ofxFatLine flColor = fl;
				for( int j = 0; j < flColor.size(); j++ ) {
					ofFloatColor c = fl.getColor(j);
					float br = 0.99;
					if( dynamicPen ) br = j/32.0 + 2.0 * ofGetElapsedTimef();
					c.setBrightness(br - (int)br);
					flColor.updateColor(j, c);
				}
				flColor.update();
				flColor.draw();
			}
		}
		
		ofPopStyle();
		
		ofDisableAlphaBlending();
		drawImage.end();
		
		if(cameraMode == EASYCAM_MODE) {
			cam.begin();
			ofScale(1, -1, -1);
			ofTranslate(0, 0, -2);
			ofPushStyle();
			ofSetColor(ofColor::pink);
			glPointSize(3);
			markers.drawVertices();
			ofPopStyle();
		} else if(cameraMode == PRO_MODE) {
			ofSetupScreenPerspective(proSize.width, proSize.height);
			proCalibration.loadProjectionMatrix(0.0001, 100000000.0);
			ofViewport(moveKey.x, moveKey.y);
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
			
			ofPushMatrix();
			ofScale(1.0 / RES_MULT, 1.0 / RES_MULT);
			drawImage.draw(0, 0);
			ofPopMatrix();
			image.draw(0, 0);
			
			image.draw(image.getWidth(), 0);
			kinect.draw(image.getWidth(), 0);
			
			ofPushStyle();
			ofPushMatrix();
			ofTranslate(image.getWidth(), 0, 0);
			ofSetColor(255);
			glPointSize(1);
			samples.draw();
			ofPopMatrix();
			ofPopStyle();
			
			ofSetColor(255);
			ofDrawBitmapString(ofToString(ofGetFrameRate()) + " fps", 20, 20);
			
			ofDisableBlendMode();
			ofEnableDepthTest();
		}
		
		ofSetColor(255);
		if( cameraMode == EASYCAM_MODE || drawPointCloud ) {
			ofDisableDepthTest();
			shader2.begin();
			shader2.setUniform1f("elapsedTime", ofGetElapsedTimef());
			mesh.draw();
			shader2.end();
			ofEnableDepthTest();
		}
		
		glPointSize(3);
		glMultMatrixf((GLfloat*)modelMat.getPtr());
		
		shader.begin();
		shader.setUniform2f("ppoint", proIntrinsic.at<double>(0, 2) / ofGetWidth(), proIntrinsic.at<double>(1, 2) / ofGetHeight());
		shader.setUniform1f("elapsedTime", ofGetElapsedTimef());
		shader.setUniform1i("shaderMode", shaderMode);
		shader.setUniformTexture("texture1", drawImage.getTextureReference(), 0);
		initMesh.draw();
		
		if( cameraMode == EASYCAM_MODE ) {
			cam.end();
		}
		
		shader.end();
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
	if( key == OF_KEY_UP ) {
		moveKey.y-=1;
	}
	if( key == OF_KEY_DOWN ) {
		moveKey.y+=1;
	}
	if( key == OF_KEY_LEFT ) {
		moveKey.x-=1;
	}
	if( key == OF_KEY_RIGHT ) {
		moveKey.x+=1;
	}
	ofLogWarning() << moveKey;

}

void ofApp::mousePressed(int x, int y, int button) {
}

void ofApp::mouseDragged(int x, int y, int button) {
}

void ofApp::mouseReleased(int x, int y, int button) {
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
