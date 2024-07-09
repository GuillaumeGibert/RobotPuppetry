#include <dlib/opencv.h>
#include <opencv2/opencv.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include "render_face.hpp"
#include "DynamixelHandler.h"

using namespace dlib;
using namespace std;

#define FACE_DOWNSAMPLE_RATIO 2
#define DISPLAY_DOWNSAMPLE_RATIO 4
#define SKIP_FRAMES 5
#define FPS 30.0
#define VERBOSE false

DynamixelHandler _oDxlHandler;
std::string _poppyDxlPortName = "/dev/ttyUSB0";
float _poppyDxlProtocol = 2.0;
int _poppyDxlBaudRate = 1000000;
int _nbJoints = 6;
float _minJointCmd = 0;
float _maxJointCmd = 1023;
float _minJointAngle = -180.0f;
float _maxJointAngle = 180.0f;
double fps = 30.0;


std::vector<cv::Point3d> get_3d_model_points()
{
	std::vector<cv::Point3d> modelPoints; 

	modelPoints.push_back(cv::Point3d(0.0f, 0.0f, 0.0f)); //The first must be (0,0,0) while using POSIT
	modelPoints.push_back(cv::Point3d(0.0f, -330.0f, -65.0f));
	modelPoints.push_back(cv::Point3d(-225.0f, 170.0f, -135.0f));
	modelPoints.push_back(cv::Point3d(225.0f, 170.0f, -135.0f));
	modelPoints.push_back(cv::Point3d(-150.0f, -150.0f, -125.0f));
	modelPoints.push_back(cv::Point3d(150.0f, -150.0f, -125.0f));

	return modelPoints;
}

std::vector<cv::Point2d> get_2d_image_points(full_object_detection &d)
{
	std::vector<cv::Point2d> image_points;
	image_points.push_back( cv::Point2d( d.part(30).x(), d.part(30).y() ) );    // Nose tip
	image_points.push_back( cv::Point2d( d.part(8).x(), d.part(8).y() ) );      // Chin
	image_points.push_back( cv::Point2d( d.part(36).x(), d.part(36).y() ) );    // Left eye left corner
	image_points.push_back( cv::Point2d( d.part(45).x(), d.part(45).y() ) );    // Right eye right corner
	image_points.push_back( cv::Point2d( d.part(48).x(), d.part(48).y() ) );    // Left Mouth corner
	image_points.push_back( cv::Point2d( d.part(54).x(), d.part(54).y() ) );    // Right mouth corner
	
	return image_points;
}

double determineMouthOpening(full_object_detection &d)
{
	double distance = 0.;
	distance = pow(d.part(62).x()-d.part(66).x(), 2) + pow(d.part(62).y()-d.part(66).y(), 2);
	return sqrt(distance);
}

cv::Mat get_camera_matrix(float focal_length, cv::Point2d center)
{
	cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
	return camera_matrix;
}

int convertAnglesToJointCmd(float fJointAngle)
{
	// y = ax + b
	float a =  (_maxJointCmd-_minJointCmd) / (_maxJointAngle - _minJointAngle);
	float b = _minJointCmd - a * _minJointAngle;
	float jointCmd = a * fJointAngle + b;
	return (int)jointCmd;
}

void goToHomePosition()
{
	std::vector<uint16_t> l_vTargetJointPosition;
	for (int l_joint = 0; l_joint < _nbJoints; l_joint++)
		l_vTargetJointPosition.push_back(convertAnglesToJointCmd(0.0f));
	
	l_vTargetJointPosition[2] = convertAnglesToJointCmd(-90.0f);
	l_vTargetJointPosition[3] = convertAnglesToJointCmd(90.0f);;
		
	_oDxlHandler.sendTargetJointPosition(l_vTargetJointPosition);
}


void moveRobotTo(float roll, float pitch, float mouth)
{
	std::vector<uint16_t> l_vTargetJointPosition;
	for (int l_joint = 0; l_joint < _nbJoints; l_joint++)
		l_vTargetJointPosition.push_back(convertAnglesToJointCmd(0.0f));
	
	l_vTargetJointPosition[2] = convertAnglesToJointCmd(-90.0f);
	l_vTargetJointPosition[3] = convertAnglesToJointCmd(90.0f+roll);
	l_vTargetJointPosition[4] = convertAnglesToJointCmd(pitch);
	l_vTargetJointPosition[5] = convertAnglesToJointCmd(-mouth);
	
	_oDxlHandler.sendTargetJointPosition(l_vTargetJointPosition);
}


cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{

	float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

	bool singular = sy < 1e-6; // If
	float x, y, z;

	if (!singular)
	{
		x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
		y = atan2(-R.at<double>(2,0), sy);
		z = atan2(R.at<double>(1,0), R.at<double>(0,0));
	}
	else
	{
		x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
		y = atan2(-R.at<double>(2,0), sy);
		z = 0;
	}

	return cv::Vec3f(x, y, z); 
}

int main()
{
	std::cout << "[INFO] Launch communication with Dynamixel motors... " << std::endl;
	_oDxlHandler.setDeviceName(_poppyDxlPortName);
	_oDxlHandler.setProtocolVersion(_poppyDxlProtocol);
	_oDxlHandler.openPort();
	_oDxlHandler.setBaudRate(_poppyDxlBaudRate);
	_oDxlHandler.enableTorque(true);
	std::cout << std::endl;
	
	goToHomePosition();
	
	//--- Load face detection and pose estimation models.
	std::cout << "[INFO] Loading face model... ";
	frontal_face_detector detector;
	shape_predictor pose_model;
	try
	{
		detector = get_frontal_face_detector();
		deserialize("../data/shape_predictor_68_face_landmarks.dat") >> pose_model;
		std::cout << " DONE!"<< std::endl;
	}
	catch(serialization_error& e)
	{
		cout << "You need dlib's default face landmarking model file to run this example." << endl;
		cout << "You can get it from the following URL: " << endl;
		cout << "   http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2" << endl;
		cout << endl << e.what() << endl;
	}
	catch(exception& e)
	{
		cout << e.what() << endl;
	}
	
	// retrieve 3D model points that will be used to estimate head orientation
	std::vector<cv::Point3d> model_points = get_3d_model_points();

	
	//--- INITIALIZE VIDEOCAPTURE
	std::cout << "[INFO] Opening camera... ";
	cv::VideoCapture cap;
	int deviceID = 0;             // 0 = open default camera
	int apiID = cv::CAP_V4L2;      // 0 = autodetect default API
	// open selected camera using selected API
	cap.open(deviceID, apiID);
	// check if we succeeded
	if (!cap.isOpened())
	{
		std::cout << " FAILED!"<< std::endl;
		std::cerr << "[ERROR] Unable to open camera!" << std::endl;
		return 2;
	}
	else
	{
		std::cout << " DONE!"<< std::endl;
	}
	
	// Get first frame and allocate memory.
	cv::Mat frame;
	cap >> frame;
	if (frame.empty()) 
	{
		std::cerr << "[ERROR] blank frame grabbed" << std::endl;
		return 3;
	}
	cv::Mat im_small, im_display;
	cv::resize(frame, im_small, cv::Size(), 1.0/FACE_DOWNSAMPLE_RATIO, 1.0/FACE_DOWNSAMPLE_RATIO);
	cv::resize(frame, im_display, cv::Size(), 1.0/DISPLAY_DOWNSAMPLE_RATIO, 1.0/DISPLAY_DOWNSAMPLE_RATIO);
	cv::Size size = frame.size();

	image_window win;
	std::vector<rectangle> faces;
	double t = (double)cv::getTickCount();
	cv::Mat rotation_vector(3,1,CV_64F);
	cv::Mat translation_vector(3,1,CV_64F);
	
	int count = 0;
	
	while(true)
	{
		if (count == 0)
			t = cv::getTickCount();
		
		// Grab a frame
		cap >> frame;
		if (frame.empty()) 
		{
			std::cerr << "[ERROR] blank frame grabbed" << std::endl;
			break;
		}

		// Resize image for face detection
		cv::resize(frame, im_small, cv::Size(), 1.0/FACE_DOWNSAMPLE_RATIO, 1.0/FACE_DOWNSAMPLE_RATIO);

		// Change to dlib's image format. No memory is copied.
		cv_image<bgr_pixel> cimg_small(im_small);
		cv_image<bgr_pixel> cimg(frame);

		// Detect faces 
		if (count % SKIP_FRAMES == 0)
			faces = detector(cimg_small);
		
		if (VERBOSE)
			cout<< "Number of faces detected= " << faces.size() << endl;
		
		// Find the pose of the first face.
		if (faces.size() > 0)
		{
			rectangle r(
			(long)(faces[0].left() * FACE_DOWNSAMPLE_RATIO),
			(long)(faces[0].top() * FACE_DOWNSAMPLE_RATIO),
			(long)(faces[0].right() * FACE_DOWNSAMPLE_RATIO),
			(long)(faces[0].bottom() * FACE_DOWNSAMPLE_RATIO)
			);
			full_object_detection shape = pose_model(cimg, r);
			
			render_face(frame, shape);
			std::vector<cv::Point2d> image_points = get_2d_image_points(shape);
			double focal_length = frame.cols;
			cv::Mat camera_matrix = get_camera_matrix(focal_length, cv::Point2d(frame.cols/2,frame.rows/2));
			
			cv::Mat rotation_matrix;
			

			cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type);
			cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector, false);
			
			cv::Rodrigues(rotation_vector, rotation_matrix);
			cv::Vec3f rpy = rotationMatrixToEulerAngles(rotation_matrix);
			//std::cout << "rpy = (" << rpy[0]/3.14*180 << ",  " << rpy[1]/3.14*180 << ",  " << rpy[2]/3.14*180 << ") " << std::endl; 
			
			float roll;
			if (rpy[0]/3.14*180 > 0)
				roll = 180.0-rpy[0]/3.14*180;
			else
				roll = -180.0-rpy[0]/3.14*180;
			//std::cout << "roll = " << roll << std::endl; 
			double distance = determineMouthOpening(shape);
			//std::cout << "distance = " << distance << std::endl; 
			
			moveRobotTo(roll, rpy[1]/3.14*180, distance);
			std::vector<cv::Point3d> nose_end_point3D;
			std::vector<cv::Point2d> nose_end_point2D;
			nose_end_point3D.push_back(cv::Point3d(0,0,1000.0));

			cv::projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);		
			//                cv::Point2d projected_point = find_projected_point(rotation_matrix, translation_vector, camera_matrix, cv::Point3d(0,0,1000.0));
			cv::line(frame,image_points[0], nose_end_point2D[0], cv::Scalar(255,0,0), 2);
			//                cv::line(im,image_points[0], projected_point, cv::Scalar(0,0,255), 2);

		}
		// see FPS    
		cv::putText(frame, cv::format("fps %.2f",fps), cv::Point(50, size.height - 50), cv::FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(0, 0, 255), 3);

		// Resize image for display
		im_display = frame;
		cv::resize(frame, im_display, cv::Size(), 1.0/DISPLAY_DOWNSAMPLE_RATIO, 1.0/DISPLAY_DOWNSAMPLE_RATIO);
		cv::imshow("Fast Facial Landmark Detector", im_display);

		// WaitKey slows down the runtime quite a lot
		//if (cv::waitKey(1000.0/FPS) >= 0)
		//	break;
		if (count % 15 == 0)
		{
			int k = cv::waitKey(1);
			if (k == 'q')
				return 0;
		}
		
		count++;
		if (count == 100)
		{
			t = ((double)cv::getTickCount() -t)/cv::getTickFrequency();
			fps = 100.0/t;
			count = 0;
			
		}

	}
	
	_oDxlHandler.enableTorque(false);
	_oDxlHandler.closePort();
	
	// destroy all gui
	cv::destroyAllWindows();
	// release camera
	cap.release();
	
	return 0;
}

