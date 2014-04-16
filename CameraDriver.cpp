/***********************************************************************************************************************************

CameraDriver.cpp
Written by Giovanni Sutanto (gsutanto@usc.edu or giovanni.sutanto@gmail.com), February 2014
Special Thanks to Emil Valkov for his work on RaspberryPiCamera-OpenCV Libraries inside RaspiCamCV.h
See Emil Valkov work at: https://robidouille.wordpress.com/2013/10/19/raspberry-pi-camera-with-opencv/

This file contains functions and procedures specific to the camera being used to take images/videos.
Right now there are two camera-specific parameters defined:
1) Parameters for Raspberry Pi Camera
2) Parameters for Giovanni Sutanto's T430s Laptop's Camera
This file does NOT need to be copied into Raspberry Pi, because Raspberry Pi already has its own file, specific to its camera.

*************************************************************************************************************************************/

#include "PoseEstimation.h"

// ON RPi:
//#include <RaspiCamCV.h>

// TCP/IP Port to use: 50058

// ON ODROID:
VideoCapture		capture;
// ON RPi:
//RaspiCamCvCapture*	capture;
// ON Laptop:
//CvCapture*			capture;

Mat			view;

string GetDevice()
{
	// ON ODROID:
	string			device	= "ODROID";
	// ON RPi:
	//string			device	= "RPi";
	// ON Laptop:
	//string			device	= "Laptop";

	return device;
}

// Load camera parameters: Camera Matrix and Distortion Coefficients (depending on whether it is executed on laptop or RPi):
void LoadCameraParams(string device, Mat& camera_matrix, Mat& dist_coeffs)
{
	if (device	== "ODROID")
	{
		// RPi's Camera Matrix and Distortion Coefficients (results of performing camera calibration):
		camera_matrix	= (Mat_<double>(3,3) <<	7.6852285882516151e+02,		0.0,						3.1950000000000000e+02,
												0.0,						7.6755736807396465e+02,		2.3950000000000000e+02,
												0.0,						0.0,						1.0);
		dist_coeffs		= (Mat_<double>(5,1) <<	1.6677488820001057e-01,
												-1.2429670699358248e+00,
												-9.6752060057404485e-03,
												1.1339215739698106e-02,
												-1.0188238805397445e+00);
		
	}
	else if (device	== "RPi")
	{
		// RPi's Camera Matrix and Distortion Coefficients (results of performing camera calibration):
		camera_matrix	= (Mat_<double>(3,3) <<	5.7829806132722638e+02,		0.0,						1.5950000000000000e+02,
												0.0,						5.7760879651866014e+02,		1.1950000000000000e+02,
												0.0,						0.0,						1.0);

		dist_coeffs		= (Mat_<double>(5,1) <<	-8.7907702260478282e-02,
												3.4231846884769883e+00,
												-3.3898540909388648e-03,
												4.2661597237650152e-03,
												-2.4240153052710252e+01);
	}
	else if (device == "Laptop")
	{
		// Giovanni's ThinkPad T430s Laptop Webcam's Camera Matrix and Distortion Coefficients (results of performing camera calibration):
		camera_matrix	= (Mat_<double>(3,3) <<	6.4607779344117296e+002,	0.0,						3.1950000000000000e+002,
												0.0,						6.4611590221166568e+002,	2.3950000000000000e+002,
												0.0,						0.0,						1.0);
		dist_coeffs		= (Mat_<double>(5,1) <<	-1.2568930877482700e-002,
												-6.5894418731935356e-002,
												-2.1086106689000052e-003,
												-1.1323933275534813e-003,
												-6.6066720498874881e-002);
	}
}

// Initiate camera capture and video-frame grabbing:
void GetCamera()
{
	// ON ODROID:
	capture.open(0);
	// ON RPi:
	//capture = raspiCamCvCreateCameraCapture(0);	// Index doesn't really matter
	// ON Laptop:
	//capture	= cvCaptureFromCAM(0);
}

// Acquire input image:
Mat GetImage()
{
	// ON ODROID:
	capture >> view;
	return	view;
	// ON RPi:
	//return(Mat(raspiCamCvQueryFrame(capture)));
	// ON Laptop:
	//return(Mat(cvQueryFrame(capture)));
}

// Release camera capture and stop video-frame grabbing:
void ReleaseCamera()
{
	// ON ODROID:
	capture.release();
	// ON RPi:
	//raspiCamCvReleaseCapture(&capture);
	// ON Laptop:
	//cvReleaseCapture(&capture);
}
