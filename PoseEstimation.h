#pragma once

#include <stdio.h>
#include <math.h>

#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace cv;
using namespace std;

// TCP/IP Port to use: 50058

// Functions and Procedures defined in CameraDriver.cpp:
string GetDevice();
void LoadCameraParams(string device, Mat& camera_matrix, Mat& dist_coeffs);
void GetCamera();
Mat GetImage();
void ReleaseCamera();

// Functions and Procedures defined in PoseEstimation.cpp:
void CalcBoardCornerPositions(vector<Point3f>& corner_object_points, Size board_size=Size(3, 4), float square_size=13.67/*(in millimeter)*/);
void UndistortImage(Mat& in_img, Mat camera_matrix, Mat dist_coeffs);
bool GetPose(Mat& undist_in_img, Mat& transfvec12, Mat& rvec, Mat& tvec, vector<Point3f> corner_object_points/*(3D object points)*/, Mat camera_matrix, Mat dist_coeffs, bool print=true, Size board_size=Size(3, 4));
void DrawObjCoordFrame(Mat& undist_in_img, Mat rvec, Mat tvec, Mat camera_matrix, Mat dist_coeffs);