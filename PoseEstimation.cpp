/***********************************************************************************************************************************

Copyright (c) 2014, Giovanni Sutanto
You can contact the author at <giovanni dot sutanto at gmail dot com>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of the author nor the names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR/CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


PoseEstimation.cpp

In the big picture, the program's goal is to perform 3D pose estimation of a plane with a chessboard pattern
imprinted on it, based on its image while captured by a single camera.

The 3D pose estimation steps are as follow:
(1) Detect the chessboard corners of the pattern found in the image (captured by the camera), using 
	OpenCV function findChessboardCorners
	(http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#bool findChessboardCorners(InputArray image, Size patternSize, OutputArray corners, int flags)).
(2) Compute the 3D coordinate of the real chessboard corners of the pattern with reference to the pattern's coordinate frame.
(3) Pass the matched points into "SolvePnPRansac" algorithm of OpenCV 
	(http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#solvepnpransac), 
	to compute the rotation vector (rvec) and translation vector (tvec) that will transform the 3D corner points on the chessboard pattern 
	into its projection on the image plane (to do this, the camera must have been calibrated first, 
	and the camera matrix as well as the distortion coefficients must be known).
(4)	Compute the matrix representation of the rvec, that is the rotation matrix (rmat), a 3X3 matrix, 
	by Rodrigues conversion (http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#rodrigues).
(5)	Combine the rmat (R) and tvec (t) into the overall solution: transfmat ([R|t]), a 3X4 matrix.
(6)	Transpose and reshape transfmat to become 12X1 matrix (the first 9 components are the transposed rmat, the last 3 components are the tvec), 
	ready for transmission using TCP/IP.
(7) Using the computed rvec & tvec, compute the projections of 
	the chessboard pattern's plane coordinate frame axes (x-y-z axes) on the image plane, using OpenCV function projectPoints
	(http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#void projectPoints(InputArray objectPoints, InputArray rvec, InputArray tvec, InputArray cameraMatrix, InputArray distCoeffs, OutputArray imagePoints, OutputArray jacobian, double aspectRatio)).
(8)	Draw & display the detected & matched chessboard corner points, and the pattern's coordinate frame axes on the output image.

************************************************************************************************************************************
*/

#include "PoseEstimation.h"

#define	PI	3.14159265358979323846

// TCP/IP Port to use: 50058

void ConvertIntoHomogeneous(Mat& matrix)
{
	// Get Homogeneous (4X4) Matrix from Non-Homogeneous (3X3) Matrix:
	hconcat(matrix, (Mat)(Mat_<double>(3,1) << 0.0, 0.0, 0.0), matrix);
	vconcat(matrix, (Mat)(Mat_<double>(1,4) << 0.0, 0.0, 0.0, 1.0), matrix);
}

void ConvertIntoNonHomogeneous(Mat& matrix)
{
	// Get Non-Homogeneous (3X3) Matrix from Homogeneous (4X4) Matrix:
	matrix	= matrix(Rect(0, 0, 3, 3));
}

Mat RotationMatrix(unsigned char axis, double angle)
{
	// Get Homogeneous (4X4) Rotation Matrix along specified axis, with specified angle:
	if (axis == 'x')
	{
		return	(Mat_<double>(4,4) <<	1.0,			0.0,			0.0,			0.0,
										0.0,			cos(angle),		-sin(angle),	0.0,
										0.0,			sin(angle),		cos(angle),		0.0,
										0.0,			0.0,			0.0,			1.0);
	}
	else if (axis == 'y')
	{
		return	(Mat_<double>(4,4) <<	cos(angle),		0.0,			sin(angle),		0.0,
										0.0,			1.0,			0.0,			0.0,
										-sin(angle),	0.0,			cos(angle),		0.0,
										0.0,			0.0,			0.0,			1.0);
	}
	else if (axis == 'z')
	{
		return	(Mat_<double>(4,4) <<	cos(angle),		-sin(angle),	0.0,			0.0,
										sin(angle),		cos(angle),		0.0,			0.0,
										0.0,			0.0,			1.0,			0.0,
										0.0,			0.0,			0.0,			1.0);
	}
}

Mat TranslationMatrix(double tx, double ty, double tz)
{
	// Get Homogeneous (4X4) Translation Matrix, with specified tx (translation along x-axis),
	// ty (translation along y-axis), and tz (translation along z-axis):
	return	(Mat_<double>(4,4) <<	1.0,			0.0,			0.0,			tx,
									0.0,			1.0,			0.0,			ty,
									0.0,			0.0,			1.0,			tz,
									0.0,			0.0,			0.0,			1.0);
}

void CalcBoardCornerPositions(vector<Point3f>& corner_object_points, Size board_size/*=Size(3, 4)*/, float square_size/*=13.67(in millimeter)*/)
{
    corner_object_points.clear();

    for(int y = 0; y < board_size.height; ++y)
	{
		for(int x = 0; x < board_size.width; ++x)
		{
			corner_object_points.push_back(Point3f(float(x*square_size), float(-y*square_size), 0));
		}
	}
}

void UndistortImage(Mat& in_img, Mat camera_matrix, Mat dist_coeffs)
{
	// Undistort input image:
	Mat temp		= in_img.clone();
	undistort(temp, in_img, camera_matrix, dist_coeffs);
}

bool GetPose(Mat& undist_in_img, Mat& transfvec12, Mat& rvec, Mat& tvec, vector<Point3f> corner_object_points/*(3D object points)*/, Mat camera_matrix, Mat dist_coeffs, bool print/*=true*/, Size board_size/*=Size(3, 4)*/)
{
	// Let's say we have 3D object point P, which is seen as 2D image point p on the image plane;
	// the point p is related to point P by applying a rotation matrix R and a translation vector t to P, or mathematically:
	// p = [R|t] * P
	// See the theoretical explanation here: http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
	// Points:
	vector<Point2f>	corner_image_points;			// 2D image points (on image plane)

	bool			found;
	
	// Rotation Matrix:
	Mat				rmat;
	
	// Overall Transformation Matrices:
	Mat				transfmatCam;					// Overall Transformation Matrix (3X4) in reference to Camera Coordinate System
	Mat				transfmatCamHomogeneous;		// Overall Homogeneous Transformation Matrix (4X4) in reference to Camera Coordinate System
	Mat				transfmatSBHomogeneous;			// Overall Homogeneous Transformation Matrix (4X4) in reference to SuperBot's End-Effector Coordinate System
	Mat				transfmatSB;					// Overall Transformation Matrix (3X4) in reference to SuperBot's End-Effector Coordinate System

	// Images:
	Mat				undist_in_img_gray;
	
	found = findChessboardCorners(undist_in_img, board_size, corner_image_points, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
	if (found)		// If done with success,
	{
		// improve the found corners' coordinate accuracy for chessboard:
		cvtColor(undist_in_img, undist_in_img_gray, CV_BGR2GRAY);
		cornerSubPix(undist_in_img_gray, corner_image_points, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

		// Draw the corners:
		drawChessboardCorners(undist_in_img, board_size, Mat(corner_image_points), found);

		// Compute Rotation Vector (rvec) and Translation Vector (tvec) using solvePnPRansac algorithm:
		solvePnPRansac(corner_object_points, corner_image_points, camera_matrix, dist_coeffs, rvec, tvec);
		// Compute Rotation Matrix (rmat, size 3-by-3 matrix) from Rotation Vector (rvec, size 3-by-1 matrix) using Rodrigues transform
		// (http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#rodrigues):
		Rodrigues(rvec, rmat);
		// Rotate Target Chessboard Coordinate System 180 degrees along x-axis (Roll-direction)
		// (to align Target Chessboard Coordinate System with Camera Coordinate System):
		ConvertIntoHomogeneous(rmat);
		rmat					= rmat * RotationMatrix('x', PI);
		ConvertIntoNonHomogeneous(rmat);
		// Update the Rotation Vector (rvec) after the change above:
		Rodrigues(rmat, rvec);
		if (print)
		{
			cout<<"rvec = "<<rvec<<endl;
			cout<<"rmat = "<<rmat<<endl;
			cout<<"tvec = "<<tvec<<endl;
		}
		// Combine Rotation Matrix (rmat) and Translation Vector (tvec) into the overall transformation matrix
		// in reference to Camera coordinate system (transfmatCam, size 3-by-4 matrix), that is [rmat|tvec]:
		hconcat(rmat, tvec, transfmatCam);
		// Convert transfmatCam into transfmatSB (overall transformation matrix in reference 
		// to SuperBot's End-Effector coordinate system):
		Mat temp				= (Mat_<double>(1,4) <<	0.0, 0.0, 0.0, 1.0);
		vconcat(transfmatCam, temp, transfmatCamHomogeneous);
		Mat Rx					= RotationMatrix('x', -(PI/2));
		Mat Ry					= RotationMatrix('y', (PI/2));
		// Camera Offset from SuperBot's End-Effector coordinate system's point of origin:
		Mat T1					= TranslationMatrix(40.0, 70.0, 0.0);
		transfmatSBHomogeneous	= T1 * Ry * Rx * transfmatCamHomogeneous;
		transfmatSB				= transfmatSBHomogeneous(Rect(0, 0, 4, 3));
		//cout<<"transfmatSB = "<<transfmatSB<<endl;
		// Final transfmat re-shaping to satisfy requirements of manipulator program:
		transpose(transfmatSB, transfvec12);
		transfvec12				= transfvec12.reshape(12, 1);
	}
	else		// If NOT found,
	{
		// return extremely huge transfvec12 (which means error/failure):
		transfvec12		= (Mat_<double>(12,1) <<	10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20);
	}
	
	if ((found) && (print))
	{
		cout<<"transfvec12 = "<<transfvec12<<endl;
		cout<<endl;
	}
	
	return found;
}

void DrawObjCoordFrame(Mat& undist_in_img, Mat rvec, Mat tvec, Mat camera_matrix, Mat dist_coeffs)
{
	vector<Point3f>	coord_frame_object_points;		// 3D coordinate frame (origin, x-axis pointer, y-axis pointer, z-axis pointer) points
	vector<Point2f>	coord_frame_image_points;		// 2D image points (on image plane)

	// 3D object point coordinates of the axes' pointer of the target object plane (imprinted with the chessboard pattern)'s coordinate frame:
	Point3f			OBJ_COORD_ORIGIN(0.0f, 0.0f, 0.0f),
					OBJ_COORD_X(20.0f, 0.0f, 0.0f),
					OBJ_COORD_Y(0.0f, 20.0f, 0.0f),
					OBJ_COORD_Z(0.0f, 0.0f, 20.0f);
	
	// Push in 3D descriptor points of target plane (object)'s coordinate frame:
	coord_frame_object_points.push_back(OBJ_COORD_ORIGIN);
	coord_frame_object_points.push_back(OBJ_COORD_X);
	coord_frame_object_points.push_back(OBJ_COORD_Y);
	coord_frame_object_points.push_back(OBJ_COORD_Z);

	//cout<<"rmat = "<<rmat<<endl;
	//cout<<"rvec = "<<rvec<<endl;
	//cout<<"tvec = "<<tvec<<endl;
	
	// Project the 3D descriptor points of target plane's coordinate frame into image plane using computed rvec and tvec:
	projectPoints(coord_frame_object_points, rvec, tvec, camera_matrix, dist_coeffs, coord_frame_image_points);
	// Draw the projected X-axis of the target plane (object)'s coordinate frame on image plane, on the output image:
	line(undist_in_img, Point(coord_frame_image_points[0].x, coord_frame_image_points[0].y), Point(coord_frame_image_points[1].x, coord_frame_image_points[1].y), Scalar(0,0,255), 5, CV_AA);
	putText(undist_in_img, "X", Point(coord_frame_image_points[1].x, coord_frame_image_points[1].y), 1, 1, Scalar(0,0,255));
	// Draw the projected Y-axis of the target plane (object)'s coordinate frame on image plane, on the output image:
	line(undist_in_img, Point(coord_frame_image_points[0].x, coord_frame_image_points[0].y), Point(coord_frame_image_points[2].x, coord_frame_image_points[2].y), Scalar(0,255,0), 5, CV_AA);
	putText(undist_in_img, "Y", Point(coord_frame_image_points[2].x, coord_frame_image_points[2].y), 1, 1, Scalar(0,255,0));
	// Draw the projected Z-axis of the target plane (object)'s coordinate frame on image plane, on the output image:
	line(undist_in_img, Point(coord_frame_image_points[0].x, coord_frame_image_points[0].y), Point(coord_frame_image_points[3].x, coord_frame_image_points[3].y), Scalar(255,0,0), 5, CV_AA);
	putText(undist_in_img, "Z", Point(coord_frame_image_points[3].x, coord_frame_image_points[3].y), 1, 1, Scalar(255,0,0));
}

int FindTarget(Mat& transfvec12, Mat& img, bool print, bool show_img)
{
	// Let's say we have 3D object point P, which is seen as 2D image point p on the image plane;
	// the point p is related to point P by applying a rotation matrix R and a translation vector t to P.
	// See the theoretical explanation here: http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

	// Points:
	vector<Point3f>	corner_object_points;			// 3D object points
	vector<Point2f>	corner_image_points;			// 2D image points (on image plane)

	vector<Point3f>	coord_frame_object_points;		// 3D coordinate frame (origin, x-axis pointer, y-axis pointer, z-axis pointer) points
	vector<Point2f>	coord_frame_image_points;		// 2D image points (on image plane)

	Mat				my_camera_matrix, my_dist_coeffs;

	bool			found;

	// Translation and Rotation Vectors:
	Mat				tvec;							// translation vector
	Mat				rvec;							// rotation vector (convertible to rotation matrix via Rodrigues transformation)

	LoadCameraParams(GetDevice(), my_camera_matrix, my_dist_coeffs);

	CalcBoardCornerPositions(corner_object_points);

	img = GetImage();
	/*if (argc==4)
	{
		img = imread(argv[3], CV_LOAD_IMAGE_COLOR);
	}*/
	
	UndistortImage(img, my_camera_matrix, my_dist_coeffs);

	found			= GetPose(img, transfvec12, rvec, tvec, corner_object_points, my_camera_matrix, my_dist_coeffs, print);

	if (show_img)
	{
		if (found)		// If done with success,
		{
			DrawObjCoordFrame(img, rvec, tvec, my_camera_matrix, my_dist_coeffs);
		}
	}

	if (found)
	{
		return 1;
	}
	else
	{
		return -1;
	}
}
