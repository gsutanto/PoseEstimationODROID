/***********************************************************************************************************************************

PoseEstimation.cpp
Written by Giovanni Sutanto (gsutanto@usc.edu or giovanni.sutanto@gmail.com), February 2014

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

*************************************************************************************************************************************/

#include "PoseEstimation.h"

// TCP/IP Port to use: 50058

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
	
	// Rotation Matrix and Overall Transformation Matrix:
	Mat				rmat, transfmat;

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
		if (print)
		{
			cout<<"rvec = "<<rvec<<endl;
			cout<<"rmat = "<<rmat<<endl;
			cout<<"tvec = "<<tvec<<endl;
		}
		// Combine Rotation Matrix (rmat) and Translation Vector (tvec) into the overall transformation matrix
		// (transfmat, size 3-by-4 matrix), that is [rmat|tvec]:
		hconcat(rmat, tvec, transfmat);
		// Final transfmat re-shaping to satisfy requirements of manipulator program:
		transpose(transfmat, transfvec12);
		transfvec12	= transfvec12.reshape(12, 1);
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
	
	if (print)
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

	/*Mat				temp_vec1, temp_vec2, temp_vec3;
	Mat				rmat, transfmat;

	transfmat		= transfvec12.reshape(3, 4);

	// Here we want to get back Rotation Vector (rvec) and Translation Vector (tvec) from transfmat ([rmat|tmat]^T) Matrix,
	// for the purpose of drawing the coordinate frame of the chessboard pattern's plane. But something strange happened here.
	// The OpenCV function transpose(src, dst) and src.t() (both are for transposing a matrix/ src Mat) did not work properly
	// or as it should be (tested using OpenCV 2.3.1). So I used the following trick to come around this problem:
	transfmat.row(0).copyTo(temp_vec1);
	temp_vec1		= temp_vec1.reshape(1, 3);
	transfmat.row(1).copyTo(temp_vec2);
	temp_vec2		= temp_vec2.reshape(1, 3);
	transfmat.row(2).copyTo(temp_vec3);
	temp_vec3		= temp_vec3.reshape(1, 3);
	hconcat(temp_vec1, temp_vec2, rmat);
	hconcat(rmat, temp_vec3, rmat);
	// Compute Rotation Vector (rvec, size 3-by-1 matrix) from Rotation Matrix (rmat, size 3-by-3 matrix) using Rodrigues transform
	// (http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#rodrigues):
	Rodrigues(rmat, rvec);

	transfmat.row(3).copyTo(tvec);
	tvec			= tvec.reshape(1, 3);*/

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