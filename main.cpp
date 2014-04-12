/***********************************************************************************************************************************

main.cpp
Written by Giovanni Sutanto (gsutanto@usc.edu or giovanni.sutanto@gmail.com), February 2014

Special Thanks to Emil Valkov for his work on RaspberryPiCamera-OpenCV Libraries inside RaspiCamCV.h
See Emil Valkov work at: https://robidouille.wordpress.com/2013/10/19/raspberry-pi-camera-with-opencv/

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
(5)	Combine the rmat (R) and tvec (t) into the overall solution: outmat ([R|t]), a 3X4 matrix.
(6)	Transpose and reshape outmat to become 12X1 matrix (the first 9 components are the transposed rmat, the last 3 components are the tvec), 
	ready for transmission using TCP/IP.
(7) Using the computed rvec & tvec, compute the projections of 
	the chessboard pattern's plane coordinate frame axes (x-y-z axes) on the image plane, using OpenCV function projectPoints
	(http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#void projectPoints(InputArray objectPoints, InputArray rvec, InputArray tvec, InputArray cameraMatrix, InputArray distCoeffs, OutputArray imagePoints, OutputArray jacobian, double aspectRatio)).
(8)	Draw & display the detected & matched chessboard corner points, and the pattern's coordinate frame axes on the output image.

*************************************************************************************************************************************/

#include "PoseEstimation.h"

// TCP/IP Port to use: 50058

int main(int argc, const char** argv)
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

	// Overall Transformation Vector:
	Mat				transfvec12;

	// Images:
	Mat				in_img;

	GetCamera();
	LoadCameraParams(GetDevice(), my_camera_matrix, my_dist_coeffs);

	CalcBoardCornerPositions(corner_object_points);

	if (atoi(argv[2]))
	{
		// Create displaying window:
		namedWindow("Result", CV_WINDOW_NORMAL);
	}

	while (true)
	{
		if (argc==4)
		{
			in_img = imread(argv[3], CV_LOAD_IMAGE_COLOR);
		}
		else
		{
			in_img = GetImage();
		}

		UndistortImage(in_img, my_camera_matrix, my_dist_coeffs);

		found				= GetPose(in_img, transfvec12, rvec, tvec, corner_object_points, my_camera_matrix, my_dist_coeffs, atoi(argv[1]));

		if (atoi(argv[2]))
		{
			if (found)		// If done with success,
			{
				DrawObjCoordFrame(in_img, rvec, tvec, my_camera_matrix, my_dist_coeffs);
			}

			imshow("Result", in_img);
		}

		// Wait for 10 miliseconds until user press some key:
		int iKey = waitKey(10);

		// If user press "ESC" key:
		if (iKey == 27)
		{
			//imwrite("Result.jpg", in_img);
			break;	// Break the infinite loop...
		}
	}

	if (atoi(argv[2]))
	{
		// Close displaying windows:
		cvDestroyWindow("Result");
	}

	ReleaseCamera();

	return 0;
}