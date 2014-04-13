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
	// Overall Transformation Vector:
	Mat		transfvec12;

	// Image Buffer:
	Mat		img;

	// Flags:
	bool	print			= atoi(argv[1]);
	bool	show_img		= atoi(argv[2]);

	// target_found = 1 if target pattern is found, otherwise target_found = -1:
	int		target_found;

	GetCamera();

	if (show_img)
	{
		// Create displaying window:
		namedWindow("Result", CV_WINDOW_NORMAL);
	}

	while (true)
	{
		target_found	= FindTarget(transfvec12, img, print, show_img);

		if (show_img)
		{
			imshow("Result", img);
		}

		// Wait for 10 miliseconds until user press some key:
		int iKey = waitKey(10);

		// If user press "ESC" key:
		if (iKey == 27)
		{
			//imwrite("Result.jpg", img);
			break;	// Break the infinite loop...
		}
	}

	if (show_img)
	{
		// Close displaying windows:
		cvDestroyWindow("Result");
	}

	ReleaseCamera();

	return 0;
}