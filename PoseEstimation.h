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

************************************************************************************************************************************
*/

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
int FindTarget(Mat& transfvec12, Mat& img, bool print, bool show_img);
