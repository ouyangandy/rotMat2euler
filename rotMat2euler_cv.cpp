#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <fstream>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

// Calculates rotation matrix given euler angles.
Mat eulerAnglesToRotationMatrix(Vec3f &theta)
{
	// Calculate rotation about x axis
	Mat R_x = (Mat_<double>(3, 3) <<
		1, 0, 0,
		0, cos(theta[0]), -sin(theta[0]),
		0, sin(theta[0]), cos(theta[0])
		);
	// Calculate rotation about y axis
	Mat R_y = (Mat_<double>(3, 3) <<
		cos(theta[1]), 0, sin(theta[1]),
		0, 1, 0,
		-sin(theta[1]), 0, cos(theta[1])
		);
	// Calculate rotation about z axis
	Mat R_z = (Mat_<double>(3, 3) <<
		cos(theta[2]), -sin(theta[2]), 0,
		sin(theta[2]), cos(theta[2]), 0,
		0, 0, 1
		);
	// Combined rotation matrix
	Mat R = R_z * R_y * R_x;
	return R;
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
	Mat Rt;
	transpose(R, Rt);
	Mat shouldBeIdentity = Rt * R;
	Mat I = Mat::eye(3, 3, shouldBeIdentity.type());
	return  norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{
	assert(isRotationMatrix(R));
	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
	bool singular = sy < 1e-6; // If
	double x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else
	{
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
#if 1
	x = x*180.0f / 3.141592653589793f;
	y = y*180.0f / 3.141592653589793f;
	z = z*180.0f / 3.141592653589793f;
#endif
	return Vec3f(x, y, z);
}

int main()
{
	Vec3f eulerAngles;

	//int i;
	////double r_vec[3]={-2.100418,-2.167796,0.273330};[[0.78520514][0.0233998 ][0.00969251
	//double r_vec[3] = { 2.174810320117,-0.004623494847,0.010459004693 };//eulerAngles[45,1,1]
	//double R_matrix[9];
	////CvMat *pr_vec;
	////CvMat *pR_matrix;
	//CvMat *pr_vec = cvCreateMat(1, 3, CV_64FC1);
	//CvMat *pR_matrix = cvCreateMat(3, 3, CV_64FC1);
	//cvInitMatHeader(pr_vec, 1, 3, CV_64FC1, r_vec, CV_AUTOSTEP);
	//cvInitMatHeader(pR_matrix, 3, 3, CV_64FC1, R_matrix, CV_AUTOSTEP);
	//cvRodrigues2(pr_vec, pR_matrix, 0);
	//Mat rotation_matrix(pR_matrix->rows, pR_matrix->cols, pR_matrix->type, pR_matrix->data.fl);
	//eulerAngles = rotationMatrixToEulerAngles(rotation_matrix);
	//cout << "pR_matrix = " << endl;
	//cout << rotation_matrix << endl;
	//cout << "eulerAngles = " << endl;
	//cout << eulerAngles << endl;

	Mat_<double> r_l = (Mat_<double>(3, 1) << -0.002865326065, -2.841691129650, 1.256884419023);//后摄像机的旋转向量
	Mat R_L;
	Rodrigues(r_l, R_L);
	eulerAngles = rotationMatrixToEulerAngles(R_L);
	cout << "R_L = " << endl;
	cout << R_L << endl;
	cout << "eulerAngles = " << endl;
	cout << eulerAngles << endl;

	//cvRodrigues2(&pr_vec,&pR_matrix,0);
	return 0;
}