/*
该代码来自博客：https://blog.csdn.net/A_L_A_N/article/details/82855998
使用了Eigen 和 OpenCV3.4.5
*/

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/eigen>

using namespace std;

void ICP(const vector<cv::Point3f> &pts1, const vector<cv::Point3f> &pts2);

int main(void)
{
	Eigen::Vector3f p1_c1(0, 0, 20);
	Eigen::Vector3f p2_c1(2, 4, 30);
	Eigen::Vector3f p3_c1(5, 9, 40);
	Eigen::Vector3f p4_c1(6, 8, 25);

	// 绕y轴逆时针旋转30度(转yaw)的矩阵
	//Eigen::AngleAxisf rotation_vector( 30 * cv::CV_PI / 180, Eigen::Vector3f(0.f, 1.f, 0.f));
	Eigen::AngleAxisf rotation_vector(30 * CV_PI / 180, Eigen::Vector3f::UnitY());//绕y轴逆时针旋转30度(转yaw)
	Eigen::Matrix<float, 3, 3> R21 = rotation_vector.toRotationMatrix();
	cout << "R21: " << endl << R21 << endl;
	Eigen::Vector3f t21(5, 3, 1);


	Eigen::Vector3f p1_c2 = R21 * p1_c1 + t21;
	Eigen::Vector3f p2_c2 = R21 * p2_c1 + t21;
	Eigen::Vector3f p3_c2 = R21 * p3_c1 + t21;
	Eigen::Vector3f p4_c2 = R21 * p4_c1 + t21;

	vector<cv::Point3f> p_c1, p_c2;
	p_c1.push_back(cv::Point3f(p1_c1[0], p1_c1[1], p1_c1[2]));
	p_c1.push_back(cv::Point3f(p2_c1[0], p2_c1[1], p2_c1[2]));
	p_c1.push_back(cv::Point3f(p3_c1[0], p3_c1[1], p3_c1[2]));
	p_c1.push_back(cv::Point3f(p4_c1[0], p4_c1[1], p4_c1[2]));

	p_c2.push_back(cv::Point3f(p1_c2[0], p1_c2[1], p1_c2[2]));
	p_c2.push_back(cv::Point3f(p2_c2[0], p2_c2[1], p2_c2[2]));
	p_c2.push_back(cv::Point3f(p3_c2[0], p3_c2[1], p3_c2[2]));
	p_c2.push_back(cv::Point3f(p4_c2[0], p4_c2[1], p4_c2[2]));

	ICP(p_c1, p_c2);


	system("pause");
	return 0;
}


void ICP(const vector<cv::Point3f>& pts1, const vector<cv::Point3f>& pts2) {
	cv::Point3f p1, p2; // center of mass
	int N = pts1.size();
	for (int i = 0; i < N; ++i)
	{
		p1 += pts1[i];
		p2 += pts2[i];
	}

	// 计算出了两片点云的center of mass，但要求两片点云具有相同的点数
	p1 = cv::Point3f(cv::Vec3f(p1) / N);
	p2 = cv::Point3f(cv::Vec3f(p2) / N);		

	// remove the center
	vector<cv::Point3f> q1(N), q2(N);
	for (int i = 0; i < N; ++i)
	{
		q1[i] = pts1[i] - p1;
		q2[i] = pts2[i] - p2;
	}

	// compute q1*q2^T
	Eigen::Matrix3f W = Eigen::Matrix3f::Zero();
	cout << W << endl;
	for (int i = 0; i < N; ++i)
	{
		W += Eigen::Vector3f(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3f(q2[i].x, q2[i].y, q2[i].z).transpose();
		std::cout << W << endl;
		cout << "-------------" << endl;

	}

	// SVD on W
	Eigen::JacobiSVD<Eigen::Matrix3f> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3f U = svd.matrixU();
	Eigen::Matrix3f V = svd.matrixV();

	Eigen::Matrix3f R_12 = U * (V.transpose());
	Eigen::Vector3f t_12 = Eigen::Vector3f(p1.x, p1.y, p1.z) - R_12 * Eigen::Vector3f(p2.x, p2.y, p2.z);

	// Validation
	Eigen::AngleAxisf R_21;
	R_21.fromRotationMatrix(R_12.transpose());
	cout << "axi: " << R_21.axis().transpose() << endl;
	cout << "angle: " << R_21.angle() * 180 / CV_PI << endl;
	cout << "t: " << (-R_12.transpose()*t_12).transpose() << endl;


}