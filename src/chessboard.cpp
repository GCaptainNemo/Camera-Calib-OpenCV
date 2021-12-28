#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include "chessboard.h"
// windows
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
   //define something for Windows (32-bit and 64-bit, this part is common)
#include <io.h>

#ifdef _WIN64
   //define something for Windows (64-bit only)
#else
   //define something for Windows (32-bit only)
#endif
#elif __linux__
	// linux
#include <sys/io.h>
#endif

void zhang_zhengyou_calib(const char * folder_address) 
{
	std::vector<std::vector<cv::Point2f>> img_points_seq;
		  
	std::vector<std::string> files_vec;
	getFileNames(folder_address, files_vec);
	std::cout << "files_vec.shape = " << files_vec.size() << std::endl;
	cv::Size image_size;
	cv::Size board_size(6, 8); // chessboard corner nums
	for (auto file = files_vec.begin(); file != files_vec.end(); ++file)
	{
		std::cout << *file << std::endl;
		cv::Mat origin_img = cv::imread(*file);
		cv::Mat gray_img;
		cv::cvtColor(origin_img, gray_img, cv::COLOR_RGB2GRAY);
		cv::Mat out_img;
			
		// 
		std::vector<cv::Point2f> img_points_buf;
		if (findChessboardCorners(origin_img, board_size, img_points_buf) == 0)
		{
	 		std::cout << "can not find chessboard corners!\n"; 
	 		exit(1);
		}
		else {
	 		std::cout << "success!" << std::endl;
	 		cv::Mat view_gray;
	 		cv::cvtColor(origin_img, view_gray, cv::COLOR_RGB2GRAY);
	 		cv::find4QuadCornerSubpix(view_gray, img_points_buf, cv::Size(11, 11)); 
	 		img_points_seq.push_back(img_points_buf);
	 		cv::drawChessboardCorners(view_gray, board_size, img_points_buf, true); 
	 		cv::namedWindow("Camera Calibration", cv::WINDOW_NORMAL);
	 		cv::imshow("Camera Calibration", view_gray);
	 		cv::waitKey(1000);
	 		image_size.width = view_gray.cols;
	 		image_size.height = view_gray.rows;
		}
			
	}
		
	std::cout << img_points_seq.size() << std::endl;
		
	cv::Size square_size(2.44, 2.44);  
	std::vector<std::vector<cv::Point3f>> object_points; 
	cv::Mat intrinsic_mat(3, 3, CV_32FC1, cv::Scalar::all(0)); 
	std::vector<int> point_counts;  
	cv::Mat distCoeffs(1, 5, CV_32FC1, cv::Scalar::all(0)); 
	std::vector<cv::Mat> tvecsMat; 
	std::vector<cv::Mat> rvecsMat; 
	int i, j, t;
	int img_count = img_points_seq.size();
	for (t = 0; t < img_count; t++)
	{
		std::vector<cv::Point3f> tempPointSet;
		for (i = 0; i < board_size.height; i++)
		{
	 		for (j = 0; j < board_size.width; j++)
	 		{
	 			cv::Point3f realPoint;
	 			realPoint.x = i * square_size.width;
	 			realPoint.y = j * square_size.height;
	 			realPoint.z = 0;
	 			tempPointSet.push_back(realPoint);
	 		}
		}
		object_points.push_back(tempPointSet);
	}

	for (i = 0; i < img_count; i++)
	{
		point_counts.push_back(board_size.width * board_size.height);
	}
	cv::calibrateCamera(object_points, img_points_seq, image_size, intrinsic_mat, distCoeffs, rvecsMat, tvecsMat, 0);
	double total_err = 0.0; 
	double err = 0.0; 
	std::vector<cv::Point2f> image_points2; 
	for (i = 0; i < img_count; i++)
	{
		std::vector<cv::Point3f> tempPointSet = object_points[i];
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], intrinsic_mat, distCoeffs, image_points2);
		std::vector<cv::Point2f> tempImagePoint = img_points_seq[i];
		cv::Mat tempImagePointMat(1, tempImagePoint.size(), CV_32FC2);
		cv::Mat image_points2Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
	 		image_points2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points2[j].x, image_points2[j].y);
	 		tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, cv::NORM_L2);
		total_err += err /= point_counts[i];
		std::cout << i + 1 << " image average error = " << err << "pixels" << std::endl;
	}	
	std::cout << "intrinsic_mat = " << intrinsic_mat << std::endl;
	std::cout << "distCoeffs = " << distCoeffs << std::endl;

	std::cout << "average_error = " << total_err / img_count << " pixels" << std::endl;
};

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
// linux cannot use _findnext, _findfirst
void getFileNames(std::string path, std::vector<std::string>& files)
{
	intptr_t hFile = 0;
	struct _finddata_t fileinfo;
	std::string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
	 	if ((fileinfo.attrib & _A_SUBDIR))
	 	{
	 		if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
	 			getFileNames(p.assign(path).append("\\").append(fileinfo.name), files);
	 	}
	 	else
	 	{
	 		files.push_back(p.assign(path).append("\\").append(fileinfo.name));
	 	}
	} while (_findnext(hFile, &fileinfo) == 0);
	_findclose(hFile);
}
}
#elif __linux__
// linux
#endif

int main() 
{
	zhang_zhengyou_calib("../imgs/chessboard/");
	return 0;
}
