#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include "chessboard.h"
#include <sstream>
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
#include <dirent.h>
#include <unistd.h>
#include <stdlib.h>
#endif

void zhang_zhengyou_calib(const char * input_address, const char *output_address) 
{
	std::vector<std::vector<cv::Point2f>> img_points_seq;
		  
	std::vector<std::string> files_vec;
	getFileNames(input_address, files_vec);
	std::cout << "files_vec.shape = " << files_vec.size() << std::endl;
	cv::Size image_size;
	cv::Size board_size(6, 8); // chessboard corner nums
	// for (auto file = files_vec.begin(); file != files_vec.end(); ++file)
	std::vector<std::string> valid_files_vec;
	for (auto file: files_vec)
	{	
		std::cout << file << std::endl;
		cv::Mat origin_img = cv::imread(file);
		if(origin_img.data == NULL)
		{
			std::cout << file << " isn't a readable image" << std::endl;
			continue;
		}	
		
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
		valid_files_vec.push_back(file);
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
	std::vector<double> error_lst;
	error_lst.reserve(img_count);   // set capacity
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
		err = norm(image_points2Mat, tempImagePointMat, cv::NORM_L2) / point_counts[i];
		error_lst.push_back(err);
		total_err += err;
		std::cout << i + 1 << " image average error = " << err << "pixels" << std::endl;
	}	
	std::cout << "intrinsic_mat = " << intrinsic_mat << std::endl;
	std::cout << "distCoeffs = " << distCoeffs << std::endl;
	std::cout << "average_error = " << total_err / img_count << " pixels" << std::endl;
	// ///////////////////////////////////////////////////////////////////////////////////
	// write output
	// ///////////////////////////////////////////////////////////////////////////////////
	std::string output_file = std::string(output_address) + std::string("/calib.yaml");
	cv::FileStorage fs( output_file.c_str(), cv::FileStorage::WRITE );  // FileStorage class used to output xml in OpenCV
	fs << "CameraMat" << intrinsic_mat;
    fs << "DistCoeff" << distCoeffs;
	fs << "ImageSize" << image_size;
    fs << "ReprojectionError" << total_err / img_count;
	fs << "DistModel" << "plumb_bob";
	fs << "ValidImageFiles" << valid_files_vec;
	fs << "EachReprojectionError" << error_lst;

};

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
// linux cannot use _findnext, _findfirst
// get all files in path(include subdir)
void getAllFileNames(std::string path, std::vector<std::string>& files)
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
					getAllFileNames(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

void getFileNames(input_address, files_vec)
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
				continue;
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
};


#elif __linux__
// get all files in path(include subdir)
void getAllFileNames(std::string basePath, std::vector<std::string>& files)
{
	DIR *dir;
    struct dirent *ptr;
    if ((dir=opendir(basePath.c_str())) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }
    while ((ptr=readdir(dir)) != NULL)
    {
    	if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
        	continue;
        else if(ptr->d_type == 8)    ///file
			files.push_back(basePath + "/" + std::string(ptr->d_name));
        else if(ptr->d_type == 10)    ///link file
			files.push_back(basePath + "/" + std::string(ptr->d_name));
        else if(ptr->d_type == 4)    ///dir
        {
           std::string sub_dir_path = basePath + "/" + std::string(ptr->d_name);
           getAllFileNames(sub_dir_path, files);
        }
    }
    closedir(dir);
    return;
}

void getFileNames(std::string basePath, std::vector<std::string>& files)
{
	DIR *dir;
    struct dirent *ptr;
    if ((dir=opendir(basePath.c_str())) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }
    while ((ptr=readdir(dir)) != NULL)
    {
    	if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
        	continue;
        else if(ptr->d_type == 8)    ///file
			files.push_back(basePath + "/" + std::string(ptr->d_name));
        else if(ptr->d_type == 10)    ///link file
			files.push_back(basePath + "/" + std::string(ptr->d_name));
        else if(ptr->d_type == 4)    ///dir
        {
           continue;
        }
    }
    closedir(dir);
    return;
}


// linux
#endif

int main(int argc, char *argv[]) 
{
	if (argc != 3){std::cout << "please enter input/output dir\n"; return -1;}
	// default "../imgs/chessboard/"
	const char *input_file_dir = argv[1];
	const char *output_file_dir = argv[2];
	zhang_zhengyou_calib(input_file_dir, output_file_dir);
	return 0;
}
