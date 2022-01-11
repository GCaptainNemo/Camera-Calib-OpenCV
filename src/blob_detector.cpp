#include <opencv2/calib3d.hpp>
#include "circle_board.h"
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

// test different blob detectors parameters
void test_blob_detectors(const char *img_dir)
{
	SimpleBlobDetector::Params params;
	/*params.minThreshold = 0;
	params.maxThreshold = 255;*/
	params.maxArea = 10e4;
	params.minArea = 10;
	params.filterByArea = true;

	/*params.filterByCircularity = true;
	params.minCircularity = 0.1;*/
	params.filterByColor = true;
	params.blobColor = 255; // 255 choose bright blob, 0 choose dark blob.
	params.filterByConvexity = true;
	params.minConvexity = 0.9;
	/*params.minDistBetweenBlobs = 5;
	params.filterByInertia = false;
	params.minInertiaRatio = 0.5;*/
	Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
	Mat img = imread(img_dir, 0);
	
	// //////////////////////////////////////////////////////
	// detect by blob detectors
	// //////////////////////////////////////////////////////
	std::vector<cv::KeyPoint> keypoints;
	blobDetector->detect(img, keypoints, cv::Mat());
	cv::Mat im_with_keypoints;
	cv::drawKeypoints(img, keypoints, im_with_keypoints);
	//cv::drawKeypoints(img, keypoints, im_with_keypoints, (0, 0, 255),
		//DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	cv::namedWindow("im_with_keypoints", cv::WINDOW_NORMAL);
	cv::imshow("im_with_keypoints", im_with_keypoints);
	std::cout << "keypoints size = " << keypoints.size() << std::endl;
	cv::waitKey(0);

	std::vector<Point2f> centers;
	Size patternSize(8, 6);

	bool found = findCirclesGrid(img, patternSize, centers, CALIB_CB_SYMMETRIC_GRID | CALIB_CB_CLUSTERING, blobDetector);
	if (found) { std::cout << "found !!!!" << std::endl; }
	else { std::cout << "not found !!!!" << std::endl; }

	Mat cimg;
	cvtColor(img, cimg, COLOR_GRAY2BGR);
	drawChessboardCorners(cimg, patternSize, centers, found);

	double sf = 960. / MAX(img.rows, img.cols);
	
	#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
        // OPENCV 4.5.0
		resize(img, img, Size(), sf, sf, INTER_LINEAR_EXACT);
		resize(cimg, cimg, Size(), sf, sf, INTER_LINEAR_EXACT);
	#elif __linux__
		resize(img, img, Size(), sf, sf, CV_INTER_LINEAR);
		resize(cimg, cimg, Size(), sf, sf, CV_INTER_LINEAR);
	#endif
	cv::namedWindow("origin img", cv::WINDOW_NORMAL);
	cv::namedWindow("corners", cv::WINDOW_NORMAL);

	imshow("origin img", img);
	imshow("corners", cimg);

	waitKey();

};

void get_blob_detectors(Ptr<FeatureDetector> *blobDetector)
{
	// Blob算子参数
	SimpleBlobDetector::Params params;
	/*params.minThreshold = 0;
	params.maxThreshold = 255;*/
	params.maxArea = 10e4;
	params.minArea = 10;
	params.filterByArea = true;

	/*params.filterByCircularity = true;
	params.minCircularity = 0.1;*/
	params.filterByColor = true;  
	params.blobColor = 255; // 255 choose bright blob, 0 choose dark blob.
	params.filterByConvexity = true;
	params.minConvexity = 0.9;
	params.filterByInertia = true;
	params.minInertiaRatio = 0.7; // line:0, circle:1
	/*params.minDistBetweenBlobs = 5;
	params.filterByInertia = false;
	params.minInertiaRatio = 0.5;*/
	*blobDetector = SimpleBlobDetector::create(params);	
};
