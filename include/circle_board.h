#pragma once


void get_blob_detectors(cv::Ptr<cv::FeatureDetector> *blobDetector);


void get_blob_detectors(cv::Ptr<cv::FeatureDetector> *blobDetector, const bool &is_ir);


void test_blob_detectors(const char *img_dir);

