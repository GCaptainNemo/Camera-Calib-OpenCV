#pragma once
#include <vector>
#include <string>

void zhang_zhengyou_calib(const char * folder_address);


// get all files in path(include subdir)
void getAllFileNames(std::string path, std::vector<std::string>& files);


// get all files in path(dont include subdir)
void getFileNames(std::string path, std::vector<std::string>& files);


