#ifndef _DEBUG_H
#define _DEBUG_H

#include <iostream>
#include <fstream>
#include <vector>
#include <jsoncpp/json/json.h>
#include "common.hpp"
#include "Point.hpp"
#include "Bayazit.hpp"
#include <typeinfo>
#include <dirent.h>
#include <algorithm>
#include <chrono>

Polygon extract_polygon(std::string mypath) {
  std::ifstream file(mypath, std::ios::binary);
  Polygon polygon;
  if (!file) {
    std::cerr << "Failed to open file" << std::endl;
    return polygon;
  }

  // Parse the JSON data
  Json::Value root;
  file >> root;

  // Process the JSON data
  std::vector<std::vector<double>> data;
  for (const auto& item : root) {
    std::vector<double> sublist;
    for (const auto& value : item) {
      sublist.push_back(value.asDouble());
    }
    data.push_back(sublist);
  }

  // Print the retrieved data
  int Nmax = 500;
  int index = 0;
  for (const auto& sublist : data) {
    // auto& pt = sublist;
    // std::cout << sublist[0] << std::endl;
    Point point = Point(sublist[0], sublist[1]);
    // for (const auto& value : sublist) {
    //     std::cout << value << " ";
    //     std::cout << "type : " << typeid(value).name()  << std::endl;
    // }
    // std::cout << std::endl;
    if (index <= Nmax) {
      polygon.push_back(point);
    }
    index++;
  }
  std::cout << "size of object : " << polygon.size() << std::endl;
  return polygon;
}

int main() {
  // std::string my_path = "/home/thomas_cbrs/Desktop/log_file_gazebo/debug/polygon.json";
  // std::string my_path_poly = "/home/thomas_cbrs/Desktop/log_file_gazebo/debug/holes_03.json";
  // std::string my_path_holes = "/home/thomas_cbrs/Desktop/log_file_gazebo/debug/holes_01.json";
  // Polygon polygon = extract_polygon(my_path_poly);

  std::string folderPath =
      "/home/thomas_cbrs/Desktop/log_file_gazebo/debug/expe_02/";  // Replace with your folder path
  std::vector<std::string> fileNames;

  DIR* dir;
  struct dirent* entry;

  // Open the directory
  dir = opendir(folderPath.c_str());
  if (dir != nullptr) {
    // Read directory entries
    while ((entry = readdir(dir)) != nullptr) {
      std::string fileName = entry->d_name;

      // Exclude current directory and parent directory entries
      if (fileName != "." && fileName != "..") {
        fileNames.push_back(fileName);
      }
    }

    // Close the directory
    closedir(dir);
  } else {
    std::cerr << "Failed to open directory" << std::endl;
    return 1;
  }
  // Print the file namesBayazit
  Bayazit algo = Bayazit();
  std::sort(fileNames.begin(), fileNames.end());
  for (const auto& fileName : fileNames) {
    // std::cout << "Processing : " << fileName << std::endl;
    Polygon polygon = extract_polygon(folderPath + fileName);
    auto startTime = std::chrono::high_resolution_clock::now();

    auto res = algo.decomposePolyWrapper(polygon);

    // Capture the end time
    auto endTime = std::chrono::high_resolution_clock::now();

    // Calculate the duration in milliseconds
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();

    std::cout << "Time taken by function : " << duration << " us" << std::endl;
  }

  return 0;
}

#endif
