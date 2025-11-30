// Copyright 2022 Chen Jun
#pragma once
// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "Armor.hpp"

namespace helios_cv
{

class NumberClassifier
{
public:
  explicit NumberClassifier(const std::string& model_path, const std::string& label_path, const double threshold,
                            const std::vector<std::string>& ignore_classes = {});

  // RAII Class need to delete copy constructor and copy assignment operator
  NumberClassifier(const NumberClassifier&) = delete;
  NumberClassifier& operator=(const NumberClassifier&) = delete;
  NumberClassifier(NumberClassifier&&) = delete;

  ~NumberClassifier() = default;

  void extractNumbers(const cv::Mat& src, std::vector<Armor>& armors);

  void classify(std::vector<Armor>& armors);

  double threshold;

private:
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;
  std::vector<std::string> ignore_classes_;
};

}  // namespace helios_cv