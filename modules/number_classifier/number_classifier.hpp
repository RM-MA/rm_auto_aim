#ifndef _NUMBER_CLASSIFIER_HPP_
#define _NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "../../utils/robot.hpp"

namespace Modules {

  using Armor = Robot::Armour;

  class NumberClassifier{

    public:
  NumberClassifier(
    const std::string & model_path, const std::string & label_path, const double threshold);

  void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors);

  void doClassify(std::vector<Armor> & armors);

  double threshold;
    private:
  cv::dnn::Net net_;
  std::vector<char> class_names_;
  
  };

}

#endif 