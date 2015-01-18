/**
 * @file GlobalRecognizer.h
 * @brief Object recognition class using global descriptors
 */

#pragma once

#include <string>

/**
 * @class GlobalRecognizer
 */
class GlobalRecognizer {
  
 public:
  GlobalRecognizer();
  ~GlobalRecognizer();
  bool load( std::string _obj_input );
  
 private:
  
  
};
