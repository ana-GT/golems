/**
 * @file GlobalRecognizer.cpp
 * @brief Implementation of global recognizer 
 */
#include "GlobalRecognizer.h"
#include "jsoncpp/json/json.h"
#include <stdio.h>
#include <iostream>
#include <fstream>

/**
 * @function GlobalRecognizer
 * @brief Constructor
 */
GlobalRecognizer::GlobalRecognizer() {

}


/**
 * @function GlobalRecognizer
 * @brief Destructor
 */
GlobalRecognizer::~GlobalRecognizer() {

}


/**
 * @function load 
 * @brief Load json file with info from objects to be recognized 
 */
bool GlobalRecognizer::load( std::string _obj_input ) {

  Json::Value root;
  Json::Reader reader;

  std::ifstream obj_string( _obj_input.c_str(), std::ifstream::binary );
  
  std::cout <<"Parsing file: "<< _obj_input << std::endl;
  bool parsingSuccessful = reader.parse( obj_string, root );
  if( !parsingSuccessful ) {
    std::cout << "Failed to parse file: \n "<< reader.getFormattedErrorMessages()<<std::endl;
    return false;
  } else{ printf("So far so good \n"); }

  int num_models = root.get("num_models", -1).asInt();
  std::cout << "Num models: " << num_models << std::endl;
  
  return true;  
}
