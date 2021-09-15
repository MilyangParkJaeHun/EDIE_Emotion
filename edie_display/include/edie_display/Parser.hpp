/*
 * Parser.hpp
 *
 *  Author: Park Jaehun
 *  Refactoring: Park Jaehun , 2021.09.15
 */

#ifndef PARSER_HPP
#define PARSER_HPP
#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <sstream>
#include <pwd.h>
#include <iterator>
#include <string>
#include <vector>
#include <cstdlib>
#include <unistd.h>
#include <map>
#include <ros/package.h>

class Parser
{
  private:
    // json file path
    std::string json_file_path_;

  public:
    /**
     * @brief Construct a new Parser object
     * 
     */
    Parser();
    /**
     * @brief Destroy the Parser object
     * 
     */
    ~Parser();

    /**
     * @brief Get image sequences according to emotion name
     * 
     * @param emotion_name 
     * @return std::vector<int> is image sequences
     * ex) [0, 1, 2, 3, 2, 3, 2, 1, 0]
     */
    std::vector<int> getSequence(std::string emotion_name);

    /**
     * @brief Make image sequences using base contents and sequence of base contents
     * 
     * @param base_contents_map is base contents
     * @param sequence is seqeunce of base contents
     * @return std::vector<int>
     */
    std::vector<int> makeSequence(std::map<std::string, std::vector<int> > base_contents_map, Json::Value sequence);
     
    /**
     * @brief Get base contents from json data
     * json data format : {"a": "0", "b": "1~7"}
     * base contents  : {"a": [0], "b": [1, 2, 3, 4, 5, 6, 7]}
     * 
     * @param json_data is json format data
     * @return std::map<std::string, std::vector<int> >
     */
    std::map<std::string, std::vector<int> > parseBaseContents(Json::Value json_data);

    /**
     * @brief Parse the contents line
     * 
     * @param contents_str is contents line
     * ex) "1~7"
     * @return std::vector<int> 
     */
    std::vector<int> parseContentsLine(std::string contents_str);

    /**
     * @brief Split strings through delimiters
     * 
     * @param s is string to split
     * @param delimiter 
     * @return std::vector<std::string>
     */
    std::vector<std::string> strSplit(const std::string& s, char delimiter);

    /**
     * @brief Read json file containing sequence information and save it in json_contents.
     * 
     * @param json_contents is map format
     */
    void readJson(Json::Value &json_contents);

};

#endif
