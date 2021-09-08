/*
 * Parser.hpp
 *
 *  Author: jh9277
 *  Refactoring: jh9277 , 2020.06.29
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
    // Constructor
    Parser();
    // Desturctor
    ~Parser();

    std::vector<std::string> StrSplit(const std::string& s, char delimiter);
    int String2Int(std::string str);

    std::vector<int> MakeSequence(std::map<std::string, std::vector<int> > base_contents_map, Json::Value sequence);

    std::vector<int> ParsingContents(std::string contents_str);

    std::map<std::string, std::vector<int> > Json2Map(Json::Value base_contents);

    void ReadJson(Json::Value &json_contents);

    std::vector<int> GetSequence(std::string emotion_name);
};

#endif
