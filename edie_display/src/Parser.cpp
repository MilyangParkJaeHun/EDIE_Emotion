/*
 * Parser.cpp
 *
 *  Author: Park Jaehun
 *  Refactoring: Park Jaehun , 2021.09.15
 */

#include "edie_display/Parser.hpp"

Parser::Parser()
{
  // Get json file path
  json_file_path_ = ros::package::getPath("edie_display") + "/config/emotion_sequence.json";
}

Parser::~Parser(){}

std::vector<int> Parser::makeSequence(std::map<std::string, std::vector<int> > base_contents_map, Json::Value sequence)
{
  std::vector<int> emotion_sequence;

  std::vector<std::string> sequence_id = sequence.getMemberNames();

  // Read sequence
  for(int i = 0; i < sequence.size(); i++)
  {
    std::string id = sequence_id[i];
    std::string contents = sequence[id]["contents"].asString();
    int count = sequence[id]["count"].asInt();
    // Set how many repeat one image
    int repeat = 1;
    if(sequence[id].isMember("repeat"))
    {
      repeat = sequence[id]["repeat"].asInt();
    }

    // Repeat the same sequence count times
    for(int j = 0; j < count; j++)
    {
      bool reverse = false;
      for(int k = 0; k < contents.size(); k++)
      {
        std::string now_contents_id = "";
        now_contents_id += contents.at(k);
        // Check is reversed sequence
        if(now_contents_id != "R")
        {
          std::vector<int> base_contents = base_contents_map[now_contents_id];
          if(!reverse)
          {
            for(auto it = base_contents.begin(); it != base_contents.end(); ++it)
            {
              for(int l = 0; l < repeat; l++)
              {
                emotion_sequence.push_back(*it);
              }
            }
          }
          else
          {
            // Run in reverse order
            for(auto it = base_contents.end() - 1; it != base_contents.begin() - 1; --it)
            {
              for(int l = 0; l < repeat; l++)
              {
                emotion_sequence.push_back(*it);
              }
            }
            reverse = false;
          }
        }
        else
        {
          reverse = true;
        }
      }
    }
  }
  return emotion_sequence;
}

std::vector<int> Parser::parseContentsLine(std::string contents_line)
{
  std::vector<int> contents;

  std::vector<std::string> comma_split_contents, wave_split_contents;

  // Split strings through ','
  comma_split_contents = strSplit(contents_line, ',');

  for(int i = 0; i < comma_split_contents.size(); i++)
  {
    std::string now = comma_split_contents[i];

    // Split strings through '~'
    wave_split_contents = strSplit(now, '~');

    // if contents including '~'
    if(wave_split_contents.size() >= 2)
    {
      int start = std::stoi(wave_split_contents[0]);
      int end = std::stoi(wave_split_contents[1]);

      for(int j = start; j <= end; j++)
      {
        contents.push_back(j);
      }
    }
    else
    {
      int num = std::stoi(wave_split_contents[0]);
      contents.push_back(num);
    }
  }

  return contents;
}

std::map<std::string, std::vector<int> > Parser::parseBaseContents(Json::Value json_data)
{
  std::map<std::string, std::vector<int> > base_contents;
  std::vector<std::string> id_list = json_data.getMemberNames();

  for(int i = 0; i < json_data.size(); i++) {
    std::string id = id_list[i];

    // parse line by line
    std::vector<int> contents = parseContentsLine(json_data[id].asString());

    base_contents.insert(std::make_pair(id, contents));
  }

  return base_contents;
}

std::vector<int> Parser::getSequence(std::string emotion_name)
{
  Json::Value json_contents, base_contents, sequence;
  std::map<std::string, std::vector<int> > base_contents_map;

  // read sequence info from json file.
  readJson(json_contents);

  base_contents = json_contents[emotion_name]["base_contents"];
  sequence = json_contents[emotion_name]["sequence"];

  base_contents_map = parseBaseContents(base_contents);

  return makeSequence(base_contents_map, sequence);
}


//
std::vector<std::string> Parser::strSplit(const std::string& s, char delimiter)
{
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimiter))
   {   
      tokens.push_back(token);
   }   
   return tokens;
}

void Parser::readJson(Json::Value &json_contents)
{
  std::ifstream ifs(json_file_path_);
  Json::Reader reader;
  reader.parse(ifs, json_contents);

  return;
}