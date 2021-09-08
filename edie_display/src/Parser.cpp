/*
 * Parser.cpp
 *
 *  Author: jh9277
 *  Refactoring: jh9277 , 2020.06.29
 */

#include "edie_display/Parser.hpp"

Parser::Parser()
{
  // Get json file path
  json_file_path_ = ros::package::getPath("edie_display") + "/config/emotion_sequence.json";
}

Parser::~Parser(){}

// Split strings through delimiters
std::vector<std::string> Parser::StrSplit(const std::string& s, char delimiter)
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

// Convert string to int
int Parser::String2Int(std::string str)
{
  return std::stoi(str);
}

// Make emotion sequence with base contents
std::vector<int> Parser::MakeSequence(std::map<std::string, std::vector<int> > base_contents_map, Json::Value sequence)
{
  std::vector<int> emotion_sequence;

  std::vector<std::string> sequence_id = sequence.getMemberNames();

  // Read sequence
  for(int i = 0; i < sequence.size(); i++)
  {
    std::string id = sequence_id[i];
    std::string contents = sequence[id]["contents"].asString();
    int count = sequence[id]["count"].asInt();
    int repeat = 1;
    if(sequence[id].isMember("repeat"))
    {
      repeat = sequence[id]["repeat"].asInt();
    }

    // Repeat the same content count times
    for(int j = 0; j < count; j++)
    {
      bool reverse = false;
      // Read contents
      for(int k = 0; k < contents.size(); k++)
      {
        std::string now_contents_id = "";
        now_contents_id += contents.at(k);
        // If not reverse flag
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

// Parsing from contents string
std::vector<int> Parser::ParsingContents(std::string contents_str)
{
  std::vector<int> contents;

  std::vector<std::string> comma_split_contents, wave_split_contents;

  // Split strings through ','
  comma_split_contents = StrSplit(contents_str, ',');

  for(int i = 0; i < comma_split_contents.size(); i++)
  {
    std::string now = comma_split_contents[i];

    // Split strings through '~'
    wave_split_contents = StrSplit(now, '~');

    // if contents including '~'
    if(wave_split_contents.size() >= 2)
    {
      int start = String2Int(wave_split_contents[0]);
      int end = String2Int(wave_split_contents[1]);

      for(int j = start; j <= end; j++)
      {
        contents.push_back(j);
      }
    }
    else
    {
      int num = String2Int(wave_split_contents[0]);
      contents.push_back(num);
    }
  }

  return contents;
}

// Convert Json format to Map structure
std::map<std::string, std::vector<int> > Parser::Json2Map(Json::Value base_contents)
{
  std::map<std::string, std::vector<int> > base_contents_map;
  std::vector<int> contents;
  std::vector<std::string> contents_id = base_contents.getMemberNames();

  for(int i = 0; i < base_contents.size(); i++)
  {
    std::string id = contents_id[i];

    contents = ParsingContents(base_contents[id].asString());
    base_contents_map.insert(std::make_pair(id, contents));
  }

  return base_contents_map;
}

// Read Json file
void Parser::ReadJson(Json::Value &json_contents)
{
  std::ifstream ifs(json_file_path_);
  Json::Reader reader;
  reader.parse(ifs, json_contents);

  return;
}

// Get emotion sequence according to emotion name
std::vector<int> Parser::GetSequence(std::string emotion_name)
{
  std::vector<int> emotion_sequence;

  Json::Value json_contents, base_contents, sequence;
  std::map<std::string, std::vector<int> > base_contents_map;

  ReadJson(json_contents);

  base_contents = json_contents[emotion_name]["base_contents"];
  sequence = json_contents[emotion_name]["sequence"];

  base_contents_map = Json2Map(base_contents);

  emotion_sequence = MakeSequence(base_contents_map, sequence);

  return emotion_sequence;
}
