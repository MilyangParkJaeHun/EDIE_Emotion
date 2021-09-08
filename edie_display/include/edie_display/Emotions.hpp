/*
 * Emotions.cpp
 *
 *  Author: jh9277
 *  Refactoring: jh9277 , 2020.06.25
 */

#ifndef EMOTIONS_HPP
#define EMOTIONS_HPP
#include <edie_struct.h>
#include "edie_display/Emotion.hpp"
#include <cstdio>
#include <vector>
#include <cstring>

class Emotions
{
  private:
    // Emotion array
    std::vector<Emotion> contents_;
    // Now emotion id
    int now_id_;
    // Total emotion count
    int total_emotion_count_;
  public:
    //Constructor
    Emotions();
    //Destructor
    ~Emotions();

    bool Change(int id);
    void Play();
    bool Done();
    std::string GetFileName();
    std::string TestGet(int id);
    cv::Mat GetImage();
};

#endif
