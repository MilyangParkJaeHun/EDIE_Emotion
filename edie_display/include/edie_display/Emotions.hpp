/*
 * Emotions.cpp
 *
 *  Author: Park Jaehun
 *  Refactoring: Park Jaehun , 2021.09.15
 */

#ifndef EMOTIONS_HPP
#define EMOTIONS_HPP
#include "edie_display/Emotion.hpp"
#include <cstdio>
#include <vector>
#include <cstring>

class Emotions
{
  private:
    // list containing emotion instances
    std::vector<Emotion> emotion_list_;
    // current emotion id
    int now_id_;
    // total number of emotions
    int total_emotion_count_;

  public:
    /**
     * @brief Construct a new Emotions object.
     * 
     */
    Emotions();

    /**
     * @brief Destroy the Emotions object.
     * 
     */
    ~Emotions();

    /**
     * @brief Change currnet emotion.
     * 
     * @param id Unique id that represents an emotion.
     * @return true if successfully change emotion id.
     */
    bool change(int id);

    /**
     * @brief Update current frame index.
     * 
     */
    void update();

    /**
     * @brief Check emotion play done.
     * 
     * @return true if the current emotion has ended.
     */
    bool isDone();

    /**
     * @brief Get the File Name object of current emotion.
     * 
     * @return std::string
     */
    std::string getFileName();

    /**
     * @brief Get the Image object of current emotion.
     * 
     * @return cv::Mat 
     */
    cv::Mat getImage();

    // Get current emotion name
    std::string testGet(int id);
};

#endif
