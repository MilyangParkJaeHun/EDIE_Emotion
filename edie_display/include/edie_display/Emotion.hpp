/*
 * Emotion.hpp
 *
 *  Author: Park Jaehun
 *  Refactoring: Park Jaehun , 2021.09.15
 */

#ifndef EMOTION_HPP
#define EMOTION_HPP
#include <boost/filesystem.hpp>
#include <cstring>
#include <cstdio>
#include <sstream>
#include <unistd.h>
#include <pwd.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "edie_display/Parser.hpp"
#include <ros/package.h>
#include <iostream>

enum class EmotionState
{
  BLINK,
  LAUGH,
  FLINCH,
  LOOK_AROUND,
  LOVE,
  BATTERYLOW,
  COUNT = 6
};

std::ostream& operator<<(std::ostream& os, EmotionState state);

class Emotion
{
  private:
    // Total image file count
    int size_;
    // Total emotion sequence count
    int sequence_size_;
    // Now image index
    int now_idx_;
    // Image file format
    std::string format_;
    // Image file directory path
    std::string img_path_;
    // Image file path
    std::string img_file_name_;
    // Name of emotion
    std::string emotion_name_;
    // Enum of emotion
    EmotionState emotion_;
    // Emotion sequence
    std::vector<int> sequence_;
    // Emotion image frame
    std::vector<cv::Mat> image_;

  public:
    /**
     * @brief Construct a new Emotion object
     * 
     * @param emotionId is unique number assigned to each emotion. 
     */
    Emotion(int emotionId);

    /**
     * @brief Destroy the Emotion
     * 
     */
    ~Emotion();

    /**
     * @brief Count the total number of .ext files in directory
     * 
     * @param directory where the file exists.
     * @param ext is file extension
     * @return int
     */
    int countFiles(std::string directory, std::string ext);

    /**
     * @brief Change Enum class to string
     * 
     * @param emotion is enum class represents emotions.
     * @return std::string 
     */
    std::string state2Str(EmotionState emotion);

    /**
     * @brief Assign an emotionId to the current Emotion instance
     * and set Emotion instance to the inital state.
     * 1. Get image files for making emotion represents.
     * 2. Get sequnence to run images in order.
     *
     * @param emotionId is unique number assigned to each emotion.
     */
    void set(int emotionId);

    /**
     * @brief Check the sequence is over.
     * 
     * @return true if the sequence is over.
     */
    bool isPlayDone();

    // Start current Emotion at first image.
    void start();

    //Add index to point to next image.
    void addIdx();

    // Get current now_idx_.
    int getIdx();

    // Get sequence length.
    int getSize();

    // Get emotion name as string
    std::string getName();

    // Get current image file name
    std::string getImgFileName();

    // Get image file name according to idx
    std::string getImgFileName(int idx);

    // Get current image
    cv::Mat getImage();

};

#endif
