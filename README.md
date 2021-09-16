# EDIE_Emotion
ROS package for robot emotional expression.
<p align="center">
<img src=https://user-images.githubusercontent.com/22341340/133569386-c0d83cad-8cb0-4892-a24c-1ce37ea52cc2.gif width="200" height="100"> <img src=https://user-images.githubusercontent.com/22341340/133569452-59d3c052-06aa-4f5a-b4ad-08cd84b577e3.gif width="200" height="100"></p>

<p align="center">
<img src=https://user-images.githubusercontent.com/22341340/133569477-224663f2-bd19-4482-ac18-f25b9109c0db.gif width="200" height="100">    <img src=https://user-images.githubusercontent.com/22341340/133568311-682e46be-aa86-46a7-97c1-c3d7e5aef5e6.gif width="200" height="100"></p>

<p align="center">
<img src=https://user-images.githubusercontent.com/22341340/133569342-779bbd5f-b7e7-4f4e-bd88-eaa9b31bb334.gif width="200" height="100"> <img src=https://user-images.githubusercontent.com/22341340/133569404-62f11904-bd23-40f3-a425-2fea6c475989.gif  width="200" height="100"></p>

By Park JaeHun
## Environments
- Ubuntu 18.04
- ROS melodic
- OpenCV 3.4.6

## Requirements
- jsoncpp
- OpenCV

## Build
```
$ catkin_make
```

## Run 
1. Run emotion handler node
```
$ rosrun edie_emotion_handler edie_emotion_node
```
2. Run display node
```
$ rosrun edie_display edie_display_node
```

## Test
```
$ rostopic pub /edie/emotion std_msgs/Int8 "data: 1"
```

## How to add new emotion expression
### 1. Create image folder
```
$ cd EDIE_Emotion/edie_display/image/Emotions
$ mkdir NEW_EMOTION
```
### 2. Add image to image folder
- name format : "emotions_${id}.png"
- ex) emotions_0.png

### 3. Edit emotion_sequence.json
```
$ vi EDIE_Emotion/edie_display/config/emotion_sequence.json
```
- ```base_contents``` : List of basic contents.
  - key : a~z
  - value : digit|,|~
    - ex) 1~4 : 1,2,3,4
    - ex) 0,2~4,1 : 0,2,3,4,1
  - ex) ```"base_contents": {"a": "0", "b": "1~2"}```
- ```sequence``` : Sequences created using ```base contents```
  - key : index increments from 0.
  - value :
    - key : "contents"
    - value : a~z|R
      - a~z values means base_content.
      - R means reverse order sequnce.
        ex) if ```b : 1,2``` then ```bRb : 1,2,2,1```
    - key : "count"
    - value : digit
      - Set the number of times the contents are repeated.
        ex) if ```contents: "bRb", "count": 2``` then ```1,2,2,1,1,2,2,1```
    - key : "repeat"
    - value : digit
      - Set the number of times the elements of the content are repeated
        ex) if ```contents: "bRb", "repeat": 2``` then ```1,1,2,2,2,2,1,1```
  - ex) add new emotion sequence
    - sequence info
      ```json
      "NEW_EMOTION": {
        "base_contents": {
          "a": "0",
          "b": "1~2"
        },
        "sequence": {
          "0": {
            "contents": "a",
            "count": 1
          },
          "1": {
            "contents": "bRb",
            "count": 1
          },
          "2": {
            "contents": "a",
            "count": 5
          }
        }
      }
      ```
    - sequence : 
      ```0,1,2,2,1,0,0,0,0,0```
### 4. Edit EDIE_Emotion/edie_display/src/Emotion.cpp
- Add NEW_EMOTION
  ```c++
  std::ostream& operator<<(std::ostream& os, EmotionState state)
  {
    switch(state)
    {
      case EmotionState::BLINK         : os << "BLINK"; break;
      case EmotionState::FLINCH        : os << "FLINCH"; break;
      case EmotionState::LAUGH         : os << "LAUGH"; break;
      case EmotionState::LOOK_AROUND   : os << "LOOK_AROUND"; break;
      case EmotionState::LOVE          : os << "LOVE"; break;
      case EmotionState::BATTERYLOW    : os << "BATTERYLOW"; break;
  +   case EmotionState::NEW_EMOTION   : os << "NEW_EMOTION"; break;
      default                          : os << "ERROR : " << static_cast<int>(state); break;
    }
    return os;
  }
  ```
### 5. Edit EDIE_Emotion/edie_display/include/edie_display/Emotion.hpp
- Add NEW_EMOTION & Increase COUNT
  ```c++
  enum class EmotionState
  {
    BLINK,
    LAUGH,
    FLINCH,
    LOOK_AROUND,
    LOVE,
    BATTERYLOW,
  + NEW_EMOTION,
  - COUNT = 6
  + COUNT = 7
  };
  ```
  
## Image COPYRIGHT
Aei ROBOT : http://arobot4all.com/
