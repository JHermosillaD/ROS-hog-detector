# HOG pedestrian detector
![Static Badge](https://img.shields.io/badge/ros%20-%20noetic%20-blue) ![GitHub repo size](https://img.shields.io/github/repo-size/jhermosillad/hog)
 ![GitHub top language](https://img.shields.io/github/languages/top/jhermosillad/hog) 

ROS package for pedestrian detection using Histograms of Oriented Gradients (HOG). The package subscribes to an image and uses OpenCV's trained HOG detector. The image containing the bounding box and its coordinates are published in real time through different topics.
## Set-up
### Requirements
- [x] ROS melodic or higher
- [x] OpenCV
### Installation
Clone the repository to the workspace source path.
```
user@hostname:~/workspace/src$ git clone https://github.com/JHermosillaD/hog.git
```
Compile the package.
```
user@hostname:~/workspace$ catkin_make
```
## Usage
Edit file `launch/body.launch` by the appropriate value of parameter `/camera_topic` according to the name of the used image topic :bangbang:

Run the launcher.
```
user@hostname:~/workspace$ roslaunch hog body.launch
```
## Visualization
The image containing the bounding box can be displayed in Rviz, the coordinates through topic `/hog/bounding_box`.

<img width="605" height="360" src="/hog.png">
