# vision_landing
Precision landing using visual targets.
This is a little project to achieve precision landing on drones using ArduCopter firmware, using vision alone.  Fiducial markers are printed and used as landing targets, and these targets provide orientation, location and distance information when combined with accurate size information of the markers and calibrated camera information.

Dependencies
--------------------
**OpenCV (>3.0 recommended)**  
If possible find packages for your OS for version 3.0 or above (or install from source, for the brave), or else install 2.x packages for your OS.  For debian/ubuntu which currently only has opencv2.4, you probably want to install at least these packages:  
 - libopencv-dev  
 - libopencv-calib3d-dev  
 - libopencv-highgui-dev  
 - libopencv-imgproc-dev 
Note that this was developed using OpenCV 3.2.  SolvePNP which is the underlying function for pose estimation that aruco uses is somewhat broken in 2.4 so aruco includes it's own routines, only using solvepnp in OpenCV >3.0.

**Aruco (>2.0)**
It is recommended to install 2.0.16 from https://github.com/fnoop/aruco as it has patched pkg-config files.  
Installation is straight forward:  
 ```
 git clone https://github.com/fnoop/aruco
 cd aruco
 cmake . && make && sudo make install
 ```
 
**Dronekit**
Install the latest dronekit:  
 ```
 sudo pip install dronekit
 ```

Installation
--------------------
First ensure the above dependencies are already installed.  

There are two main components:  
 - vision_landing, a python script
 - track_targets, a c++ program

track_targets must be compiled and installed into the main directory before vision_landing can be run.  vision_landing calls track_targets to do the actual target detection and vector calculations.
 ```
 git clone https://github.com/fnoop/vision_landing
 cd vision_landing/src
 cmake . && make && make install
 ```
You should now have a track_targets file in the same directory as the vision_landing script (the root of the vision_landing project).

Running
--------------------
To run this project, you MUST have accurate calibration data for your camera sensor/lens combination.  This can be quite challenging to accomplish, so sample calibration data for Odroid oCam 5cr and Raspberry Pi cameras (v1 only) are included in the calibration directory.  More will hopefully be added in the future.

There are three mandatory arguments:
 - connectionstring: This is the dronekit connection string, eg. /dev/ttyS0 (for serial connection to FC), tcp:localhost:5770 (tcp connection to mavproxy), udp:localhost:14560 (udp connection to mavproxy).
 - markersize: This is the size of the fiducial marker in meters.  So a typical april tag printed on A3 will be around 235cm squared, so the value here would be 0.235.
 - calibration: This is the file containing calibration data, eg. calibration/ocam5cr-calibration-640x480.yml

There are several optional arguments:
 - --input: This is the video stream 'pipeline' used to look for targets.  It defaults to /dev/video0 which is what most USB cameras show up as (/dev/video2 typically for odroid xu4 with hardware encoding activated).  Video files can be used for testing by just specifying the video file name here.
 - --output: This is the output 'pipeline' that can be used to save the video stream with AR data overlaid.  This can either be a video file name (which will be uncompressed and very large), or you can create gstreamer pipelines.  Example pipline for odroid xu4 with hardware encoding activated to stream h264 compressed video with AR markers in realtime would be:
  appsrc ! autovideoconvert ! v4l2video11h264enc ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.1.70 port=5000 sync=false
 - --markerid: Specify a marker ID to detect.  If specified, only a target matching this ID will be used for landing and any other target will be rejected.  If not specified, all targets in the specified or default dictionary will be detected and the closest will be chosen for landing.
 - --markerdict: Type of marker dictionary to use.  Only one dictionary can be detected at any one time.  Supported dictionaries are: ARUCO ARUCO_MIP_16h3 ARUCO_MIP_25h7 ARUCO_MIP_36h12 ARTOOLKITPLUS ARTOOLKITPLUSBCH TAG16h5 TAG25h7 TAG25h9 TAG36h11 TAG36h10
 - --simulator: If this flag is set, when the dronekit connection is made the script will wait for prearm checks to pass and then take off to preset altitude and then initiate landing.  This is typically used when connecting to SITL simulator to test precision landing.
 
**Examples**
```
./vision_landing /dev/ttyS0 0.235 calibration/ocam5cr-calibration-640x480.yml
./vision_landing --markerid 580 --markerdict TAG36h11 /dev/ttyS0 0.235 calibration/ocam5cr-calibration-640x480.yml
./vision_landing --simulator --input /dev/video2 --output "appsrc ! autovideoconvert ! v4l2video11h264enc ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.1.70 port=5000 sync=false" --markerdict TAG36h11 tcp:localhost:5777 0.235 calibration/ocam5cr-calibration-640x480.yml
./vision_landing --input /dev/video2 --output "appsrc ! autovideoconvert ! v4l2video11h264enc ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.1.70 port=5000 sync=false" --markerdict ARUCO_MIP_36h12 tcp:localhost:5777 0.235 calibration/ocam5cr-calibration-640x480.yml

```
