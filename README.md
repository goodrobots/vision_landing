# vision_landing
### Precision landing using visual targets.  
This is a project to achieve precision landing on drones using ArduCopter firmware, using vision alone.  Fiducial markers are printed and used as landing targets, and these targets provide orientation, location and distance information when combined with accurate size information of the markers and calibrated camera information.

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

Printing Targets/Markers
--------------------
The landing targets used in this project are called 'fiducial markers'.  Fiducial markers are encoded objects (think QR codes or bar codes) that are used by computer vision systems as reference points in a scene.  There are numerous different types of encodings, which are called 'dictionaries'.  The marker library used in this project (Aruco) has its own set of dictionaries, but it also supports other dictionary types such as AprilTags.  
In order to perform the recommended calibration (detailed below in the next section), it is necessary to print a specific A4 marker board:  
 https://github.com/fnoop/vision_landing/blob/master/calibration/aruco_calibration_board_a4.pdf  
It is important to print this on A4 size paper in exactly the size and proportions given in the PDF, as there are specific properties of the board that are expected by the calibration process.  
When it comes to the landing markers themselves, the size and format depends on the nature of the craft and landing situations.  The higher the craft normally flies or will start landing from, the larger the target needs to be in order to be detected.  However, larger targets are more difficult to produce, transport and store, and they become unreadable at lower altitudes as they exceed the Field of View of the camera.  If precision landing is acceptable from a lower altitude then a smaller target can be used.  Alternatively, multiple targets can be used with varying sizes, so a larger target can be used for high altitude lock-on, then the vision system can switch to smaller targets at lower altitude for better precision and to keep within the FoV.  
Aruco recommends the 'ARUCO_MIP_36h12' dictionary for the best compromise between marker size and robustness.  This is a 36bit (6x6) 250 element dictionary and the intermarker distance is smaller than other aruco dictionaries such as 16h3 (4x4), so it's possible after further testing that these dictionaries with smaller marker size are better for precision landing.  
<img src="https://github.com/fnoop/vision_landing/blob/master/markers/aruco_mip_36h12_00012.png" width="300">  
Single markers like above can be printed at any size.  Once printed, measure the size of the marker (black edge to black edge) and this is fed as the marker size parameter to vision_landing.  

**the following is not yet implemented, tracked in [https://github.com/fnoop/vision_landing/issues/16]**  
In order to address the problem of large markers exceeding the camera FoV at lower altitudes, multiple markers of differing sizes can be used.  As the craft descends in altitude locked on to a large marker, at some point a smaller marker will come into view and can be locked on to.  Marker boards can be used for increased robustness and accuracy.  An example of such a multiple marker board is included as an A1 PDF (vision_landing/markers/a1-landing.pdf):  
<img src="https://github.com/fnoop/vision_landing/blob/master/markers/a1-landing.png" width="300">  

Camera Calibration
--------------------
In order to perform any accurate Computer Vision work, you must first calibrate your camera.  Every camera and lens combination has different focal lengths, field of view, aperture, sensor size, optical center and lens distortions (eg. fisheye also known as positive radial distortion, or barrel distortion).  Even cameras or lenses of the same make/model will have small manufacturing tolerance/mistakes.  All of these must be known and compensated for.  
OpenCV (which is what the vision code in this project uses) uses optional 'Camera Matrix' and 'Distortion Coefficients'  matrices (collectively called intrinsics) which are used to 'undistort' the raw image for accurate further processing and can also be used to automatically calculate focal length and field of view, necessary in turn to calculate the target angular offsets for precision landing and pose estimation used for accurate distance measurements.  There are numerous calibration methods for opencv, but there is a simple interactive routine included in the Aruco software installed as a dependency, so that process is recommended and documented briefly here:  
<include link>



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
./vision_landing --input /dev/video2 --output /srv/maverick/data/video/landing.mpg --markerdict TAG36h11 tcp:localhost:5777 0.235 calibration/ocam5cr-calibration-640x480.yml

```
