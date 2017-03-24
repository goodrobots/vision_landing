/**
track_targets
https://github.com/fnoop/vision_landing

This program uses opencv and aruco to detect fiducial markers in a stream.
It uses tracking across images in order to avoid ambiguity problem with a single marker.
For each target detected, it performs pose estimation and outputs the marker ID and translation vectors.
These vectors are used by vision_landing mavlink messages to enable precision landing by vision alone.

Compile with cmake: cmake . && make
 or manually: g++ src/track_targets.cpp -o track_targets -std=gnu++11 `pkg-config --cflags --libs aruco`

Run separately with: ./track_targets -d TAG36h11 /dev/video0 calibration.yml 0.235
./track_targets -w 1280 -g 720 -d TAG36h11 -o 'appsrc ! autovideoconvert ! v4l2video11h264enc extra-controls="encode,h264_level=10,h264_profile=4,frame_level_rate_control_enable=1,video_bitrate=2097152" ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.1.70 port=5000 sync=false' /dev/video2 calibration/ocam5cr-calibration-1280x720.yml 0.235
./track_targets -w 1280 -g 720 -d TAG36h11 -o /srv/maverick/data/videos/landing.avi /dev/video2 calibration/ocam5cr-calibration-1280x720.yml 0.235
./track_targets -b 0.8 -d TAG36h11 -o 'appsrc ! autovideoconvert ! v4l2video11h264enc extra-contros="encode,h264_level=10,h264_profile=4,frame_level_rate_control_enable=1,video_bitrate=2097152" ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.1.70 port=5000 sync=false' -i 1 -v /dev/video2 calibration/ocam5cr-calibration-640x480.yml 0.235
**/

#include <iostream>
#include <string>
#include <signal.h>
#include <poll.h>
#include <ctime>
#include <sys/select.h>
#include "args.hxx"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "aruco/aruco.h"

using namespace cv;
using namespace aruco;

// Setup sig handling
static volatile sig_atomic_t sigflag = 0;
static volatile sig_atomic_t stateflag = 0; // 0 = stopped, 1 = started
void handle_sig(int sig) {
    cout << "SIGNAL:" << sig << ":Received" << endl;
    sigflag = 1;
}
void handle_sigusr1(int sig) {
    cout << "SIGNAL:SIGUSR1:Received:" << sig << ":Switching on Vision Processing" << endl;
    stateflag = 1;
}
void handle_sigusr2(int sig) {
    cout << "SIGNAL:SIGUSR2:Received:" << sig << ":Switching off Vision Processing" << endl;
    stateflag = 0;
}

// Setup fps tracker
double CLOCK()
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC,  &t);
    return (t.tv_sec * 1000)+(t.tv_nsec*1e-6);
}
double _avgdur=0;
double _fpsstart=0;
double _avgfps=0;
double _fps1sec=0;
double avgdur(double newdur)
{
    _avgdur=0.98*_avgdur+0.02*newdur;
    return _avgdur;
}
double avgfps()
{
    if(CLOCK()-_fpsstart>1000)      
    {
        _fpsstart=CLOCK();
        _avgfps=0.7*_avgfps+0.3*_fps1sec;
        _fps1sec=0;
    }
    _fps1sec++;
    return _avgfps;
}

// Define function to draw AR landing marker
void drawARLandingCube(cv::Mat &Image, Marker &m, const CameraParameters &CP) {
    Mat objectPoints(8, 3, CV_32FC1);
    double halfSize = m.ssize / 2;

    objectPoints.at< float >(0, 0) = -halfSize;
    objectPoints.at< float >(0, 1) = -halfSize;
    objectPoints.at< float >(0, 2) = 0;
    objectPoints.at< float >(1, 0) = halfSize;
    objectPoints.at< float >(1, 1) = -halfSize;
    objectPoints.at< float >(1, 2) = 0;
    objectPoints.at< float >(2, 0) = halfSize;
    objectPoints.at< float >(2, 1) = halfSize;
    objectPoints.at< float >(2, 2) = 0;
    objectPoints.at< float >(3, 0) = -halfSize;
    objectPoints.at< float >(3, 1) = halfSize;
    objectPoints.at< float >(3, 2) = 0;

    objectPoints.at< float >(4, 0) = -halfSize;
    objectPoints.at< float >(4, 1) = -halfSize;
    objectPoints.at< float >(4, 2) = m.ssize;
    objectPoints.at< float >(5, 0) = halfSize;
    objectPoints.at< float >(5, 1) = -halfSize;
    objectPoints.at< float >(5, 2) = m.ssize;
    objectPoints.at< float >(6, 0) = halfSize;
    objectPoints.at< float >(6, 1) = halfSize;
    objectPoints.at< float >(6, 2) = m.ssize;
    objectPoints.at< float >(7, 0) = -halfSize;
    objectPoints.at< float >(7, 1) = halfSize;
    objectPoints.at< float >(7, 2) = m.ssize;

    vector< Point2f > imagePoints;
    cv::projectPoints(objectPoints, m.Rvec, m.Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);
    // draw lines of different colours
    for (int i = 0; i < 4; i++)
        cv::line(Image, imagePoints[i], imagePoints[(i + 1) % 4], Scalar(0, 255, 0, 255), 1, CV_AA);

    for (int i = 0; i < 4; i++)
        cv::line(Image, imagePoints[i + 4], imagePoints[4 + (i + 1) % 4], Scalar(0, 255, 0, 255), 1, CV_AA);

    for (int i = 0; i < 4; i++)
        cv::line(Image, imagePoints[i], imagePoints[i + 4], Scalar(0, 255, 0, 255), 1, CV_AA);
}

// Print the calculated distance at bottom of image
void drawVectors(Mat &in, Scalar color, int lineWidth, int vOffset, int MarkerId, double X, double Y, double Distance, double cX, double cY) {
    char cad[100];
    sprintf(cad, "ID:%i, Distance:%0.3fm, X:%0.3f, Y:%0.3f, cX:%0.3f, cY:%0.3f", MarkerId, Distance, X, Y, cX, cY);
    Point cent(10, vOffset);
    cv::putText(in, cad, cent, FONT_HERSHEY_SIMPLEX, std::max(0.5f,float(lineWidth)*0.3f), color, lineWidth);
}

// main..
int main(int argc, char** argv) {
    // Unbuffer stdout and stdin
    cout.setf(ios::unitbuf);
    ios_base::sync_with_stdio(false);

    // Setup arguments for parser
    args::ArgumentParser parser("Track fiducial markers and estimate pose, output translation vectors for vision_landing");
    args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
    args::Flag verbose(parser, "verbose", "Verbose", {'v', "verbose"});
    args::ValueFlag<int> markerid(parser, "markerid", "Marker ID", {'i', "id"});
    args::ValueFlag<string> dict(parser, "dict", "Marker Dictionary", {'d', "dict"});
    args::ValueFlag<string> output(parser, "output", "Output Stream", {'o', "output"});
    args::ValueFlag<int> width(parser, "width", "Video Input Resolution - Width", {'w', "width"});
    args::ValueFlag<int> height(parser, "height", "Video Input Resolution - Height", {'g', "height"});
    args::ValueFlag<int> fps(parser, "fps", "Video Output FPS - Kludge factor", {'f', "fps"});
    args::ValueFlag<double> brightness(parser, "brightness", "Camera Brightness/Gain", {'b', "brightness"});
    args::ValueFlag<string> fourcc(parser, "fourcc", "FourCC CoDec code", {'c', "fourcc"});
    args::Positional<string> input(parser, "input", "Input Stream");
    args::Positional<string> calibration(parser, "calibration", "Calibration Data");
    args::Positional<double> markersize(parser, "markersize", "Marker Size");
    
    // Parse arguments
    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch (args::Help)
    {
        std::cout << parser;
        return 0;
    }
    catch (args::ParseError e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }
    catch (args::ValidationError e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }
    
    if (!input || !calibration || !markersize) {
        std::cout << parser;
        return 1;
    }
    
    // Setup core objects
    aruco::CameraParameters CamParam;
    cv::Mat rawimage;
    VideoCapture vreader(args::get(input));

    // Bail if camera can't be opened
    if (!vreader.isOpened()) {
        cerr << "Error: Input stream can't be opened" << endl;
        return 1;
    }

    // Register signals
    signal(SIGINT, handle_sig);
    signal(SIGTERM, handle_sig);
    signal(SIGUSR1, handle_sigusr1);
    signal(SIGUSR2, handle_sigusr2);

    // If resolution is specified then use, otherwise use default
    int inputwidth = 640;
    int inputheight = 480;
    if (width)
        inputwidth = args::get(width);
    if (height)
        inputheight = args::get(height);
        
    // If fps is specified then use, otherwise use default
    // Note this doesn't seem to matter for network streaming, only when writing to file
    int inputfps = 30;
    if (fps)
        inputfps = args::get(fps);
        
    // If brightness is specified then use, otherwise use default
    double inputbrightness = 0.5;
    if (brightness)
        inputbrightness = args::get(brightness);

    // Set camera properties
    vreader.set(CAP_PROP_BRIGHTNESS, inputbrightness);
    vreader.set(CV_CAP_PROP_FRAME_WIDTH, inputwidth);
    vreader.set(CV_CAP_PROP_FRAME_HEIGHT, inputheight);
    vreader.set(CV_CAP_PROP_FPS, inputfps);

    // Read and parse camera calibration data
    CamParam.readFromXMLFile(args::get(calibration));
    if (!CamParam.isValid()) {
        cerr << "Calibration Parameters not valid" << endl;
        return -1;
    }

    // Take a single image and resize calibration parameters based on input stream dimensions
    vreader >> rawimage;
    CamParam.resize(rawimage.size());
    
    // Calculate the fov from the calibration intrinsics
    const double pi = std::atan(1)*4;
    const double fovx = 2 * atan(inputwidth / (2 * CamParam.CameraMatrix.at<float>(0,0))) * (180.0/pi); 
    const double fovy = 2 * atan(inputheight / (2 * CamParam.CameraMatrix.at<float>(1,1))) * (180.0/pi);
    cout << "info:FoVx~" << fovx << ":FoVy~" << fovy << ":vWidth~" << inputwidth << ":vHeight~" << inputheight << endl;

    // Create an output object, if output specified then setup the pipeline unless output is set to 'window'
    VideoWriter writer;
    if (output && args::get(output) != "window") {
        if (fourcc) {
            string _fourcc = args::get(fourcc);
            writer.open(args::get(output), CV_FOURCC(_fourcc[0], _fourcc[1], _fourcc[2], _fourcc[3]), inputfps, cv::Size(inputwidth, inputheight), true);
        } else {
            writer.open(args::get(output), 0, inputfps, cv::Size(inputwidth, inputheight), true);
        }
        if (!writer.isOpened()) {
            cerr << "Error can't create video writer" << endl;
            return 1;
        }
    }

    // Setup the marker detection
    double MarkerSize = args::get(markersize);
    MarkerDetector MDetector;
    MDetector.setThresholdParams(7, 7);
    MDetector.setThresholdParamRange(2, 0);
    std::map<uint32_t,MarkerPoseTracker> MTracker; // use a map so that for each id, we use a different pose tracker
    if (dict)
        MDetector.setDictionary(args::get(dict), 0.f);

    // Start framecounter at 0 for fps tracking
    int frameno=0;

    // Setup stdin listener
    /*
    string incommand;
    pollfd cinfd[1];
    cinfd[0].fd = fileno(stdin);
    cinfd[0].events = POLLIN;
    */
    
    // Main loop
    while (true) {

        // If signal for interrupt/termination was received, break out of main loop and exit
        if (sigflag) {
            cout << "info:signal detected:exiting track_targets" << endl;
            break;
        }

        // Listen for commands on stdin
        // This doesn't work yet, it does weird things as soon as something comes in on stdin
        /*
        if (poll(cinfd, 1, 1000))
        {
            cout << "INCOMING MESSAGE!:" << endl;
            // getline(cin, incommand);
            // cin >> incommand;
            // cout << "MESSAGE RECEIVED!:" << incommand << endl;
            stateflag = 1;
            cin.clear();
        }
        */

        // If tracking not active, skip
        if (!stateflag) {
            // Add a 1ms sleep to slow down the loop if nothing else is being done
            nanosleep((const struct timespec[]){{0, 10000000L}}, NULL);
            // If camera is started, stop and release it
            /*
            https://github.com/fnoop/vision_landing/issues/45
            if (vreader.isOpened())
                vreader.release();
            */
            continue;
        }
        
        // If camera isn't running, start it
        /*
        if (!vreader.isOpened())
            vreader.open(args::get(input));
        */
        
        // Lodge clock for start of frame
        double framestart=CLOCK();
        
        // Copy image from input stream to cv matrix, skip iteration if empty
        vreader >> rawimage; 
        if (rawimage.empty()) continue;

        // Detect markers
        vector< Marker > Markers=MDetector.detect(rawimage);
        // Do pose estimation for each marker
        for(auto & marker:Markers)
            MTracker[marker.id].estimatePose(marker,CamParam,MarkerSize);

        // If marker id is specified then look for just that id, otherwise lock on to closest marker
        int _marker;
        if (markerid) {
            _marker = args::get(markerid);
        } else {
            double _markerdistance = 9999.99;
            for (unsigned int i = 0; i < Markers.size(); i++) {
                if (Markers[i].Tvec.at<float>(0,2) > 0 && Markers[i].Tvec.at<float>(0,2) < _markerdistance) {
                    _markerdistance = Markers[i].Tvec.at<float>(0,2);
                    _marker = Markers[i].id;
                }
            }
        }

        // Loop through each detected marker
        for (unsigned int i = 0; i < Markers.size(); i++) {
            // If marker id was found, draw a green marker
            if (Markers[i].id == _marker) {
                Markers[i].draw(rawimage, Scalar(0, 255, 0), 2, false);
                // If pose estimation was successful, draw AR cube and distance
                if (Markers[i].Tvec.at<float>(0,2) > 0) {
                    // Calculate angular offsets in radians of center of detected marker
                    double xoffset = (Markers[i].getCenter().x - inputwidth / 2.0) * (fovx * (pi/180)) / inputwidth;
                    double yoffset = (Markers[i].getCenter().y - inputheight / 2.0) * (fovy * (pi/180)) / inputheight;
                    if (verbose)
                        cout << "debug: center~" << Markers[i].getCenter() << ":area~" << Markers[i].getArea() << ":marker~" << Markers[i] << endl;
                    cout << "target:" << Markers[i].id << ":" << xoffset << ":" << yoffset << ":" << Markers[i].Tvec.at<float>(0,2) << endl;
                    if (output) { // don't burn cpu cycles if no output
                        drawARLandingCube(rawimage, Markers[i], CamParam);
                        CvDrawingUtils::draw3dAxis(rawimage, Markers[i], CamParam);
                        drawVectors(rawimage, Scalar (0,255,0), 1, (i+1)*20, Markers[i].id, xoffset, yoffset, Markers[i].Tvec.at<float>(0,2), Markers[i].getCenter().x, Markers[i].getCenter().y);
                    }
                }
            // Otherwise draw a red marker
            } else {
                if (output) { // don't burn cpu cycles if no output
                    Markers[i].draw(rawimage, Scalar(0, 0, 255), 2, false);
                    drawVectors(rawimage, Scalar (0,0,255), 1, (i+1)*20, Markers[i].id, 0, 0, Markers[i].Tvec.at<float>(0,2), Markers[i].getCenter().x, Markers[i].getCenter().y);
                }
            }
        }

        if (output && args::get(output) != "window") {
            writer << rawimage;
        } else if (output && args::get(output) == "window") {
            imshow("vision_landing", rawimage);
        }
    
        // Lodge clock for end of frame    
        double framedur = CLOCK()-framestart;

        // Print fps info every 100 frames if in debug mode
        char framestr[100];
        sprintf(framestr, "debug:avgframedur~%fms:fps~%f:frameno~%d:",avgdur(framedur),avgfps(),frameno++ );
        if (verbose && (frameno % 100 == 1))
            cout << framestr << endl;

    }
    
    cout << "track_targets complete, exiting" << endl;
    cout.flush();
    return 0;

}

