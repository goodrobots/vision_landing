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
#include <sstream>
#include <string>
#include <queue>
#include <signal.h>
#include <poll.h>
#include <ctime>
#include <sys/select.h>
#include "args.hxx"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "aruco/aruco.h"
#include "aruco/timers.h"

using namespace cv;
using namespace aruco;

// Setup sig handling
static volatile sig_atomic_t sigflag = 0;
static volatile sig_atomic_t stateflag = 0; // 0 = stopped, 1 = started
void handle_sig(int sig)
{
    std::cout << "info:SIGNAL:" << sig << ":Received" << std::endl;
    sigflag = 1;
}
void handle_sigusr1(int sig)
{
    std::cout << "info:SIGNAL:SIGUSR1:Received:" << sig << ":Switching on Vision Processing" << std::endl;
    stateflag = 1;
}
void handle_sigusr2(int sig)
{
    std::cout << "info:SIGNAL:SIGUSR2:Received:" << sig << ":Switching off Vision Processing" << std::endl;
    stateflag = 0;
}

// Setup fps tracker
double CLOCK()
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (t.tv_sec * 1000) + (t.tv_nsec * 1e-6);
}
double _avgdur = 0;
double _fpsstart = 0;
double _avgfps = 0;
double _fps1sec = 0;
double avgdur(double newdur)
{
    _avgdur = 0.98 * _avgdur + 0.02 * newdur;
    return _avgdur;
}
double avgfps()
{
    if (CLOCK() - _fpsstart > 1000)
    {
        _fpsstart = CLOCK();
        _avgfps = 0.7 * _avgfps + 0.3 * _fps1sec;
        _fps1sec = 0;
    }
    _fps1sec++;
    return _avgfps;
}

// Define function to draw AR landing marker
void drawARLandingCube(cv::Mat &Image, Marker &m, const CameraParameters &CP)
{
    Mat objectPoints(8, 3, CV_32FC1);
    double halfSize = m.ssize / 2;

    objectPoints.at<float>(0, 0) = -halfSize;
    objectPoints.at<float>(0, 1) = -halfSize;
    objectPoints.at<float>(0, 2) = 0;
    objectPoints.at<float>(1, 0) = halfSize;
    objectPoints.at<float>(1, 1) = -halfSize;
    objectPoints.at<float>(1, 2) = 0;
    objectPoints.at<float>(2, 0) = halfSize;
    objectPoints.at<float>(2, 1) = halfSize;
    objectPoints.at<float>(2, 2) = 0;
    objectPoints.at<float>(3, 0) = -halfSize;
    objectPoints.at<float>(3, 1) = halfSize;
    objectPoints.at<float>(3, 2) = 0;

    objectPoints.at<float>(4, 0) = -halfSize;
    objectPoints.at<float>(4, 1) = -halfSize;
    objectPoints.at<float>(4, 2) = m.ssize;
    objectPoints.at<float>(5, 0) = halfSize;
    objectPoints.at<float>(5, 1) = -halfSize;
    objectPoints.at<float>(5, 2) = m.ssize;
    objectPoints.at<float>(6, 0) = halfSize;
    objectPoints.at<float>(6, 1) = halfSize;
    objectPoints.at<float>(6, 2) = m.ssize;
    objectPoints.at<float>(7, 0) = -halfSize;
    objectPoints.at<float>(7, 1) = halfSize;
    objectPoints.at<float>(7, 2) = m.ssize;

    std::vector<Point2f> imagePoints;
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
void drawVectors(Mat &in, Scalar color, int lineWidth, int vOffset, int MarkerId, double X, double Y, double Distance, double cX, double cY)
{
    char cad[100];
    sprintf(cad, "ID:%i, Distance:%0.3fm, X:%0.3f, Y:%0.3f, cX:%0.3f, cY:%0.3f", MarkerId, Distance, X, Y, cX, cY);
    Point cent(10, vOffset);
    cv::putText(in, cad, cent, FONT_HERSHEY_SIMPLEX, std::max(0.5f, float(lineWidth) * 0.3f), color, lineWidth);
}

// Summarise a marker history over the past 10 frames
int markerHistory(std::map<uint32_t, std::queue<int>> &marker_history_queue, uint32_t thisId, uint32_t marker_history)
{
    // Work out current history for this marker
    uint32_t histsum = 0;
    for (unsigned int j = 0; j < marker_history; j++)
    {
        uint32_t _val = marker_history_queue[thisId].front(); // fetch value off front of queue
        histsum += _val;                                      // add value to history sum
        marker_history_queue[thisId].pop();                   // pop value off front of queue
        marker_history_queue[thisId].push(_val);              // push value back onto end of queue
    }
    return histsum;
}

// Change active marker and reset all marker histories
void changeActiveMarker(std::map<uint32_t, std::queue<int>> &marker_history_queue, uint32_t &active_marker, uint32_t newId, uint32_t marker_history)
{
    // Set the new marker as active
    active_marker = newId;

    // Reset all marker histories.  This ensures that another marker change can't happen for at least x frames where x is history size
    for (auto &markerhist : marker_history_queue)
    {
        for (unsigned int j = 0; j < marker_history; j++)
        {
            marker_history_queue[markerhist.first].push(0);
            marker_history_queue[markerhist.first].pop();
        }
    }
}

// main..
int main(int argc, char **argv)
{
    // Unbuffer stdout and stdin
    std::cout.setf(std::ios::unitbuf);
    std::ios_base::sync_with_stdio(false);

    // Setup arguments for parser
    args::ArgumentParser parser("Track fiducial markers and estimate pose, output translation vectors for vision_landing");
    args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
    args::Flag verbose(parser, "verbose", "Verbose", {'v', "verbose"});
    args::ValueFlag<int> markerid(parser, "markerid", "Marker ID", {'i', "id"});
    args::ValueFlag<std::string> dict(parser, "dict", "Marker Dictionary", {'d', "dict"});
    args::ValueFlag<std::string> output(parser, "output", "Output Stream", {'o', "output"});
    args::ValueFlag<int> width(parser, "width", "Video Input Resolution - Width", {'w', "width"});
    args::ValueFlag<int> height(parser, "height", "Video Input Resolution - Height", {'g', "height"});
    args::ValueFlag<int> fps(parser, "fps", "Video Output FPS - Kludge factor", {'f', "fps"});
    args::ValueFlag<double> brightness(parser, "brightness", "Camera Brightness/Gain", {'b', "brightness"});
    args::ValueFlag<std::string> sizemapping(parser, "sizemapping", "Marker Size Mappings, in marker_id:size format, comma separated", {'z', "sizemapping"});
    args::ValueFlag<int> markerhistory(parser, "markerhistory", "Marker tracking history, in frames", {"markerhistory"});
    args::ValueFlag<int> markerthreshold(parser, "markerthreshold", "Marker tracking threshold, percentage", {"markerthreshold"});
    args::ValueFlag<std::string> fourcc(parser, "fourcc", "FourCC CoDec code", {'c', "fourcc"});
    args::Positional<std::string> input(parser, "input", "Input Stream");
    args::Positional<std::string> calibration(parser, "calibration", "Calibration Data");
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

    if (!input || !calibration || !markersize)
    {
        std::cout << parser;
        return 1;
    }

    // Setup core objects
    aruco::CameraParameters CamParam;
    Mat rawimage;
    VideoCapture vreader(args::get(input));

    // Bail if camera can't be opened
    if (!vreader.isOpened())
    {
        std::cerr << "Error: Input stream can't be opened" << std::endl;
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
    int inputfps = 30;
    if (fps)
        inputfps = args::get(fps);

    // If brightness is specified then use, otherwise use default
    if (brightness)
        vreader.set(CAP_PROP_BRIGHTNESS, args::get(brightness));

    // Set camera properties
    vreader.set(CV_CAP_PROP_FRAME_WIDTH, inputwidth);
    vreader.set(CV_CAP_PROP_FRAME_HEIGHT, inputheight);
    vreader.set(CV_CAP_PROP_FPS, inputfps);
    // vreader.set(CAP_PROP_BUFFERSIZE, 0); // Doesn't work yet with V4L2

    // Read and parse camera calibration data
    CamParam.readFromXMLFile(args::get(calibration));
    if (!CamParam.isValid())
    {
        std::cerr << "Calibration Parameters not valid" << std::endl;
        return -1;
    }

    // Take a single image and resize calibration parameters based on input stream dimensions
    vreader >> rawimage;
    CamParam.resize(rawimage.size());

    // Calculate the fov from the calibration intrinsics
    const double pi = std::atan(1) * 4;
    const double fovx = 2 * atan(inputwidth / (2 * CamParam.CameraMatrix.at<float>(0, 0))) * (180.0 / pi);
    const double fovy = 2 * atan(inputheight / (2 * CamParam.CameraMatrix.at<float>(1, 1))) * (180.0 / pi);
    std::cout << "info:FoVx~" << fovx << ":FoVy~" << fovy << ":vWidth~" << inputwidth << ":vHeight~" << inputheight << std::endl;

    // Create an output object, if output specified then setup the pipeline unless output is set to 'window'
    VideoWriter writer;
    if (output && args::get(output) != "window")
    {
        if (fourcc)
        {
            std::string _fourcc = args::get(fourcc);
            writer.open(args::get(output), CV_FOURCC(_fourcc[0], _fourcc[1], _fourcc[2], _fourcc[3]), inputfps, cv::Size(inputwidth, inputheight), true);
        }
        else
        {
            writer.open(args::get(output), 0, inputfps, cv::Size(inputwidth, inputheight), true);
        }
        if (!writer.isOpened())
        {
            std::cerr << "Error can't create video writer" << std::endl;
            return 1;
        }
    }

    // Setup the marker detection
    double MarkerSize = args::get(markersize);
    MarkerDetector MDetector;
    MarkerDetector::Params &MParams = MDetector.getParameters();
    MDetector.setDetectionMode(aruco::DM_VIDEO_FAST, 0.02);
    MParams.setAutoSizeSpeedUp(true,0.3);
    MParams.maxThreads = -1; // -1 = all
    MParams.setCornerRefinementMethod(aruco::CORNER_SUBPIX);
    MParams.NAttemptsAutoThresFix = 3; // This is the number of random threshold iterations when no markers found

    // Tracking variables
    float Ti = MParams.minSize;
    int ThresHold = MParams.ThresHold;

    std::map<uint32_t, MarkerPoseTracker> MTracker; // use a map so that for each id, we use a different pose tracker
    if (dict)
        MDetector.setDictionary(args::get(dict), 0.f);

    // Start framecounter at 0 for fps tracking
    int frameno = 0;

    // Setup stdin listener
    /*
    string incommand;
    pollfd cinfd[1];
    cinfd[0].fd = fileno(stdin);
    cinfd[0].events = POLLIN;
    */

    // Create a map of marker sizes from 'sizemapping' config setting
    std::map<uint32_t, float> markerSizes;
    std::stringstream ss(args::get(sizemapping));
    // First split each mapping into a vector
    std::vector<std::string> _size_mappings;
    std::string _size_mapping;
    while (std::getline(ss, _size_mapping, ','))
    {
        _size_mappings.push_back(_size_mapping);
    }
    // Next tokenize each vector element into the markerSizes map
    for (std::string const &_s : _size_mappings)
    {
        auto _i = _s.find(':');
        markerSizes[atoi(_s.substr(0, _i).data())] = atof(_s.substr(_i + 1).data());
    }
    // Debug print the constructed map
    std::cout << "info:Size Mappings:";
    for (const auto &p : markerSizes)
    {
        std::cout << p.first << "=" << p.second << ", ";
    }
    std::cout << std::endl;

    // Setup marker thresholding
    uint32_t active_marker;
    // Use a map of queues to track state of each marker in the last x frames
    std::map<uint32_t, std::queue<int>> marker_history_queue;

    // If marker history or threshold is set in parameters use them, otherwise set defaults
    uint32_t marker_history;
    if (args::get(markerhistory))
    {
        marker_history = args::get(markerhistory);
    }
    else
    {
        marker_history = 15;
    }
    std::cout << "debug:Marker History:" << marker_history << std::endl;
    uint32_t marker_threshold;
    marker_threshold = args::get(markerthreshold);
    std::cout << "debug:Marker Threshold:" << marker_threshold << std::endl;

    // Print a specific info message to signify end of initialisation
    std::cout << "info:initcomp:Initialisation Complete" << std::endl;

    // Main loop
    while (true)
    {
        // If signal for interrupt/termination was received, break out of main loop and exit
        if (sigflag)
        {
            std::cout << "info:Signal Detected:Exiting track_targets" << std::endl;
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
        if (!stateflag)
        {
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

        // Start a Timer
        ScopedTimerEvents Timer("detectloop");

        // Copy image from input stream to cv matrix
        vreader.grab();
        Timer.add("GrabImage");
        // Lodge clock for start of frame, after grabbing the frame but before decompressing/converting it.
        // This is as close as we can get to the shutter time, to match to inertial frame.
        double framestart = CLOCK();
        vreader.retrieve(rawimage);
        Timer.add("DecodeImage");
        
        // Skip this loop iteration if the frame was empty
        if (rawimage.empty())
            continue;

        // Detect markers
        std::vector<Marker> Markers = MDetector.detect(rawimage);
        Timer.add("MarkerDetect");

        // Order the markers in ascending size - we want to start with the smallest.
        std::map<float, uint32_t> markerAreas;
        std::map<uint32_t, bool> markerIds;
        for (auto &marker : Markers)
        {
            markerAreas[marker.getArea()] = marker.id;
            markerIds[marker.id] = true;
            // If the marker doesn't already exist in the threshold tracking, add and populate with full set of zeros
            if (marker_history_queue.count(marker.id) == 0)
            {
                for (unsigned int i = 0; i < marker_history; i++)
                    marker_history_queue[marker.id].push(0);
            }
        }

        // Iterate through marker history and update for this frame
        for (auto &markerhist : marker_history_queue)
        {
            // If marker was detected in this frame, push a 1
            (markerIds.count(markerhist.first)) ? markerhist.second.push(1) : markerhist.second.push(0);
            // If the marker history has reached history limit, pop the oldest element
            if (markerhist.second.size() > marker_history)
            {
                markerhist.second.pop();
            }
        }

        // If marker is set in config, use that to lock on
        if (markerid)
        {
            active_marker = args::get(markerid);
            // Otherwise find the smallest marker that has a size mapping
        }
        else
        {
            for (auto &markerArea : markerAreas)
            {
                uint32_t thisId = markerArea.second;
                if (markerSizes[thisId])
                {
                    // If the current history for this marker is >threshold, then set as the active marker and clear marker histories.  Otherwise, skip to the next sized marker.
                    uint32_t _histsum = markerHistory(marker_history_queue, thisId, marker_history);
                    float _histthresh = marker_history * ((float)marker_threshold / (float)100);
                    if (_histsum > _histthresh)
                    {
                        if (active_marker == thisId)
                            break; // Don't change to the same thing
                        std::cout << "debug:changing active_marker:" << thisId << ":" << _histsum << ":" << _histthresh << ":" << std::endl;
                        changeActiveMarker(marker_history_queue, active_marker, thisId, marker_history);
                        if (verbose)
                        {
                            std::cout << "debug:marker history:";
                            for (auto &markerhist : marker_history_queue)
                            {
                                std::cout << markerhist.first << ":" << markerHistory(marker_history_queue, markerhist.first, marker_history) << ":";
                            }
                            std::cout << std::endl;
                        }
                        break;
                    }
                }
            }
        }
        // If a marker lock hasn't been found by this point, use the smallest found marker with the default marker size
        if (!active_marker)
        {
            for (auto &markerArea : markerAreas)
            {
                uint32_t thisId = markerArea.second;
                // If the history threshold for this marker is >50%, then set as the active marker and clear marker histories.  Otherwise, skip to the next sized marker.
                uint32_t _histsum = markerHistory(marker_history_queue, thisId, marker_history);
                float _histthresh = marker_history * ((float)marker_threshold / (float)100);
                if (_histsum > _histthresh)
                {
                    std::cout << "debug:changing active_marker:" << thisId << std::endl;
                    changeActiveMarker(marker_history_queue, active_marker, thisId, marker_history);
                    break;
                }
            }
        }
        Timer.add("MarkerLock");

        // Iterate through the markers, in order of size, and do pose estimation
        for (auto &markerArea : markerAreas)
        {
            if (markerArea.second != active_marker)
                continue; // Don't do pose estimation if not active marker, save cpu cycles
            float _size;
            // If marker size mapping exists for this marker, use it for pose estimation
            if (markerSizes[markerArea.second])
            {
                _size = markerSizes[markerArea.second];
                // Otherwise use generic marker size
            }
            else if (MarkerSize)
            {
                _size = MarkerSize;
                std::cout << "debug:defaulting to generic marker size: " << markerArea.second << std::endl;
            }
            // Find the Marker in the Markers map and do pose estimation.  I'm sure there's a better way of iterating through the map..
            for (unsigned int i = 0; i < Markers.size(); i++)
            {
                if (Markers[i].id == markerArea.second)
                {
                    MTracker[markerArea.second].estimatePose(Markers[i], CamParam, _size);
                }
            }
        }
        Timer.add("PoseEstimation");

        // Iterate through each detected marker and send data for active marker and draw green AR cube, otherwise draw red AR square
        for (unsigned int i = 0; i < Markers.size(); i++)
        {
            double processtime = CLOCK() - framestart;
            // If marker id matches current active marker, draw a green AR cube
            if (Markers[i].id == active_marker)
            {
                if (output)
                {
                    Markers[i].draw(rawimage, Scalar(0, 255, 0), 2, false);
                }
                // If pose estimation was successful, calculate data and output to anyone listening.
                if (Markers[i].Tvec.at<float>(0, 2) > 0)
                {
                    // Calculate vector norm for distance
                    double distance = sqrt(pow(Markers[i].Tvec.at<float>(0, 0), 2) + pow(Markers[i].Tvec.at<float>(0, 1), 2) + pow(Markers[i].Tvec.at<float>(0, 2), 2));
                    // Calculate angular offsets in radians of center of detected marker
                    double xoffset = (Markers[i].getCenter().x - inputwidth / 2.0) * (fovx * (pi / 180)) / inputwidth;
                    double yoffset = (Markers[i].getCenter().y - inputheight / 2.0) * (fovy * (pi / 180)) / inputheight;
                    if (verbose)
                    {
                        Ti = MParams.minSize;
                        ThresHold = MParams.ThresHold;
                        std::cout << "debug:active_marker:" << active_marker << ":center~" << Markers[i].getCenter() << ":area~" << Markers[i].getArea() << ":marker~" << Markers[i] << ":vectornorm~" << distance << ":Ti~" << Ti << ":ThresHold~" << ThresHold << ":Markers~" << Markers.size() << ":processtime~" << processtime << std::endl;
                    }
                    std::cout << "target:" << Markers[i].id << ":" << xoffset << ":" << yoffset << ":" << distance << ":" << processtime << std::endl;
                    Timer.add("SendMessage");
                    // Draw AR cube and distance
                    if (output)
                    { // don't burn cpu cycles if no output
                        drawARLandingCube(rawimage, Markers[i], CamParam);
                        CvDrawingUtils::draw3dAxis(rawimage, Markers[i], CamParam);
                        drawVectors(rawimage, Scalar(0, 255, 0), 1, (i + 1) * 20, Markers[i].id, xoffset, yoffset, distance, Markers[i].getCenter().x, Markers[i].getCenter().y);
                        Timer.add("DrawGreenAR");
                    }
                }
                // Otherwise draw a red square
            }
            else
            {
                std::cout << "debug:inactive_marker:center~" << Markers[i].getCenter() << ":area~" << Markers[i].getArea() << ":marker~" << Markers[i] << ":Ti~" << Ti << ":ThresHold~" << ThresHold << ":Markers~" << Markers.size() << ":processtime~" << processtime << std::endl;
                if (output)
                { // don't burn cpu cycles if no output
                    Markers[i].draw(rawimage, Scalar(0, 0, 255), 2, false);
                    drawVectors(rawimage, Scalar(0, 0, 255), 1, (i + 1) * 20, Markers[i].id, 0, 0, Markers[i].Tvec.at<float>(0, 2), Markers[i].getCenter().x, Markers[i].getCenter().y);
                    Timer.add("DrawRedAR");
                }
            }
        }

        if (output && args::get(output) != "window")
        {
            writer << rawimage;
            Timer.add("OutputImage");
        }
        else if (output && args::get(output) == "window")
        {
            imshow("vision_landing", rawimage);
            Timer.add("OutputImage");
        }

        // Lodge clock for end of frame
        double framedur = CLOCK() - framestart;

        // Print fps info every 100 frames if in debug mode
        char framestr[100];
        sprintf(framestr, "debug:avgframedur~%fms:fps~%f:frameno~%d:", avgdur(framedur), avgfps(), frameno++);
        if (verbose && (frameno % 100 == 1))
        {
            std::cout << framestr << std::endl;
        }
        Timer.add("EndofLoop");
    }

    std::cout << "info:track_targets complete, exiting" << std::endl;
    std::cout.flush();
    return 0;
}
