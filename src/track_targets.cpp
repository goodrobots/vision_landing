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
**/

#include <iostream>
#include "args.hxx"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "aruco/aruco.h"

using namespace cv;
using namespace aruco;

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
void drawDistance(Mat &in, Scalar color, int lineWidth, float Distance) {
    char cad[100];
    sprintf(cad, "Distance to Landing Target: %0.2fm", Distance);
    Point cent(10, 460);
    cv::putText(in, cad, cent, FONT_HERSHEY_SIMPLEX, std::max(0.5f,float(lineWidth)*0.3f), color, lineWidth);
}

// Define some initial consts
const int width = 640;
const int height = 480;
const double brightness = 0.5;
const int fps = 30;

// main..
int main(int argc, char** argv) {

    // Setup arguments for parser
    args::ArgumentParser parser("Track fiducial markers and estimate pose, output translation vectors for vision_landing");
    args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
    args::ValueFlag<string> dict(parser, "dict", "Marker Dictionary", {'d', "dict"});
    args::ValueFlag<string> output(parser, "output", "Output Stream", {'o', "output"});
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
        cout << "Error: Input stream can't be opened" << endl;
        return 1;
    }

    // Set camera properties
    vreader.set(CAP_PROP_BRIGHTNESS, brightness);
    vreader.set(CV_CAP_PROP_FRAME_WIDTH, width);
    vreader.set(CV_CAP_PROP_FRAME_HEIGHT, height);

    // Read and parse camera calibration data
    CamParam.readFromXMLFile(args::get(calibration));
    if (!CamParam.isValid()) {
        cerr << "Calibration Parameters not valid" << endl;
        return -1;
    }

    // Take a single image and resize calibration parameters based on input stream dimensions
    vreader >> rawimage;
    CamParam.resize(rawimage.size());

    // Create an output object, if output specified then setup the pipeline
    VideoWriter writer;
    if (output) {
        writer.open(args::get(output), 0, fps, cv::Size(width, height), true);
        if (!writer.isOpened()) {
            cout << "Error can't create video writer" << endl;
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

    while (true) {
        // Copy image from input stream to cv matrix, skip iteration if empty
        vreader >> rawimage; 
        if (rawimage.empty()) continue;

        // Detect markers
        vector< Marker > Markers=MDetector.detect(rawimage);
        // Do pose estimation for each marker
        for(auto & marker:Markers)
            MTracker[marker.id].estimatePose(marker,CamParam,MarkerSize);
        // Loop through each detected marker
        for (unsigned int i = 0; i < Markers.size(); i++) {
            // If marker id 1 was found, draw a green marker
            if (Markers[i].id == 1) {
                Markers[i].draw(rawimage, Scalar(0, 255, 0), 2, false);
                // If pose estimation was successful, draw AR cube and distance
                if (CamParam.isValid() && MarkerSize != -1){
                    cout << Markers[i].id << ":" << Markers[i].Tvec.at<float>(0,0) << ":" << Markers[i].Tvec.at<float>(0,1) << ":" << Markers[i].Tvec.at<float>(0,2) << endl;
                    drawARLandingCube(rawimage, Markers[i], CamParam);
                    CvDrawingUtils::draw3dAxis(rawimage, Markers[i], CamParam);
                    drawDistance(rawimage, Scalar(0, 255, 0), 1, Markers[i].Tvec.at<float>(0,2));
                }
            // Otherwise draw a red marker
            } else {
                Markers[i].draw(rawimage, Scalar(0, 0, 255), 2, false);
            }
        }

        if (output) {
            writer << rawimage;
        }
        
    }

}

