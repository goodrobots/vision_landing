#include "aruco/aruco.h"

struct LandingTarget {
	int id;
	double x;
	double y;
	double z;
	double angle;
};

void apriltag_init(const aruco::CameraParameters &CP);
bool apriltag_process_image(cv::Mat img, aruco::CameraParameters& CamParam, LandingTarget& out);
