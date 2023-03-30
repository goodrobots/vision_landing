#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <nlohmann/json.hpp>

#include <opencv2/opencv.hpp>

#include "apriltag-detector.h"

#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tagStandard41h12.h>
//#include <apriltag/tag36h11.h>

const bool DEBUG = false; // Slower, but processes all markers and provides more info
int COMPUTE_OFFSETS = false; // Keep disabled. May crash with SIGSEV when unable to compute

using std::cout;
using cv::Mat;
using cv::Point2f;
using cv::Scalar;

apriltag_detector_t *td;
apriltag_family_t *tf;
matd_t* tgt_offset;

// Camera callibration properties
apriltag_detection_info_t det_info;

cv::Mat zeroRotation = cv::Mat::zeros(3, 1, CV_64FC1); // No rotation
cv::Mat zeroTranslation = cv::Mat::zeros(3, 1, CV_64FC1); // No translation

using nlohmann::json;

extern bool test;
extern json CONFIG;

std::unordered_map<int, json> markersById;

void init_markers() {
	// Index markers by id
	for (const auto& marker : CONFIG["markers"]) {
		markersById[marker["id"]] = marker;
	}
}

void apriltag_init(const aruco::CameraParameters &CP) {
    td = apriltag_detector_create();

	// Apriltag families to detect

    tf = tagStandard41h12_create(); // Recommended by the dev team. See https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide#choosing-a-tag-family
	apriltag_detector_add_family(td, tf);

	//tf = tag36h11_create(); // Supports more IDs
    //apriltag_detector_add_family(td, tf);

    td->quad_decimate = 2;
    td->nthreads = 4;

    tgt_offset = matd_create(3, 1);
	tgt_offset->data[2] = 0;

	// Initialize apriltag detector with aruco camera matrix
	Mat camera_matrix = CP.CameraMatrix;
	det_info.fx = camera_matrix.at<float>(0, 0); // Focal lengths
	det_info.fy = camera_matrix.at<float>(1, 1);
	det_info.cx = camera_matrix.at<float>(0, 2); // Principal points of a camera
	det_info.cy = camera_matrix.at<float>(1, 2);

	init_markers();
}

void drawARLandingCube(Mat &Image, aruco::Marker &m, const aruco::CameraParameters &CP);

Mat matd_to_cvmat(matd_t* mat)
{
    int rows = mat->nrows;
    int cols = mat->ncols;
    Mat cvMat(rows, cols, CV_64FC1);

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            double val = matd_get(mat, i, j);
            cvMat.at<double>(i, j) = val;
        }
    }

    return cvMat;
}

bool apriltag_process_image(Mat img, aruco::CameraParameters& CP, LandingTarget& out) {
	Mat gray_image;
	cv::cvtColor(img, gray_image, cv::COLOR_BGR2GRAY);
	bool found = false;

	unsigned char* buffer = gray_image.data;
	int rows = gray_image.rows;
	int cols = gray_image.cols;

    image_u8_t img_header = { .width = cols, .height = rows, .stride = cols, .buf = buffer };
    apriltag_pose_t pose;

#if SPEED_TEST
    struct timespec start, stop;
    clock_gettime(CLOCK_MONOTONIC, &start);
#endif

    zarray_t *detections = apriltag_detector_detect(td, &img_header);

    // Sort the detections by size using zarray_sort (biggers first)
	// We prefer to use the pose estimation of the biggest marker because it provides a better estimation
	// NOTE: small markers can have very big z-component errors (over 2 [m]) because pixel error is amplified. Anyway, the projected landing point on the (x,y) plane won't change that much)
    zarray_sort(detections, [](const void* p1, const void* p2) -> int {
        apriltag_detection_t *det1 = *(apriltag_detection_t**) p1;
        apriltag_detection_t *det2 = *(apriltag_detection_t**) p2;

		// Leave unknown markers (is_null) at the end
        return markersById[det1->id].is_null() ? 1 : markersById[det1->id]["size"] < markersById[det2->id]["size"];
    });

	matd_t* landingPoint = NULL; // First detected landing point (of the biggest marker). We use it as a reference for other markers.

	if(test) cout << "---\n"; // New frame

	for (int i = 0; i < zarray_size(detections); i++) {
		apriltag_detection_t *det;
		zarray_get(detections, i, &det);

		if(test) cout << "Tag " << det->id << "\t";

		json marker = markersById[det->id];
		if(marker.is_null()) {
			if(test) cout << "Unknown\n";
			continue;
		}

		found = true;

		det_info.det = det;
		det_info.tagsize = marker["size"];
		tgt_offset->data[0] = marker["offsetX"];
		tgt_offset->data[1] = marker["offsetY"];

		estimate_tag_pose(&det_info, &pose);

		// Compute projected landing point (LP) in 3D (unit is meters)
		matd_t* op = matd_multiply(pose.R, tgt_offset); // offset point
		matd_t* markerLP = matd_add(op, pose.t);

		out.id = det->id;
		out.x = markerLP->data[0];
		out.y = markerLP->data[1];
		out.z = markerLP->data[2];
		const double pi = std::atan(1) * 4;
		out.angle = (180 / pi) * std::atan2(pose.R->data[1], pose.R->data[0]); // Angle of the marker in degrees (counter clockwise)

		// TODO: Add: << markerLP->data[0] << ", " << markerLP->data[1] << ", " << markerLP->data[2] << "), " << pose.R->data[3] << ", " << pose.R->data[0] << "\n";
		// cout << "LP: (" << tgt_offset->data[0] << ", " << tgt_offset->data[1] << ")\n";
		if(test) {
			if(!COMPUTE_OFFSETS || landingPoint) {
				cout << "LP: (" << out.x << ", " << out.y << ", " << out.z << "), angle: " << out.angle << ", offset: (" << tgt_offset->data[0] << ", " << tgt_offset->data[1] << ")\n";
			}
		}

		if(1) {
			// Render marker corners
			cv::line(img, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[1][0], det->p[1][1]), Scalar(0, 0, 255), 2);
			cv::line(img, cv::Point(det->p[1][0], det->p[1][1]), cv::Point(det->p[2][0], det->p[2][1]), Scalar(0, 0, 255), 2);
			cv::line(img, cv::Point(det->p[2][0], det->p[2][1]), cv::Point(det->p[3][0], det->p[3][1]), Scalar(0, 0, 255), 2);
			cv::line(img, cv::Point(det->p[3][0], det->p[3][1]), cv::Point(det->p[0][0], det->p[0][1]), Scalar(0, 0, 255), 2);
		} else {
			// Render marker cube
			aruco::Marker m;
			m.ssize = det_info.tagsize;
			m.Rvec = matd_to_cvmat(pose.R);
			m.Tvec = matd_to_cvmat(pose.t);
			drawARLandingCube(img, m, CP);
		}

		if(COMPUTE_OFFSETS) {
			if(landingPoint) {
				if(det->id == COMPUTE_OFFSETS) continue;

				if(1) {
					// Show error to previous computed landing point
					double ex = pow(markerLP->data[0] - landingPoint->data[0], 2);
					double ey = pow(markerLP->data[1] - landingPoint->data[1], 2);
					double ez = pow(markerLP->data[2] - landingPoint->data[2], 2);
					double err = sqrt(ex + ey + ez);

					// Error of the projected landing point for this marker (relative to the projected landing point of the biggest marker)
					cout << "\tERR: (" << ex << ", " << ey << ", " << ez << ") = " << err << "\n";
				}

				if(COMPUTE_OFFSETS) {
					// Tool for computing offsets for rest of the markers based on first detected marker.
					// USAGE: 1) Set the offset only on the biggest marker, 2) make sure the biggest marker is detected, 3) copy the "Computed offset" values in config.json
					// NOTE: The z-component can have big errors and this can affect the computed offset. That's also why we prefer the biggest marker.
					// NOTE: Returns garbage when marker sizes or z-component are incorrect.

					matd_t* _invR = matd_inverse(pose.R);
					matd_t* _diff = matd_subtract(landingPoint, pose.t);
					matd_t* _offset = matd_multiply(_invR, _diff);

					cout << "\tComputed offsets:\n\t\t\"offsetX\": " << _offset->data[0] << ",\n\t\t\"offsetY\": " << _offset->data[1] << "\n";

					matd_destroy(_invR);
					matd_destroy(_diff);
					matd_destroy(_offset);
				}

			} else if(COMPUTE_OFFSETS && det->id == COMPUTE_OFFSETS) {
				landingPoint = markerLP;

				// Repeat
				i = -1;
				continue;
			}
		}

		// --- Render landing point relative to marker ---

		Mat objectPoints(2, 3, CV_32FC1);

		// Marker center in world coordinates
		objectPoints.at<float>(0, 0) = pose.t->data[0];
		objectPoints.at<float>(0, 1) = pose.t->data[1];
		objectPoints.at<float>(0, 2) = pose.t->data[2];

		// markerLP (landing point) in world coordinates
		objectPoints.at<float>(1, 0) = markerLP->data[0];
		objectPoints.at<float>(1, 1) = markerLP->data[1];
		objectPoints.at<float>(1, 2) = markerLP->data[2];

		// Draw line from marker center to LP
		std::vector<Point2f> imagePoints;
		cv::projectPoints(objectPoints, zeroRotation, zeroTranslation, CP.CameraMatrix, CP.Distorsion, imagePoints);
		cv::line(img, imagePoints[0], imagePoints[1], Scalar(255, 0, 0, 255), 1, cv::LINE_AA);

		// Draw cross on LP
		Point2f dh = {10.0, 0};
		Point2f dv = {0, 10.0};
		cv::line(img, imagePoints[1] - dh, imagePoints[1] + dh, Scalar(0, 0, 255, 255), 1, cv::LINE_AA);
		cv::line(img, imagePoints[1] - dv, imagePoints[1] + dv, Scalar(0, 0, 255, 255), 1, cv::LINE_AA);

		if(0) {
			/**
				Code that recevies the ipc_data:
				See: https://github.com/chobitsfan/mavlink-udp-proxy/blob/new_main/my_mavlink_udp.c

                double tag_pose[6] = {0};
                if (recv(ipc_fd, tag_pose, sizeof(tag_pose), 0) > 0) {
                    gettimeofday(&tv, NULL);
                    float q[4] = {1, 0, 0, 0};
                    int tag_id = tag_pose[0];
                    double cam_r = tag_pose[1];
                    double cam_d = tag_pose[2];
                    double cam_f = tag_pose[3];
                    mavlink_msg_landing_target_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, tag_id, MAV_FRAME_BODY_FRD, 0, 0, sqrt(cam_r*cam_r+cam_d*cam_d+cam_f*cam_f), 0, 0, -cam_d, cam_r, cam_f, q, 0, 1);
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    write(uart_fd, buf, len);
                }
			*/
			double ipc_data[6];

			ipc_data[0] = markerLP->data[0];
			ipc_data[1] = markerLP->data[1];
			ipc_data[2] = markerLP->data[2];

			if (det->id == 0) {
				ipc_data[3] = pose.R->data[3]; // I only need these 2 elements to compute yaw
				ipc_data[4] = pose.R->data[0];
			} else {
				ipc_data[3] = ipc_data[4] = 0;
			}

			ipc_data[5] = det->id;

			// Send results
			//sendto(ipc_fd, ipc_data, sizeof(ipc_data), 0, (const struct sockaddr *)&server, sizeof(server));
		}

		matd_destroy(op);
		matd_destroy(markerLP);
		matd_destroy(pose.t);
		matd_destroy(pose.R);

		if(!DEBUG && !COMPUTE_OFFSETS) break; // Just use first detected marker
    }
 
#if SPEED_TEST
	if (zarray_size(detections) == 0) {
		clock_gettime(CLOCK_MONOTONIC, &stop);
		printf("%d\n", (int)((stop.tv_sec-start.tv_sec)*1000+(stop.tv_nsec-start.tv_nsec)*1e-6));
	}
#endif

    apriltag_detections_destroy(detections);

	return found;
}
