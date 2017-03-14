/* TagDetector.cpp
 * Implementation file for TagDetector.h
 */

#include "TagDetector.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"

using namespace cv;
using namespace std;

TagDetector::TagDetector()
{
    std::cout << "default constructor" << std::endl;

    if (_family == "tag36h11")
        tf = tag36h11_create();
    else if (_family == "tag36h10")
        tf = tag36h10_create();
    else if (_family == "tag36artoolkit")
        tf = tag36artoolkit_create();
    else if (_family == "tag25h9")
        tf = tag25h9_create();
    else if (_family ==  "tag25h7")
        tf = tag25h7_create();
    else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }
    tf->black_border = _border;

    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    td->quad_decimate = _decimate;
    td->quad_sigma = _blur;
    td->nthreads = _threads;
    td->debug = _debug;
    td->refine_edges = _refine_edges;
    td->refine_decode = _refine_decode;
    td->refine_pose = _refine_pose;

}

void TagDetector::detect_apriltags(cv::Mat frame)
{
    // Convert raw frame to image_u8_t header for detection
    cv::Mat gray;
    cv::cvtColor(frame, gray, COLOR_BGR2GRAY);
    image_u8_t im = { .width = gray.cols,
        .height = gray.rows,
        .stride = gray.cols,
        .buf = gray.data
    };

    // Find AprilTags
    zarray_t *detections = apriltag_detector_detect(td, &im);

    // Fill in TagData structure by mapping AprilTag IDs to
    // completed TagData objects
    

}

void TagDetector::close()
{

}