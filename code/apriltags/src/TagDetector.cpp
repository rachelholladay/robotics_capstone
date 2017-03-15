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
    _convert_detections(detections);

    zarray_destroy(detections);

}

void TagDetector::_convert_detections(zarray_t *detections)
{    
    int size = zarray_size(detections);
    tags.clear();

    // Convert each detection type from the AprilTags library
    // into a TagData object
    for(int i = 0; i < size; i++)
    {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        std::vector<std::vector<double>> tag_corners
            { { det->p[0][0], det->p[0][1]},
              { det->p[1][0], det->p[1][1]},
              { det->p[2][0], det->p[2][1]},
              { det->p[3][0], det->p[3][1]} };

        std::vector<double> center
            { det->c[0], det->c[1] };

        cv::Mat H = (Mat_<double>(3,3) <<
            det->H->data[0], det->H->data[1], det->H->data[2],
            det->H->data[3], det->H->data[4], det->H->data[5],
            det->H->data[6], det->H->data[7], det->H->data[8]);

        tags[det->id] = TagData(tag_corners, center, H);
    }

}

void TagDetector::close()
{
    apriltag_detector_destroy(td);
    if (_family == "tag36h11")
        tag36h11_destroy(tf);
    else if (_family == "tag36h10")
        tag36h10_destroy(tf);
    else if (_family == "tag36artoolkit")
        tag36artoolkit_destroy(tf);
    else if (_family == "tag25h9")
        tag25h9_destroy(tf);
    else if (_family ==  "tag25h7")
        tag25h7_destroy(tf);  
}