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

/**
 * Default constructor, set up default camera reading parameters, as well
 * as the tag family to detect.
 */
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

    filename = "";

}

/**
 * Constructor for detecting from file - mainly for testing to use a provided
 * input file. Uses the tag36h11 tag family.
 */
TagDetector::TagDetector(std::string file)
{
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

    filename = file;
}

/**
 * Initializes the camera, or attempts to open the video file provided
 */
void TagDetector::setup()
{
    if(filename == "")
        cap = cv::VideoCapture(0);
    else
        cap = cv::VideoCapture(filename);
    if(!cap.isOpened())
    {
        std::cerr << 
            "TagDetector::setup: Could not open video device" 
            << std::endl;
    }
}

/**
 * uses the apriltag library to detect apriltags from incoming camera data
 */
void TagDetector::detect_apriltags()
{    
    // Convert raw frame to image_u8_t header for detection
    cv::Mat gray, frame;
    cap >> frame;
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

/**
 * Helper function for detect_apriltags()
 */
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
        tags[det->id] = TagData(det);
    }
}


/**
 * Attempts to read the index of detected apriltags
 */
TagData TagDetector::getTag(int n)
{
    // Get nth TagData object, else return invalid tag
    int x = 0;
    for(std::map<int, TagData>::iterator i = tags.begin(); 
        i != tags.end(); i++)
    {
        if(x == n)
            return i->second;
        x++;
    }
    return TagData(-1);    
}

/**
 * Ends tag family and detector operation
 */
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
