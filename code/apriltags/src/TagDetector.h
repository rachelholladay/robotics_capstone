/* AprilTag detector class
 * Class to abstract Apriltag detection into simple steps, and
 * interface with Python wrappers via Boost
 */

#ifndef  TAGDETECTOR_H
#define TAGDETECTOR_H

#include <iostream>
// #include <boost/python.hpp>

#include "opencv2/opencv.hpp"

#include "apriltag.h"
#include "common/matd.h"


class TagData
{
    /* Data storage type, used to map apriltag IDs to 
     * relvant data - corner and center positions, and homography
     * matrix.
     */
public:

    TagData() {};
    TagData(int new_id) { id = new_id; };

    TagData(apriltag_detection_t* det)
    {
        id = det->id;

        cy = det->c[0];
        cx = det->c[1];

        h00 = det->H->data[0];
        h01 = det->H->data[1];
        h02 = det->H->data[2];
        h10 = det->H->data[3];
        h11 = det->H->data[4];
        h12 = det->H->data[5];
        h20 = det->H->data[6];
        h21 = det->H->data[7];
        h22 = det->H->data[8];
    };

    // Tag ID
    int id; 
    // The center of the detection in image pixel coordinates.
    double cy,cx;
    // The 3x3 homography matrix describing the projection from an
    // "ideal" tag (with corners at (-1,-1), (1,-1), (1,1), and (-1,
    // 1)) to pixels in the imag  e  
    double h00,h01,h02,h10,h11,h12,h20,h21,h22;

    // The corners of the tag in image pixel coordinates. These always
    // wrap counter-clock wise around the tag.
    // Always 4x2
    // std::vector<std::vector<double>> tag_corners;
    // std::vector<std::vector<double>> get_corners() { return tag_corners; };

};


class TagDetector
{
public:
    // Default Constructor
    TagDetector();

    // Setup camera
    void setup();
    // Detects tags and updates TagData map
    void detect_apriltags();
    // Returns the number of tags detected
    int num_detected() { return tags.size(); };
    // Closes and destroys detector objects
    void close();
   
    // Stores mapping of AprilTag ID to relevant tag data.
    std::map<int, TagData> tags;
    // Retrieves nth TagData object from map, irrespective of id
    TagData getTag(int n);

 // Class function test
    void test() { std::cout << "TagDetector test" << std::endl; };

private:

    // Takes the detection output and fills TagData with it
    void _convert_detections(zarray_t* detections);

    // Camera object
    cv::VideoCapture cap;

    // Tag detector object, initialized by constructor
    apriltag_detector_t* td;
    // Tag family, initialized by constructor
    apriltag_family_t* tf;

    // Default  internal parameters
    bool _debug = false;
    std::string _family = "tag36h11";
    int _border = 1;
    int _threads = 4;
    double _decimate = 1.0;
    double _blur = 0.0;
    bool _refine_edges = true;
    bool _refine_decode = false;
    bool _refine_pose = false;

};

#endif
