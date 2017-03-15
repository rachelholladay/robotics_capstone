/* AprilTag detector class
 * Class to abstract Apriltag detection into simple steps, and
 * interface with Python wrappers via Boost
 */

#ifndef  TAGDETECTOR_H
#define TAGDETECTOR_H

#include <iostream>

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

    TagData(std::vector<std::vector<double>> p, 
        std::vector<double> c, cv::Mat H)
    {
        tag_corners = p;
        center = c;
        homography = H;
    };

    // The corners of the tag in image pixel coordinates. These always
    // wrap counter-clock wise around the tag.
    // Always 4x2
    std::vector<std::vector<double>> tag_corners;

    // The center of the detection in image pixel coordinates.
    // Always 2x1
    std::vector<double> center;

    // The 3x3 homography matrix describing the projection from an
    // "ideal" tag (with corners at (-1,-1), (1,-1), (1,1), and (-1,
    // 1)) to pixels in the imag    
    cv::Mat homography;
};

class TagDetector
{
public:
    // Default Constructor
    TagDetector();

    // Detects tags and updates mapping of TagData
    void detect_apriltags(cv::Mat frame);

    void test()
    {
        std::cout << "hello from TagDetector" << std::endl;
    };

    void close();

private:

    // Takes the detection output and fills TagData with it
    void convert_detections(zarray_t* detections);

    // Stores mapping of AprilTag ID to relevant tag data.
    std::map<int, TagData> tags;

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
