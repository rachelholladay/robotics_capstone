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

    // Setup camera
    void setup();

    // Detects tags and updates TagData map
    void detect_apriltags();

    int num_detected() { return tags.size(); };

    void close();

    void test() { std::cout << "TagDetector test" << std::endl; };

    // Stores mapping of AprilTag ID to relevant tag data.
    std::map<int, TagData> tags;

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


// int test_fn()
// {
//     TagDetector d;
//     d.test();
//     d.setup();
//     while(true)
//     {
//         d.detect_apriltags();
//         if(d.num_detected() > 0)
//         {
//             std::cout << d.num_detected() << std::endl;
//             break;
//         }

//     }
//     return 0;
// }

// // Boost Python test function
// BOOST_PYTHON_MODULE(apriltags)
// {
//     using namespace boost::python;
//     def("test_fn", test_fn);

//     // TODO add setup to establish camera, make camera
//     // internal variable
//     // require no frame in python, only scan when called
//     // TODO need a way to pull TagData struct
//     class_<TagDetector>("TagDetector",
//         init<>())
//         .def("setup", &TagDetector::setup)
//         .def("detect_apriltags", &TagDetector::detect_apriltags)
//         .def("num_detected", &TagDetector::num_detected)
//         .def("close", &TagDetector::close)
//         .def("test", &TagDetector::test);

//     //  // Expose the class Animal.
//     // class_<Animal>("Animal",
//     //     init<std::string const &>())
//     //     .def("get_address", &Animal::get_address)
//     //     .add_property("name", &Animal::get_name, &Animal::set_name)
// }


#endif
