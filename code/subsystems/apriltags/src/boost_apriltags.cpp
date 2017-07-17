#include <boost/python.hpp>
#include "TagDetector.h"

// Boost Python test function
using namespace std;

/**
 * Boost python test function
 */
int test_fn()
{
    TagDetector d;
    d.test();
    d.setup();
    while(true)
    {
        d.detect_apriltags();
        if(d.num_detected() > 0)
        {
            std::cout << d.num_detected() << std::endl;
            break;
        }
    }
    return 0;
}

/**
 * Defines boost module for apriltags. Apriltag data is transmit with
 * four main parameter sets:
 *      id, representing the ID of the apriltag detected
 *      cx, width of the camera data
 *      cy, height of the camera data
 *      H, the 3x3 transformation matrix of the detected tag from an
 *          "ideal" tag as defined in TagDetector.h
 */
BOOST_PYTHON_MODULE(boost_apriltags)
{
    using namespace boost::python;
    def("test_fn", test_fn);

    class_<TagData>("TagData", init<>())
        .add_property("id", &TagData::id)
        .add_property("cy", &TagData::cy)
        .add_property("cx", &TagData::cx)
        .add_property("h00", &TagData::h00)
        .add_property("h01", &TagData::h01)
        .add_property("h02", &TagData::h02)
        .add_property("h10", &TagData::h10)
        .add_property("h11", &TagData::h11)
        .add_property("h12", &TagData::h12)
        .add_property("h20", &TagData::h20)
        .add_property("h21", &TagData::h21)
        .add_property("h22", &TagData::h22);

    class_<TagDetector>("TagDetector",
        init<>())
        .def(init<std::string>())
        .def("setup", &TagDetector::setup)
        .def("detect_apriltags", &TagDetector::detect_apriltags)
        .def("num_detected", &TagDetector::num_detected)
        .def("close", &TagDetector::close)
        .def("test", &TagDetector::test)
        .def("getTag", &TagDetector::getTag, "n");
    }

