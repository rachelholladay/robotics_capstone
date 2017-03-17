#include <boost/python.hpp>
#include "TagDetector.h"

// Boost Python test function
using namespace std;

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


    // TODO need a way to pull TagData struct
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

