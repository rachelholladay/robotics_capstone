#include <boost/python.hpp>
#include "TagDetector.h"

// Boost Python test function

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

    // TODO need a way to pull TagData struct
    class_<TagDetector>("TagDetector",
        init<>())
        .def("setup", &TagDetector::setup)
        .def("detect_apriltags", &TagDetector::detect_apriltags)
        .def("num_detected", &TagDetector::num_detected)
        .def("close", &TagDetector::close)
        .def("test", &TagDetector::test);

}