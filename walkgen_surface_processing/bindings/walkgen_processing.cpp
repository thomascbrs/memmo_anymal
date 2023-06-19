#include "AffordanceLoader.hpp"
#include <Point.hpp>
#include <Bayazit.hpp>

#include <boost/python.hpp>
#include <boost/python/converter/shared_ptr_to_python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>
#include <hpp/fcl/fwd.hh>

#include "vector-converter.hpp"

namespace bp = boost::python;

///////////////////////////////////
/// Binding AffordanceLoader class
///////////////////////////////////
template <typename AffordanceLoader>
struct AffordanceLoaderPythonVisitor
    : public bp::def_visitor<AffordanceLoaderPythonVisitor<AffordanceLoader>> {
  template <class PyClassAffordanceLoader>
  void visit(PyClassAffordanceLoader &cl) const {
    cl.def(bp::init<>(bp::arg(""), "Default constructor."))
        .def(bp::init<double, double, double, char *>(
            bp::args("margin", "nbTriMargin", "minArea", "affordanceName"),
            "Constructor with parameters."))

        .def("load", &AffordanceLoader::load, bp::args("filename", "R", "T"),
             "Load the .stl model.\n")
        .def("get_affordances", &AffordanceLoader::getAffordances,
             "Get the affordances.\n");
  }

  static void expose() {
    bp::class_<StdVecVec_Vector3>("StdVecVec_Vector3")
        .def(bp::vector_indexing_suite<StdVecVec_Vector3>());
    bp::class_<AffordanceLoader>("AffordanceLoader", bp::no_init)
        .def(AffordanceLoaderPythonVisitor<AffordanceLoader>());
  }
};
void exposeAffordanceLoader() {
  AffordanceLoaderPythonVisitor<AffordanceLoader>::expose();
}

void exposePoint() {
  bp::class_<Point, boost::noncopyable>(
      "Point", bp::init<double, double>(
                             bp::args("x", "y"),
                             "Constructor for point class."))
    .def_readwrite("x", &Point::x)
    .def_readwrite("y", &Point::y);
}

void exposeBayazit() {

  // typedef std::shared_ptr<ContactPhase> ContactPhasePtr;
  // typedef std::vector<std::shared_ptr<ContactPhase>> ContactPhaseVecPtr;
  // walkgen::python::StdVectorPythonVisitor<
  //     ContactPhasePtr,
  //     std::allocator<ContactPhasePtr>>::expose("StdVec");
  // walkgen::python::StdVectorPythonVisitor<
  //     ContactPhaseVecPtr,
  //     std::allocator<ContactPhaseVecPtr>>::expose("StdVec_ContactPhaseVecVec");
  // walkgen::python::StdMapPythonVisitor<
  //     int, std::vector<int>, std::less<int>,
  //     std::allocator<std::pair<const int, std::vector<int>>>>::
  //     expose("StdMap_int_VecInt");

  typedef std::shared_ptr<Point> PointPtr;
  bp::register_ptr_to_python<PointPtr>();
  typedef std::vector<PointPtr> PointVecPtr;
  walkgen::python::StdVectorPythonVisitor<
    PointPtr,
    std::allocator<PointPtr>>::expose("StdVec_PointPtr");
  walkgen::python::StdVectorPythonVisitor<
      PointVecPtr,
      std::allocator<PointVecPtr>>::expose("StdVecVec_PointPtr");

  walkgen::python::StdVectorPythonVisitor<
    Point,
    std::allocator<Point>>::expose("StdVec_Point");
  walkgen::python::StdVectorPythonVisitor<
      std::vector<Point>,
      std::allocator<std::vector<Point>>>::expose("StdVecVec_Point");

  bp::class_<Bayazit, boost::noncopyable>(
      "Bayazit", bp::init<>(bp::args(""),"Constructor for Bayazit algorithm."))
      .def("decomposePoly", &Bayazit::decomposePolyWrapper, bp::return_value_policy<bp::return_by_value>());
}

/////////////////////////////////
/// Exposing classes
/////////////////////////////////
BOOST_PYTHON_MODULE(libwalkgen_surface_processing_pywrap) {

  eigenpy::enableEigenPy();
  // eigenpy::enableEigenPySpecific<Vector3>();
  // Register converters between std::vector and Python list
  walkgen::python::StdVectorPythonVisitor<Vector3, std::allocator<Vector3>, true>::expose(
      "StdVec_Vector3d");
  exposeAffordanceLoader();
  exposePoint();
  exposeBayazit();
}
