#include "TerrainSlope.hpp"
#include "vector-converter.hpp"

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>
#include <hpp/fcl/fwd.hh>


namespace bp = boost::python;

///////////////////////////////////
/// Binding TerrainSlope class
///////////////////////////////////
template <typename TerrainSlope>
struct TerrainSlopePythonVisitor : public bp::def_visitor<TerrainSlopePythonVisitor<TerrainSlope>> {
  template <class PyClassTerrainSlope>
  void visit(PyClassTerrainSlope& cl) const {
    cl.def(bp::init<int, int, double>(bp::arg(""), "Constructor with parameters."))
      .def(bp::init<>(bp::arg("fitSizeX,fitSizeY, fitLength"), "Constructor with parameters."))
      .def("get_slope", &TerrainSlope::getSlope, bp::args("pose", "rotation_yaw", "surfaces"), "Get slope of the terrain.\n");
  }

  static void expose() {
    bp::class_<TerrainSlope>("TerrainSlope", bp::no_init).def(TerrainSlopePythonVisitor<TerrainSlope>());
  }
};
void exposeTerrainSlope() { TerrainSlopePythonVisitor<TerrainSlope>::expose(); }

/////////////////////////////////
/// Exposing classes
/////////////////////////////////
BOOST_PYTHON_MODULE(libwalkgen_surface_planner_pywrap) {

  eigenpy::enableEigenPy();
  // eigenpy::enableEigenPySpecific<Vector3>();
  // Register converters between std::vector and Python list
  StdVectorPythonVisitor<MatrixN, std::allocator<MatrixN>, true>::expose("StdVec_MatrixXd");
  exposeTerrainSlope();
}
