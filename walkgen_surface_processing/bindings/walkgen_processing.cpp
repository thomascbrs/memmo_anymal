#include "AffordanceLoader.hpp"
#include "vector-converter.hpp"

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>
#include <hpp/fcl/fwd.hh>


namespace bp = boost::python;

///////////////////////////////////
/// Binding AffordanceLoader class
///////////////////////////////////
template <typename AffordanceLoader>
struct AffordanceLoaderPythonVisitor : public bp::def_visitor<AffordanceLoaderPythonVisitor<AffordanceLoader>> {
  template <class PyClassAffordanceLoader>
  void visit(PyClassAffordanceLoader& cl) const {
    cl.def(bp::init<>(bp::arg(""), "Default constructor."))
        .def("load", &AffordanceLoader::load, bp::args("filename", "R", "T"), "Load the .stl model.\n")
        .def("get_affordances", &AffordanceLoader::getAffordances, "Get the affordances.\n");
  }

  static void expose() {
    bp::class_<StdVecVec_Vector3>("StdVecVec_Vector3").def(bp::vector_indexing_suite<StdVecVec_Vector3>());
    bp::class_<AffordanceLoader>("AffordanceLoader", bp::no_init).def(AffordanceLoaderPythonVisitor<AffordanceLoader>());
  }
};
void exposeAffordanceLoader() { AffordanceLoaderPythonVisitor<AffordanceLoader>::expose(); }

/////////////////////////////////
/// Exposing classes
/////////////////////////////////
BOOST_PYTHON_MODULE(libwalkgen_surface_processing_pywrap) {

  eigenpy::enableEigenPy();
  // eigenpy::enableEigenPySpecific<Vector3>();
  // Register converters between std::vector and Python list
  StdVectorPythonVisitor<Vector3, std::allocator<Vector3>, true>::expose("StdVec_Vector3d");
  exposeAffordanceLoader();
}
