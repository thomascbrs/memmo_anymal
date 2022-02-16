#include "FootTrajectoryBezier.hpp"

#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>


namespace bp = boost::python;


template <typename FootTrajectoryBezier>
struct FootTrajectoryBezierPythonVisitor
    : public bp::def_visitor<FootTrajectoryBezierPythonVisitor<FootTrajectoryBezier>> {
  template <class PyClassFootTrajectoryBezier>
  void visit(PyClassFootTrajectoryBezier& cl) const {
    cl.def(bp::init<>(bp::arg(""), "Default constructor."))

        .def("print_ok", &FootTrajectoryBezier::print_ok, bp::args("k"),  "print_ok_ matrix.\n");
  }

  static void expose() {
    bp::class_<FootTrajectoryBezier>("FootTrajectoryBezier", bp::no_init).def(FootTrajectoryBezierPythonVisitor<FootTrajectoryBezier>());
  }
};
void exposeFootTrajectoryBezier() {
  FootTrajectoryBezierPythonVisitor<FootTrajectoryBezier>::expose();
}

/////////////////////////////////
/// Exposing classes
/////////////////////////////////
BOOST_PYTHON_MODULE(libwalkgen_pywrap) {
  

  eigenpy::enableEigenPy();

  exposeFootTrajectoryBezier();
}