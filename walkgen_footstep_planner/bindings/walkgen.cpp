#include "FootTrajectoryBezier.hpp"
#include "Surface.hpp"

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>

namespace bp = boost::python;

/////////////////////////////////
/// Copy and deepcopy template
/////////////////////////////////
#define PYTHON_ERROR(TYPE, REASON) \
{ \
    PyErr_SetString(TYPE, REASON); \
    throw bp::error_already_set(); \
}

template<class T>
inline PyObject * managingPyObject(T *p)
{
    return typename bp::manage_new_object::apply<T *>::type()(p);
}

template<class Copyable>
bp::object
generic__copy__(bp::object copyable)
{
    Copyable *newCopyable(new Copyable(bp::extract<const Copyable
&>(copyable)));
    bp::object
result(bp::detail::new_reference(managingPyObject(newCopyable)));

    bp::extract<bp::dict>(result.attr("__dict__"))().update(
        copyable.attr("__dict__"));

    return result;
}

template<class Copyable>
bp::object
generic__deepcopy__(bp::object copyable, bp::dict memo)
{
    bp::object copyMod = bp::import("copy");
    bp::object deepcopy = copyMod.attr("deepcopy");

    Copyable *newCopyable(new Copyable(bp::extract<const Copyable
&>(copyable)));
    bp::object
result(bp::detail::new_reference(managingPyObject(newCopyable)));

    // HACK: copyableId shall be the same as the result of id(copyable)
    // in Python -
    // please tell me that there is a better way! (and which ;-p)
    int copyableId = *((int*)(copyable.ptr()));
    // int copyableId = (int)(copyable.ptr());
    memo[copyableId] = result;

    bp::extract<bp::dict>(result.attr("__dict__"))().update(
        deepcopy(bp::extract<bp::dict>(copyable.attr("__dict__"))(),
memo));

    return result;
}

/////////////////////////////////
/// Binding Bezier curves class
/////////////////////////////////
template <typename FootTrajectoryBezier>
struct FootTrajectoryBezierPythonVisitor
    : public bp::def_visitor<
          FootTrajectoryBezierPythonVisitor<FootTrajectoryBezier>> {
  template <class PyClassFootTrajectoryBezier>
  void visit(PyClassFootTrajectoryBezier &cl) const {
    cl.def(bp::init<>(bp::arg(""), "Default constructor."))
        .def("evaluateBezier", &FootTrajectoryBezier::evaluateBezier, bp::args("indice", "t"), "Evaluate Bezier curve.\n")
        .def("evaluatePoly", &FootTrajectoryBezier::evaluatePoly, bp::args("indice", "t"), "Evaluate polynomial reference curve.\n")
        .def("initialize", &FootTrajectoryBezier::initialize,"Initialize Bezier curves from Python.\n")
        .def("getT0", &FootTrajectoryBezier::getT0, "Get the T_min argument of the curve.\n" )
        .def("getCoefficients", &FootTrajectoryBezier::getCoefficients, "Get the coefficients.\n" )
        .def("create_simple_curve", &FootTrajectoryBezier::create_simple_curve, "Create a Bezier curve without collision avoidance.\n")
        .def("update", &FootTrajectoryBezier::update, "Optimises the coefficients of the Bezier curve.\n");
  }

  static void expose() {
    bp::class_<FootTrajectoryBezier>("FootTrajectoryBezier", bp::no_init)
        .def("__copy__", &generic__copy__<FootTrajectoryBezier>)
        .def("__deepcopy__", &generic__deepcopy__<FootTrajectoryBezier>)
        .def(FootTrajectoryBezierPythonVisitor<FootTrajectoryBezier>());
  }
};
void exposeFootTrajectoryBezier() {
  FootTrajectoryBezierPythonVisitor<FootTrajectoryBezier>::expose();
}

/////////////////////////////////
/// Binding Surface class
/////////////////////////////////
template <typename Surface>
struct SurfacePythonVisitor : public bp::def_visitor<SurfacePythonVisitor<Surface>> {
  template <class PyClassSurface>
  void visit(PyClassSurface& cl) const {
    cl.def(bp::init<>(bp::arg(""), "Default constructor."))
        .def(bp::init<MatrixN, VectorN, MatrixN>(bp::args("A", "b", "vertices"), "Constructor with parameters."))

        .def("get_vertices", &Surface::getVertices, "get the vertices of the surface.\n")
        .def("get_A", &Surface::getA, "get A vector of inequalities.\n")
        .def("get_b", &Surface::getb, "get b vector of inequalities.\n")

        .add_property("A", bp::make_function(&Surface::getA, bp::return_value_policy<bp::return_by_value>()))
        .add_property("b", bp::make_function(&Surface::getb, bp::return_value_policy<bp::return_by_value>()))
        .add_property("vertices",
                      bp::make_function(&Surface::getVertices, bp::return_value_policy<bp::return_by_value>()))

        .def("getHeight", &Surface::getHeight, bp::args("point"), "get the height of a point of the surface.\n")
        .def("has_point", &Surface::hasPoint, bp::args("point"), "return true if the point is in the surface.\n");
  }

  static void expose() {
    bp::class_<Surface>("Surface", bp::no_init).def(SurfacePythonVisitor<Surface>());

    ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
  }
};
void exposeSurface() { SurfacePythonVisitor<Surface>::expose(); }

/////////////////////////////////
/// Exposing classes
/////////////////////////////////
BOOST_PYTHON_MODULE(libwalkgen_footstep_planner_pywrap) {

  eigenpy::enableEigenPy();

  exposeFootTrajectoryBezier();
  exposeSurface();
}
