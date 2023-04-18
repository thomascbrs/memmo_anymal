#include "FootTrajectoryBezier.hpp"
#include "ContactPhase.hpp"
#include "ContactSchedule.hpp"
#include "Surface.hpp"
#include "Params.hpp"
#include "FootTrajectoryWrapper.hpp"
#include <Gait.hpp>

#include <pinocchio/spatial/se3.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/converter/shared_ptr_to_python.hpp>
#include <eigenpy/eigenpy.hpp>

#include "utils/map-converter.hpp"
#include "utils/vector-converter.hpp"

namespace bp = boost::python;

/////////////////////////////////
/// Copy and deepcopy template
/////////////////////////////////
#define PYTHON_ERROR(TYPE, REASON)     \
    {                                  \
        PyErr_SetString(TYPE, REASON); \
        throw bp::error_already_set(); \
    }

template <class T>
inline PyObject *managingPyObject(T *p)
{
    return typename bp::manage_new_object::apply<T *>::type()(p);
}

template <class Copyable>
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

template <class Copyable>
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
    int copyableId = *((int *)(copyable.ptr()));
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
          FootTrajectoryBezierPythonVisitor<FootTrajectoryBezier>>
{
    template <class PyClassFootTrajectoryBezier>
    void visit(PyClassFootTrajectoryBezier &cl) const
    {
        cl.def(bp::init<>(bp::arg(""), "Default constructor."))
            .def("evaluateBezier", &FootTrajectoryBezier::evaluateBezier, bp::args("indice", "t"), "Evaluate Bezier curve.\n")
            .def("evaluatePoly", &FootTrajectoryBezier::evaluatePoly, bp::args("indice", "t"), "Evaluate polynomial reference curve.\n")
            .def("initialize", &FootTrajectoryBezier::initialize, "Initialize Bezier curves from Python.\n")
            .def("set_parameters_up", &FootTrajectoryBezier::set_parameters_up, "Set hyperparameters for going up.")
            .def("set_parameters_down", &FootTrajectoryBezier::set_parameters_down, "Set hyperparameters for going down.")
            .def("getT0", &FootTrajectoryBezier::getT0, "Get the T_min argument of the curve.\n")
            .def("getCoefficients", &FootTrajectoryBezier::getCoefficients, "Get the coefficients.\n")
            .def("create_simple_curve", &FootTrajectoryBezier::create_simple_curve, "Create a Bezier curve without collision avoidance.\n")
            .def("update", &FootTrajectoryBezier::update, "Optimises the coefficients of the Bezier curve.\n");
    }

    static void expose()
    {
        bp::class_<FootTrajectoryBezier>("FootTrajectoryBezier", bp::no_init)
            .def("__copy__", &generic__copy__<FootTrajectoryBezier>)
            .def("__deepcopy__", &generic__deepcopy__<FootTrajectoryBezier>)
            .def(FootTrajectoryBezierPythonVisitor<FootTrajectoryBezier>());
    }
};
void exposeFootTrajectoryBezier()
{
    FootTrajectoryBezierPythonVisitor<FootTrajectoryBezier>::expose();
}

// Expose the FootTrajectoryBezierWrapper class to Python
void exposeBezierWrapper()
{   
    bp::register_ptr_to_python<std::shared_ptr<FootTrajectoryWrapper>>();
    bp::class_<FootTrajectoryWrapper, boost::noncopyable>("FootTrajectoryWrapper", bp::init<double, int, double, pinocchio::SE3, pinocchio::SE3, bp::optional<Params>>\
    (bp::args("dt", "N", "step_height", "M_current", "M_next"), "Constructor for parameter to laod the yaml."))
        .def("position", &FootTrajectoryWrapper::position, bp::args("k"))
        .def("velocity", &FootTrajectoryWrapper::velocity, bp::args("k"))
        .def("update", &FootTrajectoryWrapper::update, bp::args("x0", "v0", "xf", "t0", "init_surface", "end_surface"))
        .def("get_coefficients", &FootTrajectoryWrapper::get_coefficients)
        .def("get_t0", &FootTrajectoryWrapper::getT0)
        .def("get_curve", &FootTrajectoryWrapper::get_curve, bp::return_value_policy<bp::return_by_value>())
        .def("__copy__", &generic__copy__<FootTrajectoryWrapper>)
        .def("__deepcopy__", &generic__deepcopy__<FootTrajectoryWrapper>);
}

void exposeQuadrupedalGait()
{   
    // typedef std::map<std::string, pinocchio::SE3> StdMap_string_SE3; // Defined in Gait.hpp
    // typedef std::vector<StdMap_string_SE3> StdVec_Map_string_SE3; // Defined in Gait.hpp
    walkgen::python::StdMapPythonVisitor<typename StdMap_string_SE3::key_type,typename StdMap_string_SE3::mapped_type,typename StdMap_string_SE3::key_compare,typename StdMap_string_SE3::allocator_type>::expose("StdMap_string_SE3");
    walkgen::python::StdVectorPythonVisitor<typename StdVec_Map_string_SE3::value_type, typename StdVec_Map_string_SE3::allocator_type>::expose("StdVec_Map_string_SE3");

    bp::class_<QuadrupedalGaitGenerator>("QuadrupedalGaitGenerator", bp::init<double, int, std::string, std::string, std::string, std::string>(
        (bp::arg("dt")=0.02, bp::arg("S")=4, bp::arg("lf")="LF_FOOT", bp::arg("lh")="LH_FOOT", bp::arg("rf")="RF_FOOT", bp::arg("rh")="RH_FOOT")
    ))
    .def("walk", &QuadrupedalGaitGenerator::walk,
         (bp::arg("contacts"), bp::arg("N_ds"), bp::arg("N_ss"), 
          bp::arg("N_uds")=0, bp::arg("N_uss")=0, bp::arg("stepHeight")=0.15, 
          bp::arg("startPhase")=true, bp::arg("endPhase")=true))
    .def("trot", &QuadrupedalGaitGenerator::trot,
         (bp::arg("contacts"), bp::arg("N_ds"), bp::arg("N_ss"), 
          bp::arg("N_uds")=0, bp::arg("N_uss")=0, bp::arg("stepHeight")=0.15, 
          bp::arg("startPhase")=true, bp::arg("endPhase")=true))
    .def_readwrite("dt", &QuadrupedalGaitGenerator::dt_, "Time step duration")
    .def_readwrite("S", &QuadrupedalGaitGenerator::S_, "Number of contact")
    .def_readwrite("lf", &QuadrupedalGaitGenerator::lf_, "Left front leg")
    .def_readwrite("lh", &QuadrupedalGaitGenerator::lh_, "Left hind leg")
    .def_readwrite("rf", &QuadrupedalGaitGenerator::rf_, "Rigth front name")
    .def_readwrite("rh", &QuadrupedalGaitGenerator::rh_, "Rigth front name")
    .def("__copy__", &generic__copy__<QuadrupedalGaitGenerator>)
    .def("__deepcopy__", &generic__deepcopy__<QuadrupedalGaitGenerator>);
}

/////////////////////////////////
/// Binding Surface class
/////////////////////////////////
template <typename Surface>
struct SurfacePythonVisitor : public bp::def_visitor<SurfacePythonVisitor<Surface>>
{
    template <class PyClassSurface>
    void visit(PyClassSurface &cl) const
    {
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

    static void expose()
    {
        bp::class_<Surface>("Surface", bp::no_init).def(SurfacePythonVisitor<Surface>());

        ENABLE_SPECIFIC_MATRIX_TYPE(MatrixN);
    }
};
void exposeSurface() { SurfacePythonVisitor<Surface>::expose(); }

// Expose the ContactPhase class to Python
void exposeParams()
{
    bp::class_<Params, boost::noncopyable>("FootStepPlannerParams", bp::init< bp::optional<std::string> >(bp::args("filename"), "Constructor for parameter to laod the yaml."))
        .def_readwrite("type", &Params::type, "Type of gait")
        .def_readwrite("dt", &Params::dt, "Time step duration")
        .def_readwrite("horizon", &Params::horizon, "Planning horizon (in steps)")
        .def_readwrite("nsteps", &Params::nsteps, "Number of steps to plan")
        .def_readwrite("stepHeight", &Params::stepHeight, "Step height")
        .def_readwrite("N_ds", &Params::N_ds, "Number of double support phases")
        .def_readwrite("N_ss", &Params::N_ss, "Number of single support phases")
        .def_readwrite("N_uds", &Params::N_uds, "Number of unloading double support phases")
        .def_readwrite("N_uss", &Params::N_uss, "Number of unloading single support phases")
        .def_readwrite("N_phase_return", &Params::N_phase_return, "Number of phases for returning to the starting position")
        .def_readwrite("margin_up", &Params::margin_up, "Margin for the Bezier curves")
        .def_readwrite("t_margin_up", &Params::t_margin_up, "Time margin for the Bezier curves")
        .def_readwrite("z_margin_up", &Params::z_margin_up, "Height margin for the Bezier curves")
        .def_readwrite("margin_down", &Params::margin_down, "Margin for the Bezier curves")
        .def_readwrite("t_margin_down", &Params::t_margin_down, "Time margin for the Bezier curves")
        .def_readwrite("z_margin_down", &Params::z_margin_down, "Height margin for the Bezier curves")
        .def_readwrite("N_sample", &Params::N_sample, "Number of samples for the Bezier curves")
        .def_readwrite("N_sample_ineq", &Params::N_sample_ineq, "Number of samples for the inequality constraints")
        .def_readwrite("degree", &Params::degree, "Degree of the Bezier curves")
        .def("__copy__", &generic__copy__<Params>)
        .def("__deepcopy__", &generic__deepcopy__<Params>);
}

// Binding ContactType class
void exposeContactType()
{
    bp::enum_<ContactType>("ContactType")
        .value("POINT", ContactType::POINT)
        .value("FULL", ContactType::FULL);
}

// Expose the ContactPhase class to Python
void exposeContactPhase()
{
    bp::register_ptr_to_python<std::shared_ptr<FootTrajectoryBezier>>();
    bp::register_ptr_to_python<std::shared_ptr<ContactPhase>>();
    // TODO : bp::optionnal does not work for both variables.
    bp::class_<ContactPhase, boost::noncopyable>("ContactPhase", bp::init<int, bp::optional<ContactType> >(bp::args("T", "contactType"), "Constructor for a ContactPhase object without a trajectory."))
        .def(bp::init<int, ContactType, std::shared_ptr<FootTrajectoryWrapper>>(bp::args("T", "contactType", "trajectory"),
                                                                               "Constructor for a ContactPhase object."))
        .def(bp::init<int, std::shared_ptr<FootTrajectoryWrapper>>(bp::args("T", "trajectory"),
                                                                               "Constructor for a ContactPhase object."))
        .add_property("trajectory",
                      bp::make_function(&ContactPhase::getTrajectory, bp::return_value_policy<bp::return_by_value>()),
                      &ContactPhase::setTrajectory, "Trajectory in the ContactPhase.")
        .def_readwrite("T", &ContactPhase::T_, "Number of nodes in the ContactPhase.")
        .def_readwrite("contactType", &ContactPhase::contactType_, "Number of nodes in the ContactPhase.")
        .def("__add__", &ContactPhase::operator+, bp::return_value_policy<bp::return_by_value>())
        .def("__copy__", &generic__copy__<ContactPhase>)
        .def("__deepcopy__", &generic__deepcopy__<ContactPhase>);
}

void exposeContactSchedule()
{

    typedef std::shared_ptr<ContactPhase> ContactPhasePtr;
    typedef std::vector<std::shared_ptr<ContactPhase>> ContactPhaseVecPtr;
    walkgen::python::StdVectorPythonVisitor<ContactPhasePtr, std::allocator<ContactPhasePtr>>::expose("StdVec_ContactPhaseVec");
    walkgen::python::StdVectorPythonVisitor<ContactPhaseVecPtr, std::allocator<ContactPhaseVecPtr>>::expose("StdVec_ContactPhaseVecVec");
    walkgen::python::StdMapPythonVisitor<int, std::vector<int>, std::less<int>, std::allocator<std::pair<const int, std::vector<int>>>>::expose("StdMap_int_VecInt");

    bp::register_ptr_to_python<std::shared_ptr<ContactSchedule>>();
    bp::class_<ContactSchedule, boost::noncopyable>("ContactSchedule", bp::init<double, int, int, std::vector<std::string>>(bp::args("dt", "T", "S_total", "contactNames"), "Constructor for a ContactSchedule object."))
        .def("addSchedule", &ContactSchedule::addSchedule)
        .def("updateSwitches", &ContactSchedule::updateSwitches)
        .def("checkSchedule", &ContactSchedule::checkSchedule)
        .def_readwrite("dt", &ContactSchedule::dt_)
        .def_readwrite("T", &ContactSchedule::T_)
        .def_readwrite("S_total", &ContactSchedule::S_total_)
        .def_readwrite("C", &ContactSchedule::C_)
        .def_readwrite("contactNames", &ContactSchedule::contactNames_)
        .def_readwrite("phases", &ContactSchedule::phases_)
        .def_readwrite("switches", &ContactSchedule::switches_)
        .def("__copy__", &generic__copy__<ContactSchedule>)
        .def("__deepcopy__", &generic__deepcopy__<ContactSchedule>)
        .def("__add__", &ContactSchedule::operator+, bp::return_value_policy<bp::return_by_value>());
}

/////////////////////////////////
/// Exposing classes
/////////////////////////////////
BOOST_PYTHON_MODULE(libwalkgen_footstep_planner_pywrap)
{
    eigenpy::enableEigenPy();

    exposeFootTrajectoryBezier();
    exposeBezierWrapper();
    exposeSurface();
    exposeParams();
    exposeContactType();
    exposeContactPhase();
    exposeContactSchedule();
    exposeQuadrupedalGait();
}
