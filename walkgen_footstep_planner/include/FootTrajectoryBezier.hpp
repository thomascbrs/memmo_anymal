///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief This is the header for FootTrajectoryGenerator class
///
/// \details This class generates a reference trajectory for the swing foot, in
/// position, velocity
///           and acceleration
///
//////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef TRAJGEN_BEZIER_H_INCLUDED
#define TRAJGEN_BEZIER_H_INCLUDED

#include <iostream>
#include <ostream>

#include "Surface.hpp"
#include "Types.hpp"

#include "eiquadprog/eiquadprog-fast.hpp"
#include "ndcurves/bezier_curve.h"
#include "ndcurves/fwd.h"
#include "ndcurves/optimization/details.h"
#include "ndcurves/piecewise_curve.h"
#include <Eigen/Dense>

using namespace ndcurves;
using namespace eiquadprog::solvers;

typedef optimization::problem_definition<pointX_t, double> problem_definition_t;
typedef optimization::problem_data<pointX_t, double> problem_data_t;
typedef std::vector<bezier_t::point_t,
                    Eigen::aligned_allocator<bezier_t::point_t>>
    t_point_t;

enum ElevationType { UP = 0, DOWN = 1, HORIZONTAL = 2 };

class FootTrajectoryBezier {
public:
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Constructor
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  FootTrajectoryBezier();

  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Destructor.
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ~FootTrajectoryBezier(){}; // Empty constructor

  // Copy constructor
  FootTrajectoryBezier(const FootTrajectoryBezier &other);

  void initialize(int const &N_samples_in, int const &N_samples_ineq_in,
                  int const &degree_in, double const &t_swing,
                  double const &maxHeight);

  void set_parameters_up(double const &margin_max_in, double const &t_margin_in,
                         double const &z_margin_in);
  void set_parameters_down(double const &margin_max_in,
                           double const &t_margin_in,
                           double const &z_margin_in);

  void create_simple_curve(Vector3 const &pos_init, Vector3 const &vel_init,
                           Vector3 const &pos_end, double t0);

  void update(Vector3 const &pos_init, Vector3 const &vel_init,
              Vector3 const &pos_end, double t0, Surface surface_init,
              Surface surface_end);

  Vector3 evaluateBezier(int const &indice, double const &t);
  Vector3 evaluatePoly(int const &indice, double const &t);

  void updatePolyCoeff_XY(Vector3 const &x_init, Vector3 const &v_init,
                          Vector3 const &a_init, Vector3 const &x_target,
                          double const &t0, double const &t1);
  void updatePolyCoeff_Z(Vector3 const &x_init, Vector3 const &x_target,
                         double const &t1, double const &h);

  // Methods to compute intersection point
  bool doIntersect_segment(Vector2 const &p1, Vector2 const &q1,
                           Vector2 const &p2, Vector2 const &q2);
  bool onSegment(Vector2 const &p, Vector2 const &q, Vector2 const &r);
  int orientation(Vector2 const &p, Vector2 const &q, Vector2 const &r);
  void get_intersect_segment(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2);
  void updateInequalityUp(Vector2 const &pos_init, Vector2 const &pos_end,
                          Surface const &surface);
  void updateInequalityDown(Vector2 const &pos_init, Vector2 const &pos_end,
                            Surface const &surface);

  double getT0() { return t0_; };

  MatrixN getCoefficients() {
    return vectorToEigenArray<bezier_t::t_point_t, MatrixN>(curve_.waypoints());
  };

  int flag = 0;

private:
  Vector6 Ax; ///< Coefficients for the X component
  Vector6 Ay; ///< Coefficients for the Y component
  Vector7 Az; ///< Coefficients for the Z component

  Vector2 intersectionPoint_; // tmp point of intersection
  Vector3 starting_position_; // Initial position at t0 = 0.

  double t_swing_;
  bool useBezier;
  double maxHeight_;
  double t0_;
  ElevationType elevation_;

  // Number of points in the least square problem
  int N_samples;
  int N_samples_ineq;
  //  dimension of our problem (here 3 as our curve is 3D)
  int dim = 3;
  // Degree of the Bezier curve to match the polys
  int degree;
  // Size of the optimised vector in the least square (6 = nb of initial/final
  // conditions) = dim*(degree + 1 - 6)
  int res_size;

  // Problem Definition, containing Initial/Final conditions and flag for
  // conditions.
  problem_definition_t pDef = problem_definition_t(3);
  static const bool safe = true;

  // Vector of Bezier curves, containing the curves optimised for each foot
  bezier_t fitBezier;
  bezier_t::bezier_curve_t curve_;

  // QP matrix
  MatrixN P_;
  VectorN q_;
  MatrixN G_;
  VectorN h_;
  MatrixN C_;
  VectorN d_;
  VectorN x;

  Vector3 ineq_;
  double ineq_vector_;

  // Hyper parameters.
  // Margin around the obstacle.
  double margin_max_up_;
  double margin_max_down_;
  // % of the curve after critical point.
  double t_margin_up_;
  double t_margin_down_;
  double t_stop_up_;
  double t_stop_down_;
  // % of the height of the obstacle around the critical point.
  double z_margin_up_;
  double z_margin_down_;
  double margin_adapted_;
  double EPS_; // epsilon for QP.

  // QP solver
  EiquadprogFast_status expected = EIQUADPROG_FAST_OPTIMAL;
  EiquadprogFast_status status;
  EiquadprogFast qp;
};

#endif // TRAJGEN_H_INCLUDED
