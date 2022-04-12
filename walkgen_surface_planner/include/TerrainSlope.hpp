///////////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief This is the header for AffordanceLoader class
///
/// \details AffordanceLoader data structure
///
//////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef AffordanceLoader_H_INCLUDED
#define AffordanceLoader_H_INCLUDED

#include "Types.hpp"
#include <Eigen/Dense>

#include <hpp/fcl/internal/tools.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>
#include "eiquadprog/eiquadprog-fast.hpp"

using hpp::fcl::GJKSolver;
using namespace hpp::fcl;

class TerrainSlope {
 public:
  // Constructor
  TerrainSlope();
  ///////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Constructor with parameters. Evaluate the slope of the terrain by evaluating the height
  ///        of the convex patch using a square centered on the robot position.
  ///        Choose the number of points in X, Y local frame.
  ///
  /// \param[in] pose Translation.
  /// \param[in] R Yaw rotation matrix.
  /// \param[in] vecCollisionPts Surface points.
  ///
  ///////////////////////////////////////////////////////////////////////////
  TerrainSlope(int const& fitSizeX, int const& fitSizeY, double const& fitLength);

  // Destructor
  ~TerrainSlope() {}

  ///////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Get roll, pitch and yaw for of the slope of the terrain.
  ///
  /// \param[in] pose Translation.
  /// \param[in] R Yaw rotation matrix.
  /// \param[in] vecCollisionPts Surface points.
  ///
  /// \return Roll, pitch, yaw vector3.
  ///
  ///////////////////////////////////////////////////////////////////////////
  VectorN getSlope(VectorN const& pose, MatrixN const& rotation_yaw,  std::vector<MatrixN> const& vecCollisionPts);

  ///////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Get surface equation [a,b,c,d] such as : ax + by + cz + d = 0.
  ///
  /// \param[in] pose Points of the surface.
  ///
  /// \return [a,b,c,d] vector4.
  ///
  ///////////////////////////////////////////////////////////////////////////
  Vector4 getSurfaceEq(MatrixN const& points);

  ///////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Get Height of the surface in X,Y position.
  ///
  /// \param[in] x X-axis position..
  /// \param[in] y Y-axis position.
  /// \param[in] equation Surface equation.
  ///
  /// \return height evaluated at x,y.
  ///
  ///////////////////////////////////////////////////////////////////////////
  double getHeight(double const& x, double const& y, Vector4 const& equation);

 private:
  GJKSolver solver;
  hpp::fcl::Transform3f tf1_, tf2_;
  Matrix3 rotation1_, rotation2_;
  // Given a X,Y position, a box type is used to evaluate which convex object is crossed.
  hpp::fcl::Box box_;

  // Use hpp-fcl collision function to evaluate the collision.
  // hpp::fcl::CollisionRequest request_collision_;
  // hpp::fcl::CollisionResult res_collision_;

  // Use hpp-fcl distance function to evaluate the collision (faster).
  hpp::fcl::DistanceRequest request_distance_;
  hpp::fcl::DistanceResult res_distance_;

  int fitSizeX_; // Number of points on x axis.
  int fitSizeY_; // Number of points on y axis.
  double fitLength_; // Half size of the square centered of the robot position.
  VectorN fit_ = VectorN::Zero(3);

  std::vector<hpp::fcl::ConvexBase*> convexObjects_;
  std::vector<Vector4> surfaceEqs_;

  // min. 1/2 * x' C_ x + q_' x
  // s.t. C_ x + d_ = 0
  //      G_ x + h_ >= 0
  MatrixN P_ = MatrixN::Zero(3, 3);
  VectorN q_ = VectorN::Zero(3);

  MatrixN G_ = MatrixN::Zero(3, 3);
  VectorN h_ = VectorN::Zero(3);

  MatrixN C_ = MatrixN::Zero(3, 3);
  VectorN d_ = VectorN::Zero(3);

  MatrixN A_;
  VectorN b_;

  // qp solver
  eiquadprog::solvers::EiquadprogFast_status status_;
  eiquadprog::solvers::EiquadprogFast qp_;
};

#endif  // AffordanceLoader_H_INCLUDED
