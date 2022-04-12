#include "TerrainSlope.hpp"

using namespace hpp;
TerrainSlope::TerrainSlope() {
  fitSizeX_ = 10;
  fitSizeY_ = 10;
  fitLength_ = 0.3;
  A_ = MatrixN::Zero(fitSizeX_ * fitSizeY_, 3);
  b_ = VectorN::Zero(fitSizeX_ * fitSizeY_);
  tf1_.setIdentity();
  tf2_.setIdentity();
  rotation2_.setIdentity();
  rotation1_.setIdentity();

  box_ = hpp::fcl::Box(0.01, 0.01, 3.);
  request_distance_ = hpp::fcl::DistanceRequest(false, 0., 0.);  // use distance function in hppfcl
  // request_collision =  hpp::fcl::CollisionRequest(hpp::fcl::DISTANCE_LOWER_BOUND, 1);
  // request_collision =  hpp::fcl::CollisionRequest(hpp::fcl::NO_REQUEST, 1);
  // request_collision.enable_distance_lower_bound = true;
}

TerrainSlope::TerrainSlope(int const& fitSizeX, int const& fitSizeY, double const& fitLength) {
  fitSizeX_ = fitSizeX;
  fitSizeY_ = fitSizeY;
  fitLength_ = fitLength;
  A_ = MatrixN::Zero(fitSizeX_ * fitSizeY_, 3);
  b_ = VectorN::Zero(fitSizeX_ * fitSizeY_);
  tf1_.setIdentity();
  tf2_.setIdentity();
  rotation2_.setIdentity();
  rotation1_.setIdentity();

  box_ = hpp::fcl::Box(0.01, 0.01, 3.);
  request_distance_ = hpp::fcl::DistanceRequest(false, 0., 0.);  // use distance function in hppfcl
  // request_collision =  hpp::fcl::CollisionRequest(hpp::fcl::DISTANCE_LOWER_BOUND, 1);
  // request_collision =  hpp::fcl::CollisionRequest(hpp::fcl::NO_REQUEST, 1);
  // request_collision.enable_distance_lower_bound = true;
}

Vector4 TerrainSlope::getSurfaceEq(MatrixN const& points) {
  Vector3 pt0 = points.row(0);
  Vector3 pt1 = points.row(1);
  Vector3 pt2 = points.row(2);
  Vector3 normal = (pt1 - pt0).cross(pt2 - pt0);
  if (normal.transpose() * Vector3(0., 0., 1.) < 0) {
    // In case the vertices are not sorted counterclock wise.
    normal = -normal;
  }
  Vector4 equation;
  equation.head(3) = normal / normal.norm();
  equation.tail(1) = -equation.head(3).transpose() * pt0;
  return equation;
}

double TerrainSlope::getHeight(double const& x, double const& y, Vector4 const& equation) {
  if (equation(2) != 0.) {
    return (1 / equation(2)) * (-equation(3) - equation(0) * x - equation(1) * y);
  } else {
    return 0.;
  }
}

VectorN TerrainSlope::getSlope(VectorN const& pose, MatrixN const& rotation_yaw,
                               std::vector<MatrixN> const& vecCollisionPts) {

  // Construct the convex objects from the surfaces equations.
  for (int j = 0; j < (int)vecCollisionPts.size(); j++) {
    std::vector<hpp::fcl::Vec3f> points;
    for (int i = 0; i < (int)vecCollisionPts[j].rows(); i++) {
      points.push_back(
          Vec3f(vecCollisionPts[j].row(i)[0], vecCollisionPts[j].row(i)[1], vecCollisionPts[j].row(i)[2]));
      // Increase artificially the shape of the surfaces to create a 3D convex hull. Fails otherwise. (volume
      // computation)
      points.push_back(
          Vec3f(vecCollisionPts[j].row(i)[0], vecCollisionPts[j].row(i)[1], vecCollisionPts[j].row(i)[2] + 0.002));
    }
    convexObjects_.push_back(hpp::fcl::ConvexBase::convexHull(points.data(), (int)points.size(), false, NULL));
    surfaceEqs_.push_back(getSurfaceEq(vecCollisionPts[j]));
  }

  VectorN xVector =
      VectorN::LinSpaced(fitSizeX_, - fitLength_,  fitLength_);
  VectorN yVector =
      VectorN::LinSpaced(fitSizeY_,- fitLength_, + fitLength_);

  int index = 0;
  // For each points, check which convex object is collide and evaluate the height.
  for (int i = 0; i < fitSizeX_; i++) {
    for (int j = 0; j < fitSizeY_; j++) {
      Vector3 position(xVector[i], yVector[j], 0.);
      position = rotation_yaw * position + pose;
      tf1_.setTransform(rotation1_, Vec3f(position(0), position(1), pose(2)));
      bool foundCollision = false;
      double z_max = 0.;
      double z_max_notFound = 0.;
      double d_min = 10;
      for (int k = 0; k < (int)convexObjects_.size(); k++) {
        hpp::fcl::distance(&box_, tf1_, convexObjects_[k], tf2_, request_distance_, res_distance_);
        // hpp::fcl::collide(&box, tf1, convexObjects_[0], tf2, request_collision, res_col);
        if (res_distance_.min_distance <= 0) {
          if (!foundCollision) {
            z_max = getHeight(position(0), position(1), surfaceEqs_[k]);
          } else {
            z_max = std::max(z_max, getHeight(position(0), position(1), surfaceEqs_[k]));
          }
          foundCollision = true;
        }
        else{
          if (res_distance_.min_distance < d_min){
            d_min = std::min(d_min, res_distance_.min_distance);
            z_max_notFound = getHeight(position(0), position(1), surfaceEqs_[k]);
          }
        }
        res_distance_.clear();
      }
      if (foundCollision) {
        A_.row(index) << position(0), position(1), 1.;
        b_(index) = z_max;
      } else {
        if (d_min != 10.){
          A_.row(index) << position(0), position(1), 1.;
          b_(index) = z_max_notFound;
        }
      }
      index++;
    }
  }
  // Solve the QP.
  qp_.reset(3, 0, 0);
  P_ = A_.transpose() * A_;
  q_ = -A_.transpose() * b_;
  status_ = qp_.solve_quadprog(P_, q_, C_, d_, G_, h_, fit_);

  // Delete pointers.
  for (int i = 0; i < (int)convexObjects_.size(); i++) {
    delete (convexObjects_[i]);
  }
  convexObjects_.clear();
  surfaceEqs_.clear();

  return fit_;
}
