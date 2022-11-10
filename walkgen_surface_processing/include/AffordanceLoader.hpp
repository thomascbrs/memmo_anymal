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
#include <hpp/fcl/mesh_loader/loader.h>
#include <boost/smart_ptr.hpp>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/data_types.h>
#include <hpp/fcl/fwd.hh>
#include <hpp/affordance/affordance-extraction.hh>
#include <hpp/affordance/operations.hh>


typedef std::vector<std::vector<Vector3>> StdVecVec_Vector3;

using namespace hpp;
class AffordanceLoader {
 public:
  // Constructor
  AffordanceLoader();

  // Destructor
  ~AffordanceLoader() {}

  ///////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Extract the affordances from .stl file.
  ///
  /// \param[in] filename .stl file.
  /// \param[in] R Rotation matrix.
  /// \param[in] T Translation vector.
  ///
  ///////////////////////////////////////////////////////////////////////////
  void load(std::string const& filename, MatrixN const& R, VectorN const& T);

  ///////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Return the affordances extracted.
  ///
  ///////////////////////////////////////////////////////////////////////////
  StdVecVec_Vector3 getAffordances() const { return affordances_; };

 private:
  StdVecVec_Vector3 affordances_;
  // Parameters to select the relevant surfaces
  const double margin = 0.03;
  const double nbTriMargin = 0.03;
  const double minArea = 0.005;
  const char* affordanceName = "Support";
};

#endif  // AffordanceLoader_H_INCLUDED
