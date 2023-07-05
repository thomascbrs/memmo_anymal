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
#include <boost/smart_ptr.hpp>
#include <hpp/affordance/affordance-extraction.hh>
#include <hpp/affordance/operations.hh>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/data_types.h>
#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/mesh_loader/loader.h>

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
  /// \brief Constructor with hpp-affordance parameters
  ///
  /// \param[in] margin_in double
  /// \param[in] nbTriMargin_in double.
  /// \param[in] minArea_in double.
  /// \param[in] affordanceName char. type of affordance
  ///
  ///////////////////////////////////////////////////////////////////////////
  AffordanceLoader(double const &margin_in, double const &nbTriMargin_in, double const &minArea_in,
                   char *const &affordanceName_in);

  ///////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Extract the affordances from .stl file.
  ///
  /// \param[in] filename .stl file.
  /// \param[in] R Rotation matrix.
  /// \param[in] T Translation vector.
  ///
  ///////////////////////////////////////////////////////////////////////////
  void load(std::string const &filename, MatrixN const &R, VectorN const &T);

  ///////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Return the affordances extracted.
  ///
  ///////////////////////////////////////////////////////////////////////////
  StdVecVec_Vector3 getAffordances() const { return affordances_; };

 private:
  StdVecVec_Vector3 affordances_;
  // Parameters to select the relevant surfaces
  double margin;
  double nbTriMargin;
  double minArea;
  char *affordanceName;
};

#endif  // AffordanceLoader_H_INCLUDED
