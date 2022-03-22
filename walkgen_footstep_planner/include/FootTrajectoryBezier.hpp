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

  void print_ok(int const &k);
};

#endif // TRAJGEN_H_INCLUDED
