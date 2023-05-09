#ifndef CONTACTPHASE_H
#define CONTACTPHASE_H

#include <FootTrajectoryWrapper.hpp>
#include <memory>

enum class ContactType {
  POINT, // 3D
  FULL   // 6D
};

class ContactPhase {
public:
  // Constructor with number of nodes
  ContactPhase(int T);

  // Constructor with number of nodes and contact type
  ContactPhase(int T, ContactType contactType);

  // Constructor with number of nodes and trajectory
  ContactPhase(int T, std::shared_ptr<FootTrajectoryWrapper> trajectory);

  // Constructor with number of nodes, contact type, and trajectory
  ContactPhase(int T, ContactType contactType,
               std::shared_ptr<FootTrajectoryWrapper> trajectory);

  // Copy constructor
  ContactPhase(const ContactPhase &other);

  // Getter for number of nodes
  int getT() const;

  // Getter for contact type
  ContactType getContactType() const;

  // Getter for trajectory
  std::shared_ptr<FootTrajectoryWrapper> &getTrajectory();

  // Setter for trajectory
  void setTrajectory(std::shared_ptr<FootTrajectoryWrapper> trajectory);

  // Operator overload to add two contact phases together
  std::shared_ptr<ContactPhase> operator+(const ContactPhase &phase) const;

  // Operator ==, necessary to register a list
  bool operator==(const ContactPhase &other) const {
    return T_ == other.T_ && contactType_ == other.contactType_ &&
           trajectory_ == other.trajectory_;
  }

  int T_;                   // Number of nodes
  ContactType contactType_; // Type of contact
  std::shared_ptr<FootTrajectoryWrapper>
      trajectory_; // Task-space trajectory during contact phase
};

#endif // CONTACTPHASE_H
