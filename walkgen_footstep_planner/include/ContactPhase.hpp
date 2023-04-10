#ifndef CONTACTPHASE_H
#define CONTACTPHASE_H

#include <memory>
#include <FootTrajectoryBezier.hpp>

enum class ContactType {
    POINT, // 3D
    FULL // 6D
};

class ContactPhase {
public:
    // Constructor with number of nodes and contact type
    ContactPhase(int T, ContactType contactType);

    // Constructor with number of nodes, contact type, and trajectory
    ContactPhase(int T, ContactType contactType, std::shared_ptr<FootTrajectoryBezier> trajectory);

    // Getter for number of nodes
    int getT() const;

    // Getter for contact type
    ContactType getContactType() const;

    // Getter for trajectory
    std::shared_ptr<FootTrajectoryBezier>& getTrajectory();

    // Setter for trajectory
    void setTrajectory(std::shared_ptr<FootTrajectoryBezier> trajectory);

    // Operator overload to add two contact phases together
    std::shared_ptr<ContactPhase> operator+(const ContactPhase& phase) const;

    int T_; // Number of nodes
    ContactType contactType_; // Type of contact
    std::shared_ptr<FootTrajectoryBezier> trajectory_; // Task-space trajectory during contact phase
};

#endif // CONTACTPHASE_H
