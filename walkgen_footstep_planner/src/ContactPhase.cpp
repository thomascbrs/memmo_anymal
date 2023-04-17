#include "ContactPhase.hpp"

// Constructor with number of nodes
ContactPhase::ContactPhase(int T) :
    T_(T),
    contactType_(ContactType::POINT),
    trajectory_(nullptr)
{}

// Constructor with number of nodes and contact type
ContactPhase::ContactPhase(int T, ContactType contactType) :
    T_(T),
    contactType_(contactType),
    trajectory_(nullptr)
{}

// Constructor with number of nodes and trajectory
ContactPhase::ContactPhase(int T, std::shared_ptr<FootTrajectoryWrapper> trajectory) :
    T_(T),
    contactType_(ContactType::POINT),
    trajectory_(std::move(trajectory))
{}

// Constructor with number of nodes, contact type, and trajectory
ContactPhase::ContactPhase(int T, ContactType contactType, std::shared_ptr<FootTrajectoryWrapper> trajectory) :
    T_(T),
    contactType_(contactType),
    trajectory_(std::move(trajectory))
{}

// Getter for number of nodes
int ContactPhase::getT() const {
    return T_;
}

// Getter for contact type
ContactType ContactPhase::getContactType() const {
    return contactType_;
}

// Getter for trajectory
std::shared_ptr<FootTrajectoryWrapper>& ContactPhase::getTrajectory() {
    return trajectory_;
}

// Setter for trajectory
void ContactPhase::setTrajectory(std::shared_ptr<FootTrajectoryWrapper> trajectory){
    this->trajectory_ = std::move(trajectory);
} 

// Operator overload to add two contact phases together
std::shared_ptr<ContactPhase> ContactPhase::operator+(const ContactPhase& phase) const {
    if (contactType_ != phase.contactType_) {
        throw std::invalid_argument("Couldn't append the contact phase since it doesn't belong to the same type");
    }

    return std::make_shared<ContactPhase>(T_ + phase.T_, contactType_, std::move(trajectory_));
}
