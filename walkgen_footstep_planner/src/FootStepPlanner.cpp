#include "FootStepPlanner.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/math/rpy.hpp"
#include <pinocchio/math/quaternion.hpp>
#include "pinocchio/spatial/se3.hpp"

FootStepPlanner::FootStepPlanner(const pinocchio::Model &model, const Eigen::VectorXd &q,
                                 const Params &params, const double period) : params_(params),
                                                                              model_(model), period_(period)
{

    data_ = pinocchio::Data(model_);
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    filter_q = FilterMean(period_, static_cast<double>(params_.dt) * params_.nsteps);
    filter_v = FilterMean(period_, static_cast<double>(params_.dt) * params_.nsteps);
    initialize(q);
}

FootStepPlanner::FootStepPlanner(const pinocchio::Model &model, const Eigen::VectorXd &q, const double period) : params_(Params()),
                                                                                                                 model_(model), period_(period)
{

    data_ = pinocchio::Data(model_);
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    filter_q = FilterMean(period_, static_cast<double>(params_.dt) * params_.nsteps);
    filter_v = FilterMean(period_, static_cast<double>(params_.dt) * params_.nsteps);
    initialize(q);
}

void FootStepPlanner::initialize(const Eigen::VectorXd &q)
{
    // TODo, get the names from params file.
    const std::string lf = "LF_FOOT";
    const std::string lh = "LH_FOOT";
    const std::string rf = "RF_FOOT";
    const std::string rh = "RH_FOOT";

    contactNames_ = {lf, lh, rf, rh};
    contactNames_sl1m_ = {lf, rf, lh, rh};

    offsets_feet_[lf] = Vector3(0.367, 0.2, 0.);
    offsets_feet_[lh] = Vector3(-0.367, 0.2, 0.);
    offsets_feet_[rf] = Vector3(0.367, -0.2, 0.);
    offsets_feet_[rh] = Vector3(-0.367, -0.2, 0.);

    current_position_ = MatrixN::Zero(3, 4);
    current_velocities_ = Matrix34::Zero();
    target_fsteps_ = Matrix34::Zero();
    qf_ = VectorN::Zero(18);
    qvf_ = VectorN::Zero(18);

    nsteps_ = params_.nsteps;
    horizon_ = params_.horizon;
    dt_ = params_.dt;
    counter_gait_ = 0;

    double dx = 0.5;
    double dy = 0.5;
    double height = 0.;
    double epsilon = 10e-6;
    Eigen::Matrix<double, 6, 3> A;
    A << -1., 0., 0.,
        0., -1., 0.,
        0., 1., 0.,
        1., 0., 0.,
        0., 0., 1.,
        0., 0., -1.;
    Eigen::Matrix<double, 6, 1> b;
    b << dx - q(0), dy - q(1), dy + q(1), dx + q(0), height + epsilon, -height + epsilon;

    Eigen::Matrix<double, 4, 3> vertices;
    vertices << q[0] - dx, q[1] + dy, height,
        q[0] - dx, q[1] - dy, height,
        q[0] + dx, q[1] - dy, height,
        q[0] + dx, q[1] + dy, height;
    for (size_t foot = 0; foot < 4; ++foot)
    {
        previous_surfaces_[contactNames_[foot]] = Surface(MatrixN(A), b, MatrixN(vertices));
    }
};

// From python, it is easier to use and return only MatrixN, otherwise, a binding is necessary for each type.
MatrixN FootStepPlanner::compute_footstep(std::vector<std::shared_ptr<ContactSchedule>> &queue_cs,
                                          const VectorN &q,
                                          const VectorN &vq,
                                          const VectorN &bvref,
                                          const int timeline,
                                          const std::map<std::string, std::vector<Surface>> &selected_surfaces,
                                          const std::map<std::string, Surface> &previous_surfaces)
{
    if (q.size() != 19)
    {
        throw std::runtime_error("Current state q should be an array of size 19 [pos (x3), quaternion (x4), joint (x12)]");
    }
    if (vq.size() != 18)
    {
        throw std::runtime_error("Current velocity vq should be an array of size 18 [lin vel (x3), ang vel (x3), joint vel(x12)]");
    }
    if (bvref.size() != 6)
    {
        throw std::runtime_error("Reference velocity should be an array of size 6 [lin vel (x3), ang vel (x3)]");
    }

    // Update current feet position
    update_current_state(q, vq);

    // Filter quantities
    Vector3 rpy = pinocchio::rpy::matrixToRpy(pinocchio::SE3::Quaternion(q(6), q(3), q(4), q(5)).toRotationMatrix());
    Vector6 q_ = Vector6::Zero();
    q_.head<3>() = q.head<3>();
    q_.tail<3>() = rpy;
    qf_ = filter_q.filter(q_);

    qvf_ = filter_v.filter(vq.head<6>());

    MatrixN XX = MatrixN::Zero(3, 3);

    // Return placeholder matrix for demonstration purposes
    return update_position(queue_cs, qf_, qvf_, bvref, timeline, selected_surfaces);
}

MatrixN FootStepPlanner::update_position(std::vector<std::shared_ptr<ContactSchedule>> &queue_cs,
                                         const Vector6 &q,
                                         const Vector6 &vq,
                                         const VectorN &bvref,
                                         const int timeline_in,
                                         const std::map<std::string, std::vector<Surface>> &selected_surfaces)
{
    if (timeline_in == 0)
    {
        counter_gait_ += 1;
    }
    Matrix3 Rz = pinocchio::rpy::rpyToMatrix(double(0.), double(0.), q(5));
    // Matrix3 Rz = Matrix3::Identity();
    Matrix3 Rxy = pinocchio::rpy::rpyToMatrix(q(3), q(4), double(0.));

    Vector3 q_tmp = q.head<3>();
    q_tmp(2) = double(0.);

    Matrix34 P0 = Matrix34(current_position_);
    // Matrix34 V0 = Matrix34::Zero();
    Matrix34 V0 = Matrix34(current_velocities_);
    int timeline = timeline_in;
    int cs_index = int(0);
    std::vector<size_t> foot_timeline = {0, 0, 0, 0};
    MatrixN target_fsteps = MatrixN::Zero(3, 4);

    // Define tmp variables
    double dt_i = 0.;
    double dx = 0.;
    double dy = 0.;
    Matrix3 Rz_tmp = Matrix3::Identity();

    for (auto cs_iter = queue_cs.rbegin(); cs_iter != queue_cs.rend(); ++cs_iter)
    {
        auto &cs = **cs_iter; // Reference to ContactSchedule
        if (cs_index <= horizon_ + 2)
        {
            for (size_t c = 0; c < cs.contactNames_.size(); ++c)
            {
                auto &name = cs.contactNames_[c];
                size_t j = find_stdVec(cs.contactNames_, name); // Which foot in foot_timeline
                auto &phases = cs.phases_[c];
                if (phases.size() == 3)
                {
                    double T_stance = double(phases[0]->T_ + phases[2]->T_) * cs.dt_;
                    auto &active_phase = phases[0];
                    auto &inactive_phase = phases[1];

                    if (cs_index + active_phase->T_ - timeline <= horizon_ + 2)
                    {

                        // Displacement following the reference velocity compared to current position
                        if (active_phase->T_ + inactive_phase->T_ - timeline > 0)
                        { // case 1 and 2
                            if (std::abs(bvref(5)) > 0.01)
                            {
                                dt_i = static_cast<double>(cs_index + active_phase->T_ + inactive_phase->T_ - timeline) * cs.dt_; // dt integration dt_i
                                dx = (bvref(0) * std::sin(bvref(5) * dt_i) + bvref(1) * (std::cos(bvref(5) * dt_i) - 1.0)) / bvref(5);
                                dy = (bvref(1) * std::sin(bvref(5) * dt_i) - bvref(0) * (std::cos(bvref(5) * dt_i) - 1.0)) / bvref(5);
                                dt_i = static_cast<double>(cs_index + active_phase->T_ + inactive_phase->T_) * cs.dt_;
                                Rz_tmp = pinocchio::rpy::rpyToMatrix(double(0.), double(0.), bvref(5) * dt_i);
                            }
                            else
                            {
                                dt_i = static_cast<double>(cs_index + active_phase->T_ + inactive_phase->T_ - timeline) * cs.dt_;
                                dx = bvref(0) * dt_i;
                                dy = bvref(1) * dt_i;
                                Rz_tmp = Matrix3::Identity();
                            }
                            Vector3 q_dxdy;
                            q_dxdy << dx, dy, double(0.);
                            Vector3 heuristic = compute_heuristic(vq, bvref, Rxy, T_stance, name, false);
                            Vector3 footstep = Rz * (Rz_tmp * heuristic) + q_tmp + Rz * q_dxdy;

                            Matrix3 P_ = Matrix3::Identity();
                            Vector3 q_ = Vector3::Zero();
                            auto iter = selected_surfaces.find(name);
                            if (iter == selected_surfaces.end())
                            {
                                throw std::runtime_error("Naming non consistent in surface dictionnary.");
                            }
                            auto &sf_ = (iter->second).at(foot_timeline.at(j));
                            MatrixN G_ = -sf_.getA();                        // Inverse sign in python
                            VectorN h_ = sf_.getb() - sf_.getA() * footstep; // Inverse sign in python

                            MatrixN C_ = MatrixN::Zero(3, 0);
                            MatrixN d_ = VectorN::Zero(0);
                            VectorN delta_x = VectorN::Zero(3);

                            // In python code
                            // delta_x = quadprog_solve_qp(P_, q_, G_, h_)
                            // min (1/2)x' P x + q' x
                            // subject to  G x <= h
                            // subject to  C x  = d

                            // Eiquadprog-Fast solves the problem :
                            // min. 1/2 * x' P_ x + q_' x
                            // s.t. C_ x + d_ = 0
                            //      G_ x + h_ >= 0
                            status = qp.solve_quadprog(P_, q_, C_, d_, G_, h_, delta_x);
                            Vector3 fsteps_optim = footstep + delta_x;

                            // Update target fsteps for sl1m
                            if (foot_timeline.at(j) == 0)
                            {
                                size_t jj = find_stdVec(contactNames_sl1m_, name);
                                target_fsteps_.col(static_cast<Eigen::Index>(jj)) = fsteps_optim;
                            }

                            // Update previous surface mechanism
                            Surface previous_sf = sf_;
                            if (foot_timeline.at(j) == 0)
                            {
                                auto iter = previous_surfaces_.find(name);
                                if (iter == previous_surfaces_.end())
                                {
                                    throw std::runtime_error("Naming non consistent in previous_surface dictionnary.");
                                }
                                previous_sf = Surface(iter->second); // make a copy
                            }
                            else
                            {
                                auto iter = selected_surfaces.find(name);
                                if (iter == selected_surfaces.end())
                                {
                                    throw std::runtime_error("Naming non consistent in selected_surface dictionnary.");
                                }
                                previous_sf = Surface((iter->second).at(foot_timeline.at(j) - 1));
                            }
                            double t0 = double(0.);

                            // Update trajectory
                            if (active_phase->T_ - timeline >= 0)
                            {
                                V0.col(static_cast<Eigen::Index>(j)).setZero();
                            }
                            else
                            {
                                t0 = static_cast<double>(timeline - active_phase->T_);
                            }
                            if (counter_gait_ < 2)
                            {
                                // Letting 1/2 gait for the estimator at first since it is a moving average estimator.
                                // Hence, keeping initial position.
                                fsteps_optim.head<2>() = P0.col(static_cast<Eigen::Index>(j)).head<2>();
                            }
                            if (t0 <= static_cast<double>(inactive_phase->T_) * 0.7)
                            {
                                inactive_phase->trajectory_->update(P0.col(static_cast<Eigen::Index>(j)), V0.col(static_cast<Eigen::Index>(j)),
                                                                    fsteps_optim, t0 * cs.dt_, sf_, previous_sf);
                            }

                            // End of the flying phase, register the surface.
                            if (t0 >= static_cast<double>(inactive_phase->T_ - nsteps_))
                            {
                                // previous_surfaces_ .pop(name)
                                auto iter = previous_surfaces_.find(name);
                                if (iter == previous_surfaces_.end())
                                {
                                    throw std::runtime_error("Naming non consistent in previous_surface dictionnary.");
                                }
                                iter->second = Surface(sf_); // make a copy.
                            }
                            P0.col(static_cast<Eigen::Index>(j)) = fsteps_optim;
                            V0.col(static_cast<Eigen::Index>(j)).setZero();

                            foot_timeline.at(j) += 1;
                        }
                        else
                        { // case 3
                            V0.col(static_cast<Eigen::Index>(j)).setZero();
                        }
                    }
                }
                else
                {
                    throw std::runtime_error("Error in fstep planner. Only 3 phases per ContactSchedule considered.");
                }
            }
        }
        else
        {
            break;
        }
        cs_index += cs.T_ - timeline;
        timeline = 0;
    }

    return target_fsteps_;
}

void FootStepPlanner::update_current_state(const Eigen::VectorXd &q, const Eigen::VectorXd &vq)
{
    if (q.size() != 19)
    {
        throw std::runtime_error("Current state q should be an array of size 19 [pos (x3), quaternion (x4), joint (x12)]");
    }
    if (vq.size() != 18)
    {
        throw std::runtime_error("Current velocity vq should be an array of size 18 [lin vel (x3), ang vel (x3), joint vel(x12)]");
    }
    pinocchio::forwardKinematics(model_, data_, q, vq);
    for (size_t i = 0; i < contactNames_.size(); i++)
    {
        size_t frame_id = model_.getFrameId(contactNames_[i]);
        pinocchio::SE3 oMf = pinocchio::updateFramePlacement(model_, data_, frame_id);
        pinocchio::Motion v = pinocchio::getFrameVelocity(model_, data_, frame_id);
        current_position_.col(static_cast<Eigen::Index>(i)) = oMf.translation();
        current_velocities_.col(static_cast<Eigen::Index>(i)) = v.linear();
    }
}

Vector3 FootStepPlanner::compute_heuristic(const Eigen::VectorXd &bv,
                                           const Eigen::VectorXd &bvref,
                                           const Matrix3 &Rxy,
                                           const double T_stance,
                                           const std::string &name,
                                           const bool feedback_term)
{
    Vector3 footstep;
    footstep.setZero();

    double beta = 1.35;

    // Add symmetry term
    footstep += beta * T_stance * bvref.head<3>();

    // Add feedback term
    if (feedback_term)
    {
        footstep += k_feedback_ * bv.head<3>();
        footstep += -k_feedback_ * bvref.head<3>();
    }

    // Add centrifugal term
    Vector3 cross;
    cross << bv(1) * bvref(5) - bv(2) * bvref(4),
        bv(2) * bvref(3) - bv(0) * bvref(5),
        0.0;
    footstep += 0.5 * std::sqrt(href_ / g_) * cross;

    // Limit deviation
    footstep(0) = std::min(footstep(0), L_);
    footstep(0) = std::max(footstep(0), -L_);
    footstep(1) = std::min(footstep(1), L_);
    footstep(1) = std::max(footstep(1), -L_);

    // Add shoulders, Yaw axis taken into account later
    // size_t j = find_stdVec(contactNames_,name);
    footstep += Rxy * offsets_feet_.at(name);

    // Remove Z component (working on flat ground)
    footstep(2) = 0.0;

    return footstep;
}

// find element inside vector list.
template <typename Type>
size_t FootStepPlanner::find_stdVec(const std::vector<Type> &vec, const Type &elem)
{
    auto it = std::find(vec.begin(), vec.end(), elem);
    if (it != vec.end())
    {
        return static_cast<size_t>(std::distance(vec.begin(), it));
    }
    else
    {
        throw std::runtime_error("Element not found in vector");
    }
}
