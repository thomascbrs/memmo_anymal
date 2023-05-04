#ifndef FOOTSTEPPLANNER_H
#define FOOTSTEPPLANNER_H

#include <Eigen/Dense>
#include <map>
#include <string>
#include <vector>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <Surface.hpp>
#include <Params.hpp>
#include <Types.hpp>
#include <Filter.hpp>
#include <Surface.hpp>
#include <ContactSchedule.hpp>

#include "eiquadprog/eiquadprog-fast.hpp"

class FootStepPlanner
{
public:
    // Constructor with Params
    FootStepPlanner(const pinocchio::Model &model, const Eigen::VectorXd &q,
                    const Params &params,const double period = 0.5);
    // Constructor without Params
    FootStepPlanner(const pinocchio::Model &model, const Eigen::VectorXd &q,const double period = 0.5);
    void initialize(const Eigen::VectorXd &q);

    MatrixN compute_footstep(std::vector<std::shared_ptr<ContactSchedule>>& queue_cs,
                           const VectorN& q,
                           const VectorN& vq,
                           const VectorN& bvref,
                           const int timeline,
                           const std::map<std::string,std::vector<Surface>>& selected_surfaces,
                           const std::map<std::string,Surface>& previous_surfaces);
    
    VectorN get_qf(){return qf_;};
    VectorN get_qvf(){return qvf_;};
    std::vector<std::string> get_contact_name_sl1m(){return contactNames_sl1m_;};
    std::vector<std::string> get_contact_name(){return contactNames_;};
    MatrixN get_current_position(){return current_position_;};

private:
    // void initFeetPositions(const Eigen::VectorXd& q);
    // void updateFeetPositions(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);
    // void updatePreviousSurfaces(const std::vector<Eigen::VectorXd>& footstep_plan);
    // void getSurfacesAroundFeet(std::map<std::string, walkgen::Surface>& surfaces) const;
    // Eigen::MatrixXd getTrajectoryFromFootsteps(const std::vector<Eigen::VectorXd>& footstep_plan) const;
    // Eigen::VectorXd createFootstep(const Eigen::VectorXd& current_foot_position,
    //                                const Eigen::VectorXd& target_foot_position,
    //                                double phase) const;
    // Eigen::VectorXd getFootstepPlan(const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
    //                                 const std::vector<Eigen::VectorXd>& previous_footstep_plan,
    //                                 std::map<std::string, walkgen::Surface>& surfaces) const;

    void update_current_state(const Eigen::VectorXd& q, const Eigen::VectorXd& vq);
    Vector3 compute_heuristic(const Eigen::VectorXd& bv,
                        const Eigen::VectorXd& bvref,
                        const Matrix3& Rxy,
                        const double T_stance,
                        const std::string& name,
                        const bool feedback_term = true);
    MatrixN update_position(std::vector<std::shared_ptr<ContactSchedule>> &queue_cs, 
                        const Vector6 &q, 
                        const Vector6 &vq, 
                        const VectorN &bvref, 
                        const int timeline, 
                        const std::map<std::string,std::vector<Surface>>& selected_surfaces);
    template <typename Type>
    size_t find_stdVec(const std::vector<Type>& vec, const Type& elem) ;

    Params params_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    FilterMean filter_q;
    FilterMean filter_v;

    std::vector<std::string> contactNames_;
    std::vector<std::string> contactNames_sl1m_;
    std::map<std::string, Vector3> offsets_feet_;
    MatrixN current_position_;
    Matrix34 current_velocities_;
    Matrix34 target_fsteps_;
    VectorN qf_;
    VectorN qvf_;
    std::map<std::string, Surface> previous_surfaces_;

    double period_;
    // Constant
    const double k_feedback_ = 0.03;
    const double href_ = 0.48;
    const double g_ = 9.81;
    const double L_ = 0.5;
    
    int nsteps_;
    int horizon_;
    double dt_;
    int counter_gait_;

    // QP solver
    EiquadprogFast_status expected = EIQUADPROG_FAST_OPTIMAL;
    EiquadprogFast_status status;
    EiquadprogFast qp;
    
    // int m_nsteps;
    // double m_stop_heuristic;
    // int m_horizon;

    // bool m_debug;
    // bool m_RECORDING;
    // std::vector<Eigen::VectorXd> m_footstep_plan;

    // double m_dt;
    // walkgen::FilterMean m_q_filter;
    // Eigen::VectorXd m_q_f;
    // walkgen::FilterMean m_qv_filter;
    // Eigen::VectorXd m_qv_f;

    // std::map<std::string, walkgen::Surface> m_previous_surfaces;
};

#endif // FOOTSTEPPLANNER_H
