#pragma once

// includes
//
#include <array>

#include <tasks/config.hh>

// Eigen
#include <eigen3/Eigen/Core>


#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include "../../include/Tasks/dcmTask.h"
#include <mc_tasks/TrajectoryTaskGeneric.h>
#include <Tasks/Tasks.h>

namespace mc_tasks
{

struct MC_TASKS_DLLAPI dcmTask : public TrajectoryTaskGeneric
{

public:

    dcmTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness = 5.0, double weight = 100.);

    void reset() override
    {
        dcmTarget_ = dcm_;
    }

    /*! \brief Load parameters from a Configuration object */
    void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

 
    double omega() const
    {
        return omega_;
    }
    void omega(const double w)
    {
        omega_ = w;
    }

    void dcmTarget(const Eigen::Vector3d & d)
    {
        dcmTarget_ = d;
    }

    void update(mc_solver::QPSolver & s) override;




protected:

    void addToGUI(mc_rtc::gui::StateBuilder &) override;

    void removeFromGUI(mc_rtc::gui::StateBuilder &) override;

    void addToLogger(mc_rtc::Logger & logger) override;


private:

    size_t rIndex_;

    /** True if added to solver */
    bool inSolver_ = false;
    /** Robot handled by the task */

    double weight_ = 0;

    double omega_ = 3;

    Eigen::Vector3d dcmTarget_;
    Eigen::Vector3d dcm_;

    Eigen::Vector3d speed_ = Eigen::Vector3d::Zero();
 
    /** Store the previous eval vector */
    Eigen::VectorXd eval_;

};

using dcmTaskPtr = std::shared_ptr<dcmTask>;


} //namespace tasks