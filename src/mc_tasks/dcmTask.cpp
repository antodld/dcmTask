// associated header
#include "../../include/mc_tasks/dcmTask.h"
// includes
// std
#include <cmath>
#include <iterator>
#include <set>

// Eigen
#include <Eigen/Geometry>

// RBDyn
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>

#include <mc_solver/TasksQPSolver.h>
#include <mc_tasks/MetaTaskLoader.h>  
#include <mc_rbdyn/configuration_io.h>

#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/Point3D.h>




namespace mc_tasks
{

static inline mc_rtc::void_ptr_caster<tasks::qp::dcmTask> tasks_error{};

dcmTask::dcmTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: TrajectoryTaskGeneric(robots, robotIndex, stiffness, weight), rIndex_(robotIndex)
{
  switch(backend_)
  {
    case Backend::Tasks:
      finalize<Backend::Tasks, tasks::qp::dcmTask>(robots.mbs(), static_cast<int>(robotIndex));
      break;
    default:
      mc_rtc::log::error_and_throw("[dcmTask] Not implemented for solver backend: {}", backend_);
  }
  dcm_ = robots.robot(rIndex_).com() + (robots.robot(rIndex_).comVelocity()/omega_);

  type_ = "dcm";
  name_ = "dcm_" + robots.robot(rIndex_).name();
  reset();
}

void dcmTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  TrajectoryBase::load(solver, config);
  if(config.has("target")){dcmTarget_ = config("target");};
  omega_ = config("omega",3);

}

void dcmTask::update(mc_solver::QPSolver & s)
{
    const mc_rbdyn::Robot & robot = s.robots().at(rIndex_);
    dcm_ = robot.com() + (robot.comVelocity()/omega_);
    const auto & mb = robot.mb();
    const auto & mbc = robot.mbc();
    Eigen::MatrixXd J;
    Eigen::MatrixXd Jdot;

    auto JcomAlgo = rbd::CoMJacobian(mb);
    const auto Jcom = JcomAlgo.jacobian(mb,mbc);
    J = Jcom/omega_;
    Jdot = (JcomAlgo.jacobianDot(mb,mbc)/omega_) + Jcom;
    eval_ = dcmTarget_ - dcm_;

    const auto qdot = rbd::dofToVector(mb,mbc.alpha);
    
    tasks_error(errorT)->jac(J);

    tasks_error(errorT)->normalAcc(Jdot * qdot);
    
    tasks_error(errorT)->eval(eval_);

}

void dcmTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_dcm_target", this, [this]() -> const Eigen::Vector3d & { return dcmTarget_; });
  logger.addLogEntry(name_ + "_dcm_robot", this, [this]() -> const Eigen::Vector3d & { return dcm_; });
}

void dcmTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryBase::addToGUI(gui);
  gui.addElement({"Tasks", name_, "Gains"},
                 mc_rtc::gui::NumberInput(
                     "omega", [this]() { return this->omega(); }, [this](const double & w) { this->omega(w); }),
                mc_rtc::gui::Point3D("dcm",[this]()->const Eigen::Vector3d & {return dcm_;}),
                mc_rtc::gui::Point3D("dcm target",[this]()->const Eigen::Vector3d & {return dcmTarget_;},[this](const Eigen::Vector3d & c){dcmTarget(c);})
                                                                    );

}

void dcmTask::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryBase::removeFromGUI(gui);
  gui.removeCategory({"Task",name_});
}


} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "dcm",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      const std::string robot_name = config("robot",solver.robots().robot().name());

      const int r = solver.robots().robot(robot_name).robotIndex();
      auto t = std::make_shared<mc_tasks::dcmTask>(solver.robots(),r);
      t->reset();
      t->load(solver, config);
      return t;
    });
}