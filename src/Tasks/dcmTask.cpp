// associated header
#include "../../include/Tasks/dcmTask.h"
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


namespace tasks
{

namespace qp
{

dcmTask::dcmTask(const std::vector<rbd::MultiBody> & mbs, int rI)
{
  jac_ = Eigen::MatrixXd::Zero(3,mbs[rI].nrDof());
  normalAcc_ = Eigen::Vector3d::Zero();
  eval_ = Eigen::Vector3d::Zero();
  speed_ = Eigen::Vector3d::Zero();
}

dcmTask::dcmTask(const std::vector<rbd::MultiBody> & mbs,
                 int rI,
                 std::vector<double> weight)
: robotIndex_(rI)
{
  jac_ = Eigen::MatrixXd::Zero(3,mbs[rI].nrDof());
  normalAcc_ = Eigen::Vector3d::Zero();
  eval_ = Eigen::Vector3d::Zero();
  speed_ = Eigen::Vector3d::Zero();

}



int dcmTask::dim()
{
  return 3;
}

void dcmTask::update(const std::vector<rbd::MultiBody> & mbs,
                     const std::vector<rbd::MultiBodyConfig> & mbcs,
                     const SolverData & data)
{
  
}

const Eigen::MatrixXd & dcmTask::jac() const
{
  return jac_;
}

const Eigen::VectorXd & dcmTask::eval() const
{
  return eval_;
}

const Eigen::VectorXd & dcmTask::speed() const
{
  return speed_;
}

const Eigen::VectorXd & dcmTask::normalAcc() const
{
  return normalAcc_;
}

} //namespace qp

} // namespace tasks
