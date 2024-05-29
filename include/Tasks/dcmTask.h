#pragma once

// includes
//
#include <array>

#include <tasks/config.hh>

// Eigen
#include <eigen3/Eigen/Core>

#include <mc_tasks/MetaTask.h>
#include <Tasks/QPSolver.h>
#include <Tasks/QPSolverData.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <sch/S_Object/S_Sphere.h>
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Polyhedron/S_Polyhedron.h>
#include <sch/CD/CD_Pair.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <Tasks/Tasks.h>


namespace tasks
{

namespace qp
{

class TASKS_DLLAPI dcmTask : public HighLevelTask
{
public:
  dcmTask(const std::vector<rbd::MultiBody> & mbs, int robotIndex);
  dcmTask(const std::vector<rbd::MultiBody> & mbs,
          int robotIndex,
          std::vector<double> weight);

  void jac(const Eigen::MatrixXd & J)
  {
    jac_ = J;
  }


  void normalAcc(const Eigen::Vector3d & v)
  {
    normalAcc_ = v;
  }
  void eval(const Eigen::Vector3d & e)
  {
    eval_ = e;
  }

  virtual int dim() override;
  virtual void update(const std::vector<rbd::MultiBody> & mbs,
                      const std::vector<rbd::MultiBodyConfig> & mbcs,
                      const SolverData & data) override;

  virtual const Eigen::MatrixXd & jac() const override;
  virtual const Eigen::VectorXd & eval() const override;
  virtual const Eigen::VectorXd & speed() const override;
  virtual const Eigen::VectorXd & normalAcc() const override;

private:

  Eigen::MatrixXd jac_;
  Eigen::VectorXd normalAcc_;
  Eigen::VectorXd eval_;
  Eigen::VectorXd speed_;

  int robotIndex_;

};
} //namespace qp

} // namespace tasks
