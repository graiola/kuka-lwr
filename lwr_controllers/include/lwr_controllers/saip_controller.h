#ifndef SAIP_CONTROLLER_H
#define SAIP_CONTROLLER_H

////////// 20sim controller
//#include <twenty_sim_code/Controller.h>

//typedef Controller Cnt;

////////// kuka-lwr
#include "KinematicChainControllerBase.h"
#include <lwr_controllers/PoseRPY.h>

namespace lwr_controllers
{

class SAIPController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
{
public:
  SAIPController();
  ~SAIPController();

  bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time);
  void command(const lwr_controllers::PoseRPY::ConstPtr &msg);

private:

  int cmd_flag_;
  ros::Subscriber sub_command_;
  KDL::Frame x_;		//current pose
  KDL::Frame x_des_;	//desired pose
  KDL::JntArray q_, qdot_; //current joints position and velocity
  KDL::JntArray torques_cmd_; // computed torques
  KDL::JntArray G_;     //gravity
  KDL::Jacobian J_;	//Jacobian
  KDL::JntArray Kp_,Kd_; //gains
  KDL::Twist x_err_;  //error
  Eigen::Matrix<double,6,1> x_dot_;
  Eigen::Matrix<double,6,1> e_ref_;

  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
  boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
  boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;


  // 20sim Controller interface
  /*Cnt cnt_;
  XXDouble u_ [30 + 1]; // 0-15 H matrix (Cartesian 4x4); 16-22 joints velocities; 23-29 joints position
  XXDouble y_ [7 + 1]; // 0-6 torques*/
};

}

#endif
