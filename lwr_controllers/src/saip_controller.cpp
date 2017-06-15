#include <pluginlib/class_list_macros.h>

#include <lwr_controllers/saip_controller.h>

using namespace std;

namespace lwr_controllers
{

SAIPController::SAIPController() {}
SAIPController::~SAIPController() {}

bool SAIPController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
    if(!(KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n)))
    {
        ROS_ERROR("Couldn't initilize SAIPController.");
        return false;
    }

    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    id_solver_.reset(new KDL::ChainDynParam(kdl_chain_,gravity_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    torques_cmd_.resize(kdl_chain_.getNrOfJoints());
    J_.resize(kdl_chain_.getNrOfJoints());
    G_.resize(kdl_chain_.getNrOfJoints());
    Kp_.resize(kdl_chain_.getNrOfJoints());
    Kd_.resize(kdl_chain_.getNrOfJoints());

    sub_command_ = nh_.subscribe("command", 1, &SAIPController::command, this);

    ROS_DEBUG("Initialization for SAIPController complete.");

    return true;
}

void SAIPController::starting(const ros::Time& time)
{
    // get joint positions/velocities
    for(int i=0; i < joint_handles_.size(); i++)
    {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
        joint_des_states_.q(i) = joint_msr_states_.q(i);
        joint_des_states_.qdot(i) = joint_msr_states_.qdot(i);
        Kp_(i) = 500;
        Kd_(i) = 2;
    }

    cmd_flag_ = 0;

    fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

    jnt_to_jac_solver_->JntToJac(joint_msr_states_.q,J_);

    ROS_DEBUG("Starting for SAIPController complete.");
}

void SAIPController::update(const ros::Time& time, const ros::Duration& period)
{

    // get joint positions/velocities
    for(int i=0; i < joint_handles_.size(); i++)
    {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
        joint_des_states_.q(i) = joint_msr_states_.q(i);
        joint_des_states_.qdot(i) = joint_msr_states_.qdot(i);
    }

    // computing Inertia, Coriolis and Gravity matrices
    //id_solver_->JntToMass(joint_msr_states_.q, M_);
    //id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
    id_solver_->JntToGravity(joint_msr_states_.q, G_);

    // computing forward kinematics
    fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

    jnt_to_jac_solver_->JntToJac(joint_msr_states_.q,J_);

    if(!cmd_flag_)
    {
        x_des_ = x_;
    }

    // computing end-effector position/orientation error w.r.t. desired frame
    x_err_ = diff(x_,x_des_);

    x_dot_ = J_.data*joint_msr_states_.qdot.data;

    // setting error reference
    for(int i = 0; i < e_ref_.size(); i++)
    {
        // e = Kp*(x_des - x) + Kd*(- x_dot)
        e_ref_(i) =  Kp_(i)*x_err_(i) + Kd_(i)*(- x_dot_(i));
    }

    // finally, computing the torques
    torques_cmd_.data = J_.data.transpose()*e_ref_; //+ G_.data

    for(int i=0; i < joint_handles_.size(); i++)
    {
        joint_handles_[i].setCommand(torques_cmd_(i));
    }
}

void SAIPController::stopping(const ros::Time& time)
{
    /*out[0] = in.M.UnitX().x();
    out[1] = in.M.UnitY().x();
    out[2] = in.M.UnitZ().x();
    out[3] = in.p.x();
    out[4] = in.M.UnitX().y();
    out[5] = in.M.UnitY().y();
    out[6] = in.M.UnitZ().y();
    out[7] = in.p.y();
    out[8] = in.M.UnitX().z();
    out[9] = in.M.UnitY().z();
    out[10] = in.M.UnitZ().z();
    out[11] = in.p.z();*/
}

void SAIPController::command(const lwr_controllers::PoseRPY::ConstPtr &msg)
{
    KDL::Frame frame_des;

    switch(msg->id)
    {
        case 0:
        frame_des = KDL::Frame(
                KDL::Rotation::RPY(msg->orientation.roll,
                                  msg->orientation.pitch,
                                  msg->orientation.yaw),
                KDL::Vector(msg->position.x,
                            msg->position.y,
                            msg->position.z));
        break;

        case 1: // position only
        frame_des = KDL::Frame(
            KDL::Vector(msg->position.x,
                        msg->position.y,
                        msg->position.z));
        break;

        case 2: // orientation only
        frame_des = KDL::Frame(
            KDL::Rotation::RPY(msg->orientation.roll,
                               msg->orientation.pitch,
                               msg->orientation.yaw));
        break;

        default:
        ROS_INFO("Wrong message ID");
        return;
    }

    x_des_ = frame_des;
    cmd_flag_ = 1;
}

}

PLUGINLIB_EXPORT_CLASS( lwr_controllers::SAIPController, controller_interface::ControllerBase);
