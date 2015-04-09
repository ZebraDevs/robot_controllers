/*
 * Copyright 2015 Fetch Robotics Inc
 * Author: Michael Ferguson
 */

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <robot_controllers_interface/controller.h>
#include <robot_controllers_interface/controller_manager.h>
#include <robot_controllers_interface/joint_handle.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <geometry_msgs/Wrench.h>

namespace robot_controllers
{

/*
 * Loosely based on
 *  http://answers.ros.org/question/59187/measuring-force-of-tip-using-jacobian-function-in-kdl/
 *  http://wiki.ros.org/pr2_mechanism/Tutorials/Coding%20a%20realtime%20Cartesian%20controller%20with%20KDL
 */
class ForceEstimation : public Controller
{
public:
  ForceEstimation() : initialized_(false) {}
  virtual ~ForceEstimation() {}

  /** @brief Initialize parameters, interfaces */
  virtual int init(ros::NodeHandle& nh, ControllerManager* manager)
  {
    Controller::init(nh, manager);
    manager_ = manager;

    // Load URDF
    urdf::Model model;
    if (!model.initParam("robot_description"))
    {
      ROS_ERROR("Failed to parse URDF");
      return -1;
    }

    // Load the tree
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
    {
      ROS_ERROR("Could not construct tree from URDF");
      return -1;
    }

    // Populate the Chain
    std::string root, tip;
    nh.param<std::string>("root", root, "torso_lift_link");
    nh.param<std::string>("tip", tip, "wrist_roll_link");
    if(!kdl_tree.getChain(root, tip, kdl_chain_))
    {
      ROS_ERROR("Could not construct chain from URDF");
      return -1;
    }

    kdl_chain_dynamics_.reset(new KDL::ChainDynParam(kdl_chain_, KDL::Vector(0,0,-9.81)));
    kdl_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

    // Init positions
    positions_ = KDL::JntArray(kdl_chain_.getNrOfJoints());
    //KDL::SetToZero(positions_.q);
    //KDL::SetToZero(positions_.qdot);

    // Init Joint Handles
    joints_.clear();
    for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
      if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
        joints_.push_back(manager_->getJointHandle(kdl_chain_.getSegment(i).getJoint().getName()));

    initialized_ = true;

    wrench_pub_ = nh.advertise<geometry_msgs::Wrench>("estimated_wrench", 10);

    // Should we autostart?
    bool autostart;
    nh.param("autostart", autostart, true);
    if (autostart)
      manager->requestStart(getName());

    return 0;
  }

  /** @brief Start the controller. */
  virtual bool start()
  {
    if (!initialized_)
      return false;
    return true;
  }

  /**
   *  @brief Stop this controller.
   *  @param force If true, this controller will be stopped regardless
   *         of return value.
   *  @returns true if controller preempted successfully.
   */
  virtual bool stop(bool force)
  {
    // always allow preemption
    return true;
  }

  /** @brief Update controller, called from controller_manager update */
  virtual void update(const ros::Time& time, const ros::Duration& dt)
  {
    // Need to initialize KDL structs
    if (!initialized_)
    return;

    // Get current positions
    for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
      positions_.data[i] = joints_[i]->getPosition();

    // Do the gravity compensation
    KDL::JntArray torques(kdl_chain_.getNrOfJoints());
    kdl_chain_dynamics_->JntToGravity(positions_, torques);

    KDL::Jacobian J;
    J.resize(kdl_chain_.getNrOfJoints());
    kdl_jac_solver_->JntToJac(positions_, J);

    // Compute cartesian effort
    KDL::Wrench F;
    //std::cout << "Force: ";
    for (size_t i = 0; i < 6; i ++)
    {
      F(i) = 0;
      for (size_t j = 0; j < kdl_chain_.getNrOfJoints(); j++)
      {
        F(i) += J(i, j) * (joints_[j]->getEffort() - torques.data[j]);
      }
      //std::cout << F(i) << " ";
    }
    //std::cout << std::endl;

    geometry_msgs::Wrench w;
    w.force.x = F(0);
    w.force.y = F(1);
    w.force.z = F(2);
    w.torque.x = F(3);
    w.torque.y = F(4);
    w.torque.z = F(5);
    wrench_pub_.publish(w);
  }

  /** @brief Get the type of this controller. */
  virtual std::string getType()
  {
    return "robot_controllers/ForceEstimation";
  }

  /** @brief Get the names of joints/controllers which this controller commands. */
  virtual std::vector<std::string> getCommandedNames()
  {
    std::vector<std::string> names;
    /*NOT ACTUALLY COMMANDING
    if (initialized_)
    {
      for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
        if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
          names.push_back(kdl_chain_.getSegment(i).getJoint().getName());
    }*/
    return names;
  }

  /** @brief Get the names of joints/controllers which this controller exclusively claims. */
  virtual std::vector<std::string> getClaimedNames()
  {
    // We don't claim anything (for now)
    std::vector<std::string> names;
    return names;
  }

private:
  ControllerManager* manager_;
  std::vector<JointHandlePtr> joints_;

  bool initialized_;  /// is KDL structure setup

  KDL::Chain kdl_chain_;
  KDL::JntArray positions_;
  boost::shared_ptr<KDL::ChainDynParam> kdl_chain_dynamics_;
  boost::shared_ptr<KDL::ChainJntToJacSolver> kdl_jac_solver_;

  ros::Publisher wrench_pub_;
};

}  // namespace robot_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robot_controllers::ForceEstimation, robot_controllers::Controller)
