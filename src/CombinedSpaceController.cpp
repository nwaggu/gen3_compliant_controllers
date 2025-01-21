// pinocchio headers
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include <gen3_compliant_controllers/JointSpaceCompliantController.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"


namespace combined_controller_ns{

  class CombinedController: public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface, hardware_interface::JointStateInterface>{
  
  bool CombinedController::init(hardware_interface::RobotHW* robot, ros::NodeHandle& n)
{

  using hardware_interface::EffortJointInterface;
  using hardware_interface::JointStateInterface;

  mNodeHandle.reset(new ros::NodeHandle{n});

  //Get current joint information
  const auto jointParameters = loadJointsFromParameter(n, "joints", "effort");
  //If no joint to control quit
  if (jointParameters.empty())
    return false;

  //Printing results of joint names
  ROS_INFO_STREAM("Controlling " << jointParameters.size() << " joints:");
  //Loop through each jointParameter, will only have as many as there are joints
  for (const auto& param : jointParameters)
  {
    ROS_INFO_STREAM("- " << param.mName << " (type: " << param.mType << ")");

    if (param.mType != "effort")
    {
      ROS_ERROR_STREAM(
          "Joint '" << param.mName
                    << "' is not effort-controlled and cannot be "
                       "used in a gravity compensation controller");
      return false;
    }
  }

  //Create variable parameterName
  std::string parameterName;
  //Pull robot_desc parameter from parameter server or whatever
  mNodeHandle->param<std::string>("robot_description_parameter", parameterName, "/robot_description");

  // Load the URDF from the parameter server.
  std::string robotDescription;
  //If you can't get robotDescription return false
  if (!mNodeHandle->getParam(parameterName, robotDescription))
  {
    ROS_ERROR_STREAM("Failed loading URDF from '" << parameterName << "' parameter.");
    return false;
  }

  //Builds model using pinnocchio :)
  mModel = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModelFromXML(robotDescription, *mModel.get());
  if (!mModel)
    return false;
  mData.reset(new pinocchio::Data{*mModel.get()});


  //Get robot joint data from HWI
  const auto jointStateInterface = robot->get<JointStateInterface>();
  if (!jointStateInterface)
  {
    ROS_ERROR("Unable to get JointStateInterface from RobotHW instance.");
    return false;
  }
  //Intialize JointStateUpdater
  mJointStateUpdater.reset(new JointStateUpdater{mModel, jointStateInterface});

  //Get the Effort Interface
  const auto effortJointInterface = robot->get<EffortJointInterface>();
  if (!effortJointInterface)
  {
    ROS_ERROR("Unable to get EffortJointInterface from RobotHW instance.");
    return false;
  }

  //Match joint handles for control
  const auto numControlledDofs = mModel->nv;
  mControlledJointHandles.resize(numControlledDofs);
  for (size_t idof = 0; idof < numControlledDofs; ++idof)
  {
    const auto dofName = mModel->names[idof + 1];
    try
    {
      auto handle = effortJointInterface->getHandle(dofName);
      mControlledJointHandles[idof] = handle;
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Unable to get interface of type 'EffortJointInterface' for joint '" << dofName << "'.");
      return false;
    }
  }
  std::cout << "ControlledJointHandles created" << std::endl;

  //Point of Reference? Can This Be Changed
  std::string mEENodeName;
  n.getParam("/end_effector", mEENodeName);
  mEENode = mModel->getFrameId(mEENodeName);

  //????
  mExtendedJoints = new ExtendedJointPosition(numControlledDofs, 3 * M_PI / 2);
  mExtendedJointsGravity = new ExtendedJointPosition(numControlledDofs, 3 * M_PI / 2);

  //Setup Const Matrices
  mCount = 0;
  mNumControlledDofs = mModel->nv;

  mJointStiffnessMatrix.resize(mNumControlledDofs, mNumControlledDofs);
  mJointStiffnessMatrix.setZero();

  mRotorInertiaMatrix.resize(mNumControlledDofs, mNumControlledDofs);
  mRotorInertiaMatrix.setZero();

  mFrictionL.resize(mNumControlledDofs, mNumControlledDofs);
  mFrictionL.setZero();

  mFrictionLp.resize(mNumControlledDofs, mNumControlledDofs);
  mFrictionLp.setZero();

  mTaskKMatrix.resize(6, 6);
  mTaskKMatrix.setZero();

  mTaskDMatrix.resize(6, 6);
  mTaskDMatrix.setZero();

  //Setup Matrix Values
  if (mNumControlledDofs == 6)
  {
    mJointStiffnessMatrix.diagonal() << 4000, 4000, 4000, 3500, 3500, 3500;
    mRotorInertiaMatrix.diagonal() << 0.3, 0.3, 0.3, 0.18, 0.18, 0.2;
    mFrictionL.diagonal() << 75, 75, 75, 40, 40, 40;
    mFrictionLp.diagonal() << 5, 5, 5, 4, 4, 4;
  }
  else
  {
    mJointStiffnessMatrix.diagonal() << 4000, 4000, 4000, 4000, 3500, 3500, 3500;
    mRotorInertiaMatrix.diagonal() << 0.3, 0.3, 0.3, 0.3, 0.18, 0.18, 0.2;
    mFrictionL.diagonal() << 75, 75, 75, 75, 40, 40, 40;
    mFrictionLp.diagonal() << 5, 5, 5, 5, 4, 4, 4;
  }
  mTaskKMatrix.diagonal() << 200, 200, 200, 100, 100, 100;
  mTaskDMatrix.diagonal() << 40, 40, 40, 20, 20, 20;

  // Initialize buffers to avoid dynamic memory allocation at runtime.
  mDesiredPosition.resize(numControlledDofs);
  mDesiredVelocity.resize(numControlledDofs);
  mZeros.resize(numControlledDofs);
  mZeros.setZero();

  //mName = internal::getLeafNamespace(n);

  // Initialize controlled joints
  std::string param_name = "joints";
  if (!n.getParam(param_name, mJointNames))
  {
    ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
    return false;
  }

  // ROS API: Subscribed topics
  mSubCommand = n.subscribe<moveit_msgs::CartesianTrajectoryPoint>("command", 1, &CombinedController::commandCallback, this);

  // Dynamic reconfigure server
  f = boost::bind(&CombinedController::dynamicReconfigureCallback, this, _1, _2);
  server.setCallback(f);

  ROS_INFO("CombinedCompliantController initialized successfully");
  return true;
}
  
  void update (const ros::Time& time, const ros::Duration& period){
    
  }
  
  
  
  
  
  
  
  };

}

