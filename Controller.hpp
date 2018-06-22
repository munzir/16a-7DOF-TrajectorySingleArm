#ifndef EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_
#define EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_

#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <dart/dart.hpp>
using namespace std;
#define DOF 7
#define TOTAL 5

/// \brief Operational space controller for 6-dof manipulator
class Controller
{
public:
  /// \brief Constructor
  Controller(dart::dynamics::SkeletonPtr _robot,
             dart::dynamics::BodyNode* _endEffector);

  /// \brief Destructor
  virtual ~Controller();

  /// \brief
  void update(const Eigen::Vector3d& _targetPosition);

  // double getTime() {return mTime;}

    /// \brief Get robot
  dart::dynamics::SkeletonPtr getRobot() const;

  /// \brief Get end effector of the robot
  dart::dynamics::BodyNode* getEndEffector() const;

  /// \brief Keyboard control
  virtual void keyboard(unsigned char _key, int _x, int _y);

private:
  /// File handles for dumping data
  ofstream dataQ;
  ofstream dataQref;
  ofstream dataQdot;
  ofstream dataQdotdot;
  ofstream dataTorque;
  ofstream dataTime;
  ofstream dataM;
  ofstream dataCg;
  ofstream dataError;
  ofstream info;
  ofstream dataEE_xyz;

  /// Variables for calculating reference (optical) trajectories
  double mTime;
  double mTime2;
  double dt;
  int count = 0;
  const int totalSet = TOTAL;
  int set = 0;
  Eigen::Matrix<double, TOTAL, 1> wf;
  Eigen::Matrix<double,   DOF, 1> qref;
  Eigen::Matrix<double,   DOF, 1> dqref;
  Eigen::Matrix<double, DOF*TOTAL, 1> q0;
  Eigen::Matrix<double, DOF*TOTAL, 4> a, b;


  Eigen::Matrix<double, DOF, DOF>  M, Mlast;
  Eigen::Matrix<double, DOF, 1> Cg, Cglast;
  Eigen::Matrix<double, DOF, 1>  q, qlast;
  Eigen::Matrix<double, DOF, 1> dq, dqlast;
  Eigen::Matrix<double, DOF, 1> ddq;
  Eigen::Matrix<double, DOF, 1> mForceLast;

  /// \brief Robot
  dart::dynamics::SkeletonPtr mRobot;

  /// \brief End-effector of the robot
  dart::dynamics::BodyNode* mEndEffector;

  /// \brief Control forces
  Eigen::VectorXd mForces;

  /// \brief Proportional gain for the virtual spring forces at the end effector
  Eigen::Matrix<double,DOF,DOF> mKp;

  /// \brief Derivative gain for the virtual spring forces at the end effector
  Eigen::Matrix<double,DOF,DOF> mKv;
};

#endif  // EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_
