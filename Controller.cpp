#include "Controller.hpp"
#include <nlopt.hpp>
#include <iostream>
#include <fstream>
#include <stdlib.h>

using namespace dart;
using namespace std;
#define DOF 7

//131720 is the stack limit - 784
//==============================================================================
Controller::Controller(dart::dynamics::SkeletonPtr _robot,
                       dart::dynamics::BodyNode* _endEffector)
  : mRobot(_robot),
    mEndEffector(_endEffector)
{
  cout << endl << "============= Controller constructor activated =============" << endl << endl;
  assert(_robot != nullptr);
  assert(_endEffector != nullptr);

  cout << "Getting total degrees of freedom ";
  const int dof = mRobot->getNumDofs();
  cout << "| DOF = " << dof << endl;

  if (dof != DOF) {
    cout << "DOF do not match with defined constant ... exiting program!" << endl;
    exit (EXIT_FAILURE);
  }

  cout << "Initializing time ==> ";
  mTime = 0;
  cout << "t(0) = " << mTime << endl;

  mForces.setZero(DOF);
  mKp = Eigen::Matrix<double, DOF, DOF>::Zero();
  mKv = Eigen::Matrix<double, DOF, DOF>::Zero();

  for (int i = 0; i < DOF; ++i) {
    mKp(i, i) = 750.0;
    mKv(i, i) = 250.0;
  }

  // Remove position limits
  for (int i = 0; i < DOF; ++i)
    _robot->getJoint(i)->setPositionLimitEnforced(false);

  // Set joint damping
  for (int i = 0; i < DOF; ++i)
    _robot->getJoint(i)->setDampingCoefficient(0, 0.5);

  // Dump data
  dataTime.open   ("./data/dataTime.txt");
  dataTime    << "dataTime" << endl;

  dataQ.open      ("./data/dataQ.txt");
  dataQ       << "dataQ" << endl;

  dataQref.open   ("./data/dataQref.txt");
  dataQref    << "dataQref" << endl;

  dataQdot.open   ("./data/dataQdot.txt");
  dataQdot    << "dataQdot" << endl;

  dataQdotdot.open("./data/dataQdotdot.txt");
  dataQdotdot << "dataQdotdot" << endl;

  dataTorque.open ("./data/dataTorque.txt");
  dataTorque  << "dataTorque" << endl;

  dataM.open      ("./data/dataM.txt");
  dataM       << "dataM" << endl;

  dataCg.open     ("./data/dataCg.txt");
  dataCg      << "dataCg" << endl;

  dataError.open  ("./data/dataError.txt");
  dataError   << "dataError" << endl;

  dt = 0.002;

  // Define coefficients for sinusoidal, pulsation frequency for q and dq
  cout << " -------------- " << endl;
  cout << " Setting coefficients for optical trajectories | ";

  // // Values from actual URDF - (sub) optical trajectories
  // wf = 0.558048373585;
  // a << -0.009, -0.36, 0.311, -0.362,
  //       0.095, -0.132, -0.363, 0.474,
  //       -0.418, -0.25, -0.12, 0.119,
  //       0.023, 0.113, 0.497, 0.213,
  //       -0.23, -0.237, 0.153, -0.147,
  //       0.366, 0.366, 0.302, -0.373,
  //       -0.247, -0.166, 0.315, 0.031;
  //
  // b <<  -0.051, 0.027, 0.003, -0.332,
  //       -0.292, 0.358, -0.056, -0.436,
  //       -0.355, 0.039, -0.397, -0.445,
  //       0.328, 0.256, -0.36, 0.143,
  //       0.428, 0.093, 0.035, -0.28,
  //       -0.39, -0.085, 0.388, 0.46,
  //       -0.046, 0.135, -0.428, 0.387;
  //
  // q0 << 0.235, -0.004, -0.071, 0.095, -0.141, 0.208, -0.182;

  // // Values from actual URDF - potentially (more) optical trajectories
  wf = 0.414118417633;
  a <<  0.206, 0.322, 0.386, 0.386,
        0.092, 0.186, 0.386, 0.386,
        0.386, 0.386, 0.386, -0.043,
        0.296, 0.262, 0.062, -0.043,
        0.386, 0.262, 0.062, 0.062,
        0.368, 0.186, 0.262, 0.186,
        0.262, 0.386, 0.28, 0.28;

  b <<  -0.031, 0.386, 0.386, 0.386,
        0.298, 0.386, 0.386, 0.386,
        0.386, 0.154, 0.08, 0.08,
        0.231, 0.372, 0.367, 0.162,
        0.386, -0.043, -0.043, 0.262,
        0.319, -0.031, 0.262, -0.043,
        0.386, -0.043, 0.22, 0.18;

  q0 << -0.187, 0.085, -0.09, -0.014, -0.053, 0.187, 0.066;


  // // Values from random URDF
  // wf = 0.933881552676;
  // a << 0.127, -0.062, 0.464, -0.109,
  //     -0.122, -0.447, -0.325, 0.417,
  //     -0.156, -0.345, -0.035, 0.436,
  //     -0.153, -0.187, 0.181, 0.231,
  //     -0.482, -0.377, 0.107, 0.138,
  //     0.074, -0.117, 0.403, -0.085,
  //     -0.417, 0.357, -0.354, 0.346;
  //
  // b << 0.42, -0.485, 0.196, -0.337,
  //     -0.394, 0.313, -0.357, -0.363,
  //     0.326, -0.278, -0.258, 0.066,
  //     -0.475, -0.296, -0.429, 0.11,
  //     -0.098, 0.291, -0.137, 0.183,
  //     -0.065, 0.069, 0.284, -0.174,
  //     -0.434, -0.364, 0.168, -0.001;

  q0 << -0.203, -0.178, 0.097, 0.105, 0.193, -0.274, -0.204;

  mRobot->setPositions(q0);

  cout << "Operation completed!" << endl;

  cout << endl << "============= Controller constructor successful ============" << endl << endl;
}

//==============================================================================
Controller::~Controller() {}

//==============================================================================
struct OptParams{
  Eigen::Matrix<double, -1, DOF> P;
  Eigen::VectorXd b;
};

//==============================================================================
void printMatrix(Eigen::MatrixXd A){
  for(int i=0; i<A.rows(); i++){
    for(int j=0; j<A.cols(); j++){
      cout << A(i,j) << ", ";
    }
    cout << endl;
  }
  cout << endl;
}


//==============================================================================
double optFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
  OptParams* optParams = reinterpret_cast<OptParams *>(my_func_data);
  Eigen::Matrix<double, DOF, 1> X(x.data());

  if (!grad.empty()) {
    Eigen::Matrix<double, DOF, 1> mGrad = optParams->P.transpose()*(optParams->P*X - optParams->b);
    // cout << "mGrad: " << endl << mGrad << endl << endl;
    Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
  }
  double normSquared = pow((optParams->P*X - optParams->b).norm(), 2);
  // cout << "Norm sq: " << normSquared << endl << endl;
  return (0.5 * normSquared);
}


//==============================================================================
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

//==============================================================================
Eigen::MatrixXd error(Eigen::VectorXd dq, const int dof) {
  Eigen::MatrixXd err(DOF,1);
  for (int i = 0; i < dof; i++) {
    double sign = sgn(dq(i));
    err(i) =  sign * 1/( 1 + exp(-dq(i)) ) ;
  }
  return err;
}

//==============================================================================
void Controller::update(const Eigen::Vector3d& _targetPosition) {
  // const int dof = mRobot->getNumDofs();

  // Compute joint angles & velocities using Pulsed Trajectories
  qref << 0, 0, 0, 0, 0, 0, 0;
  dqref = qref;

  // cout << "Time: " << mTime << endl;
  for (int joint = 0; joint < DOF; joint++) {
    for (int l = 1; l <= 4; l++) {
      qref(joint) = qref(joint) + (a(joint, l-1)/(wf*l))*sin(wf*l*mTime)
      - (b(joint, l-1)/(wf*l))*cos(wf*l*mTime);
      dqref(joint) = dqref(joint) + a(joint,l-1)*cos(wf*l*mTime)
      + b(joint, l-1)*sin(wf*l*mTime);
    }
  }
  qref = qref + q0;

  mTime += dt;

  // Get the stuff that we need
  Eigen::MatrixXd  M    = mRobot->getMassMatrix();                // n x n
  Eigen::VectorXd Cg    = mRobot->getCoriolisAndGravityForces();  // n x 1
  Eigen::VectorXd  q    = mRobot->getPositions();                 // n x 1
  Eigen::VectorXd dq    = mRobot->getVelocities();                // n x 1
  Eigen::VectorXd ddq   = mRobot->getAccelerations();             // n x 1
  Eigen::VectorXd ddqref = -mKp*(q - qref) - mKv*(dq - dqref);    // n x 1


  // Optimizer stuff
  nlopt::opt opt(nlopt::LD_MMA, DOF);
  OptParams optParams;
  std::vector<double> ddqref_vec(DOF);
  double minf;
  // cout << "Initialized optimizer variables ... " << endl << endl;

  // Perform optimization to find joint accelerations
  Eigen::Matrix<double, DOF, DOF> I7 = Eigen::Matrix<double, DOF, DOF>::Identity();

  // cout << "Passing optimizing parameters ... ";
  optParams.P = I7;
  optParams.b = ddqref;
  // cout << "Success !" << endl << endl;

  opt.set_min_objective(optFunc, &optParams);
  opt.set_xtol_rel(1e-4);
  opt.set_maxtime(0.005);
  opt.optimize(ddqref_vec, minf);
  Eigen::Matrix<double, DOF, 1> ddqRef(ddqref_vec.data());

  mForces   = M*ddqRef + Cg;

  Eigen::Matrix<double, DOF, DOF> errCoeff = Eigen::Matrix<double, DOF, DOF>::Identity();
  errCoeff(0,0) =  30.0;
  errCoeff(1,1) =  10.0;
  errCoeff(2,2) =   5.0;
  errCoeff(3,3) =   5.0;
  errCoeff(4,4) =   3.0;
  errCoeff(5,5) =   5.0;
  errCoeff(6,6) =   1.0;
  Eigen::VectorXd mForceErr = mForces + errCoeff*error(dq, DOF);


  // Apply the joint space forces to the robot
  mRobot->setForces(mForceErr);

  dataTime    << mTime << endl;
  dataQ       << q.transpose() << endl;
  dataQref    << qref.transpose() << endl;
  dataQdot    << dq.transpose() << endl;
  dataQdotdot << ddq.transpose() << endl;
  dataTorque  << mForces.transpose() << endl;
  dataM       << M << endl;
  dataCg      << Cg.transpose() << endl;
  dataError   << (errCoeff*error(dq, DOF)).transpose() << endl;

  // Closing operation
  double T = 2*3.1416/wf;
  if (mTime >= 2*T ) {
    // cout << "Time period met. Closing simulation ..." << endl;
    cout << "Time period met. Stopping data recording ...";
    dataTime.close();
    dataQ.close();
    dataQref.close();
    dataQdot.close();
    dataQdotdot.close();
    dataTorque.close();
    dataM.close();
    dataCg.close();
    dataError.close();
    cout << "File handles closed!" << endl << endl << endl;
    exit (EXIT_FAILURE);
  }
}

//==============================================================================
dart::dynamics::SkeletonPtr Controller::getRobot() const {
  return mRobot;
}

//==============================================================================
dart::dynamics::BodyNode* Controller::getEndEffector() const {
  return mEndEffector;
}

//==============================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/) {
}
