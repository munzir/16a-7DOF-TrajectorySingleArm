#include "Controller.hpp"
#include <nlopt.hpp>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <iomanip>

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

    info.open  ("./data/info.txt");

    dataEE_xyz.open ("./data/dataEE_xyz.txt");
    dataEE_xyz  << "dataEE_xyz" << endl;

  dt = 0.001;

  // Define coefficients for sinusoidal, pulsation frequency for q and dq
  cout << " -------------- " << endl;
  cout << " Setting coefficients for optical trajectories | ";



  // // Values from actual URDF - potentially (more) optical trajectories
  // // Used to generate TRAINING Trajectories
  wf << 0.414118417633, 0.558048373585, 0.606608708122, 0.388369687999, 0.512745717718;
  a <<  0.206, 0.322, 0.386, 0.386,
        0.092, 0.186, 0.386, 0.386,
        0.386, 0.386, 0.386, -0.043,
        0.296, 0.262, 0.062, -0.043,
        0.386, 0.262, 0.062, 0.062,
        0.368, 0.186, 0.262, 0.186,
        0.262, 0.386, 0.28, 0.28,
       -0.009, -0.36, 0.311, -0.362,
        0.095, -0.132, -0.363, 0.474,
       -0.418, -0.25, -0.12, 0.119,
        0.023, 0.113, 0.497, 0.213,
       -0.23, -0.237, 0.153, -0.147,
        0.366, 0.366, 0.302, -0.373,
       -0.247, -0.166, 0.315, 0.031,
        0.26, 0.5, -0.477, -0.076,
        0.5, -0.313, 0.333, -0.091,
        0.159, 0.09, 0.363, -0.141,
        0.128, 0.304, 0.429, 0.446,
        0.161, 0.457, 0.5, 0.036,
        0.26, 0.099, 0.217, 0.178,
        -0.041, 0.458, 0.33, 0.145,
        -0.255, 0.293, -0.402, -0.5,
        0.5, -0.5, 0.467, 0.253,
        -0.5, 0.306, 0.5, -0.29,
        -0.155, 0.5, -0.253, -0.049,
        0.5, 0.5, 0.485, -0.027,
        -0.494, 0.047, -0.289, -0.126,
        -0.5, -0.187, -0.159, -0.5,
        -0.125, 0.5, -0.315,
        -0.5, 0.5, -0.5, 0.5, 0.5,
        -0.5, 0.5, 0.5, -0.5,
        -0.469, 0.15, 0.026, -0.359,
        0.5, 0.5, 0.115, -0.463,
        -0.5, -0.127, 0.048, -0.271,
        -0.377, -0.156, 0.247, -0.5;

  b <<  -0.031, 0.386, 0.386, 0.386,
        0.298, 0.386, 0.386, 0.386,
        0.386, 0.154, 0.08, 0.08,
        0.231, 0.372, 0.367, 0.162,
        0.386, -0.043, -0.043, 0.262,
        0.319, -0.031, 0.262, -0.043,
        0.386, -0.043, 0.22, 0.18,
       -0.051, 0.027, 0.003, -0.332,
       -0.292, 0.358, -0.056, -0.436,
       -0.355, 0.039, -0.397, -0.445,
        0.328, 0.256, -0.36, 0.143,
        0.428, 0.093, 0.035, -0.28,
       -0.39, -0.085, 0.388, 0.46,
       -0.046, 0.135, -0.428, 0.387,
       -0.044, 0.035, -0.12, -0.176,
        0.165, 0.14, 0.014, 0.303,
        -0.107, 0.115, 0.5, -0.004,
        -0.122, -0.064, 0.221, 0.354,
        -0.087, 0.041, -0.242, -0.241,
        0.167, 0.266, 0.339, 0.142,
        0.288, 0.355, 0.114, -0.025,
        0.268, -0.5, -0.315, -0.5,
        0.5, 0.202, 0.5, 0.5,
        -0.202, -0.169, 0.5, 0.204,
        -0.037, -0.384, 0.5, 0.019,
        -0.5, 0.025, -0.125, 0.271,
        -0.5, 0.461, 0.485, -0.02,
        0.197, 0.5, -0.288, -0.454,
        0.153, -0.5, -0.014, -0.288,
        0.252, 0.203, 0.5, 0.5,
        -0.434, -0.36, 0.5, 0.234,
        0.395, -0.5, 0.5, -0.376,
        -0.5, 0.5, -0.443, 0.431,
        -0.5, 0.178, 0.155, -0.387,
        0.5, 0.5, -0.216, -0.439;

  q0 << -0.187, 0.085, -0.09, -0.014, -0.053, 0.187, 0.066,
        0.235, -0.004, -0.071, 0.095, -0.141, 0.208, -0.182,
        0.265, 0.256, 0.277, 0.291, -0.185, -0.191, 0.436,
        0.263, 0.268, 0.436, 0.436, 0.386, 0.249, 0.436,
        0.26, 0.42, 0.436, 0.436, 0.212, 0.12, 0.436;


  // // Values from random URDF
  // // Used to generate TESTING trajectories
  // wf << 0.433881552676;
  // a << 0.127, -0.062, 0.464, -0.109,
  //      0.112, -0.647, -0.325, 0.417,
  //     -0.156, -0.345, -0.035, 0.436,
  //     -0.153, -0.187, 0.481, 0.831,
  //     -0.482, -0.377, 0.107, 0.138,
  //      0.074, -0.117, 0.403, -0.085,
  //     -0.417, 0.157, -0.354, 0.346;
  //
  // b << 0.42, -0.485, 0.196, -0.337,
  //     -0.394, 0.313, -0.357, -0.363,
  //      0.326, -0.278, -0.258, 0.066,
  //     -0.475, -0.296, -0.429, 0.11,
  //     -0.098, 0.291, -0.137, 0.183,
  //     -0.065, 0.069, 0.284, -0.174,
  //     -0.134, -0.364, 0.168, -0.301;
  //
  // q0 << -0.203, -0.178, 0.397, 0.105, 0.093, -0.274, -0.204;

  mRobot->setPositions(q0.segment(set,DOF));

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
    // double sign = sgn(dq(i));
    err(i) = -( 2 / ( 1 + exp(-2*dq(i)) ) -1 ) ;
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
      qref(joint) = qref(joint) + (a(joint+set*DOF, l-1)/(wf(set)*l))*sin(wf(set)*l*mTime)
      - (b(joint+set*DOF, l-1)/(wf(set)*l))*cos(wf(set)*l*mTime);
      dqref(joint) = dqref(joint) + a(joint+set*DOF,l-1)*cos(wf(set)*l*mTime)
      + b(joint+set*DOF, l-1)*sin(wf(set)*l*mTime);
    }
  }
  qref = qref + q0.segment(set*DOF,DOF);

  mTime  += dt;
  mTime2 += dt;
  count  +=  1;

  // Get the stuff that we need
  Eigen::MatrixXd  M    = mRobot->getMassMatrix();                // n x n
  Eigen::VectorXd Cg    = mRobot->getCoriolisAndGravityForces();  // n x 1
  Eigen::VectorXd  q    = mRobot->getPositions();                 // n x 1
  Eigen::VectorXd dq    = mRobot->getVelocities();                // n x 1
  Eigen::VectorXd ddq   = mRobot->getAccelerations();             // n x 1
  Eigen::VectorXd ddqref = -mKp*(q - qref) - mKv*(dq - dqref);    // n x 1

  // Get the EE's cartesian coordinate in world frame
  Eigen::VectorXd EE_pos = mEndEffector->getTransform().translation();

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
  // errCoeff(0,0) =  30.0;    //30.0;     30.0
  // errCoeff(1,1) = 200.0;    //200.0;    10.0
  // errCoeff(2,2) =  15.0;    //15.0;      5.0
  // errCoeff(3,3) = 100.0;    //100.0;     5.0
  // errCoeff(4,4) =   3.0;    //3.0;       3.0
  // errCoeff(5,5) =  25.0;    //25.0;      5.0
  // errCoeff(6,6) =   1.0;    //1.0;       1.0

  errCoeff(0,0) =  30.0;    //30.0;     30.0
  errCoeff(1,1) = 200.0;    //200.0;    10.0
  errCoeff(2,2) =   5.0;    //15.0;      5.0
  errCoeff(3,3) =   5.0;    //100.0;     5.0
  errCoeff(4,4) =   3.0;    //3.0;       3.0
  errCoeff(5,5) =   3.0;    //25.0;      5.0
  errCoeff(6,6) =   0.0;    //1.0;       1.0
  Eigen::VectorXd mForceErr = mForces + errCoeff*error(dq, DOF);


  // Apply the joint space forces to the robot
  mRobot->setForces(mForceErr);

  int sample = 30;
  if (count%sample == 0) {
    cout << fixed;
    cout << setprecision(2);
    cout << "Saving data | Time: " << mTime << " seconds | Sample #: " << count << endl;
    dataTime    << mTime << endl;
    dataQ       << q.transpose() << endl;
    dataQref    << qref.transpose() << endl;
    dataQdot    << dq.transpose() << endl;
    dataQdotdot << ddq.transpose() << endl;
    dataTorque  << mForces.transpose() << endl;
    dataM       << M << endl;
    dataCg      << Cg.transpose() << endl;
    dataError   << (errCoeff*error(dq, DOF)).transpose() << endl;
    dataEE_xyz  << EE_pos.transpose() << endl;
  }

  // Closing operation
  double T = 2*3.1416/wf(set);

  if (mTime2 >= 1*T && set < totalSet-1) {
    mTime2 = 0;
    set += 1;
    cout << "---------- GOING TO NEXT SET --------------" << endl << endl;
  }
  else if (mTime2 >= 1*T && set == totalSet-1) {
    cout << "No more sets ..." << endl;
    cout << "Time period met. Stopping data recording ...";

    info   << "Type: Training Set" << endl;
    info   << "Error coefficients:\n" << errCoeff << endl;

    info.close();
    dataTime.close();
    dataQ.close();
    dataQref.close();
    dataQdot.close();
    dataQdotdot.close();
    dataTorque.close();
    dataM.close();
    dataCg.close();
    dataError.close();
    dataEE_xyz.close();
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
