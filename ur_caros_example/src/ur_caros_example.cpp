#include <iostream>
#include "caros/serial_device_si_proxy.h"
#include "ros/package.h"
#include "rw/rw.hpp"
//#include "rw/invkin.hpp"
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using namespace rw::math;
using namespace std;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 10

class URRobot
{
  using Q = rw::math::Q;

private:
  ros::NodeHandle nh;
  rw::models::WorkCell::Ptr wc;
  rw::models::Device::Ptr device;
  rw::kinematics::State state;
  caros::SerialDeviceSIProxy *robot;
  Q q_home;
  // rw::invkin::JacobianIKSolver *solver;

public:
  URRobot()
  {
    auto packagePath = ros::package::getPath("ur_caros_example");
    wc = rw::loaders::WorkCellLoader::Factory::load(packagePath + "/WorkStation_1/WC1_Scene.wc.xml");
    device = wc->findDevice("UR1");
    state = wc->getDefaultState();
    robot = new caros::SerialDeviceSIProxy(nh, "caros_universalrobot");

    // Wait for first state message, to make sure robot is ready
    ros::topic::waitForMessage<caros_control_msgs::RobotState>("/caros_universalrobot/"
                                                               "caros_serial_device_service_interface/"
                                                               "robot_state",
                                                               nh);
    // init q_home to its basic configuration
    // q_home = {};
    ros::spinOnce();
  }

  Q getQ()
  {
    // spinOnce processes one batch of messages, calling all the callbacks
    ros::spinOnce();
    Q q = robot->getQ();
    device->setQ(q, state);
    return q;
  }

  bool setQ(Q q)
  {
    // Tell robot to move to joint config q
    float speed = 1;
    std::cout << "dentro de setQ .>  q :" << std::endl << q << std::endl << std::endl;
    if (robot->moveServoQ(q, speed))
    {
      // std::cout << "If setQ :" << std::endl;
      // moveServoQ terminates before the robot is at the destination, so wait
      // until it is
      Q qCurrent;
      do
      {
        // std::cout << "Current joint config:" << std::endl << robot->getQ() <<
        // std::endl << std::endl;
        ros::spinOnce();
        qCurrent = robot->getQ();
      } while ((qCurrent - q).norm2() > 0.01);
      // std::cout << "antes de device (SET Q) " << std::endl;
      device->setQ(q, state);
      return true;
    }
    else
      return false;
  }

  //////////////////////////////////////////////////////////////////////////////

  rw::math::VelocityScrew6D<double> calculateDeltaU(const rw::math::Transform3D<double> &baseTtool,
                                                    const rw::math::Transform3D<double> &baseTtool_desired)
  {
    // Calculate the positional difference, dp
    // cout << "dentro calcula de delta U " << endl;
    rw::math::Vector3D<double> dp = baseTtool_desired.P() - baseTtool.P();

    // Calculate the rotational difference, dw
    rw::math::EAA<double> dw(baseTtool_desired.R() * rw::math::inverse(baseTtool.R()));

    return rw::math::VelocityScrew6D<double>(dp, dw);
  }

  rw::math::Q algorithm1(const rw::models::Device::Ptr device, rw::kinematics::State state,
                         const rw::kinematics::Frame *tool, const rw::math::Transform3D<double> baseTtool_desired,
                         rw::math::Q q)
  {
    // We need an initial base to tool transform and the positional error at the
    // start (deltaU)
    rw::math::Transform3D<> baseTtool = device->baseTframe(tool, state);
    rw::math::VelocityScrew6D<double> deltaU = calculateDeltaU(baseTtool, baseTtool_desired);

    // This epsilon is the desired tolerance on the final position.
    const double epsilon = 0.1;

    while (deltaU.norm2() > epsilon)
    {
      rw::math::Jacobian J = device->baseJframe(tool, state);  // This line does the same as the function from
                                                               // Programming Exercise 4.1
      // We need the inverse of the jacobian. To do that, we need to access the
      // Eigen representation of the matrix. For information on Eigen, see
      // http://eigen.tuxfamily.org/.
      rw::math::Jacobian Jinv(J.e().inverse());

      // In RobWork there is an overload of operator* for Jacobian and
      // VelocityScrew that gives Q This can also manually be done via Eigen as
      // J.e().inverse() * deltaU.e() Note that this approach only works for
      // 6DOF robots. If you use a different robot, you need to use a pseudo
      // inverse to solve the equation J * deltaQ = deltaU
      rw::math::Q deltaQ = Jinv * deltaU;

      // Here we add the change in configuration to the current configuration
      // and move the robot to that position.
      q += deltaQ;
      device->setQ(q, state);

      // We need to calculate the forward dynamics again since the robot has
      // been moved
      baseTtool = device->baseTframe(tool, state);  // This line performs the
                                                    // forward kinematics
                                                    // (Programming
                                                    // Exercise 3.4)

      // cout << "Base T tool   " << baseTtool << endl;
      // cout << "Bast T tool desired " << baseTtool_desired << endl;

      // Update the cartesian position error
      deltaU = calculateDeltaU(baseTtool, baseTtool_desired);
      // cout << "delta U "  << deltaU << endl;
    }
    return q;
  }
  /////////////////////////////////////////////////////////////////////////////////

  bool setPose(rw::math::Vector3D<double> pos, RPY<double> angles)
  {
    ros::spinOnce();
    // std::cout <<"RPY UNO: "<<angles[0]<<" "<<angles[1]<<"
    // "<<angles[2]<<std::endl;  std::cout <<"POS UNO: "<<pos[0]<<" "<<pos[1]<<"
    // "<<pos[2]<<std::endl;
    rw::kinematics::Frame *tool_frame = wc->findFrame("UR1.TCP");
    /*std::vector<rw::kinematics::Frame*> frames = wc->getFrames();

    for(int i = 0; i < frames.size(); i++){
            std::cout<<frames[i]->name<<std::endl;
    }*/
    if (tool_frame == nullptr)
    {
      RW_THROW("Tool frame not found!");
    }
    rw::math::Transform3D<double> baseTtool_desired(pos, angles.toRotation3D());
    // The inverse kinematics algorithm needs to know about the device, the tool
    // frame and the desired pose. These parameters are const since they are not
    // changed by inverse kinematics We pass the state and the configuration, q,
    // as value so we have copies that we can change as we want during the
    // inverse kinematics.

    auto solver = new rw::invkin::JacobianIKSolver(device, state);
    // auto solver = rw::invkin::JacobianIKSolver(device, state);
    // auto solution = solver.solve(baseTtool_desired,state);
    std::vector<rw::math::Q> solution = solver->solve(baseTtool_desired, state);
    for (int k = 0; k < solution.size(); k++)
      std::cout << "solution solve inkin " << k << " -> " << solution[k] << std::endl;
    if (solution.size() > 0)
      setQ(solution[0]);

    /*rw::math::Q q_algo =
        algorithm1(device, state, tool_frame, baseTtool_desired, getQ());*/

    return true;
  }

  bool getPose()
  {
    // spinOnce processes one batch of messages, calling all the callbacks

    // rw::math::Pose6D<double> pose = rw::math::Pose6D::getPos();
    // Vector3D<double> pos = device->getPos();
    // EAA<double> eaa = device->getEAA();
    // rw::math::Transform3D<double> transformacion = device->baseTframe;
    // Q q = getQ();
    // device(q, state);
    /*auto* tcp = wc->findFrame("Joint4");
    auto name = device->getName();
    std::cout << "frames " << std::endl << tcp << std::endl << std::endl;
    std::cout << "name " << std::endl << name << std::endl << std::endl;*/

    ros::spinOnce();
    auto endFrame = wc->findFrame("UR1.TCP");
    ;
    auto endTransform = device->baseTframe(endFrame, state);
    auto rpy = RPY<double>(endTransform.R());
    auto pos = endTransform.P();
    std::cout << "RPY UNO: " << rpy[0] << " " << rpy[1] << " " << rpy[2] << std::endl;
    std::cout << "POS UNO: " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;

    return true;
  }

  bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q)
  {
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ(q, testState);
    colFrom = detector.inCollision(testState, &data);
    if (colFrom)
    {
      cerr << "Configuration in collision: " << q << endl;
      cerr << "Colliding frames: " << endl;
      FramePairSet fps = data.collidingFrames;
      for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++)
      {
        cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
      }
      return false;
    }
    return true;
  }

  rw::math::Q nearest_neigbor(rw::math::Q q_rand, std::vector<rw::math::Q> path)
  {
    double min = 10000;
    double dist = 0;
    Q q_nearest = path[0];
    for (int k = 0; k < path.size(); k++)
    {
      Q dq = q_rand - path[k];
      dist = dq.norm2();
      if (dist < min)
      {
        min = dist;
        q_nearest = path[k];
      }
    }
    return q_nearest;
  }

  bool NewConfig(rw::math::Q q_rand, rw::math::Q q_near, rw::math::Q &q_new)
  {
    CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());

    Q dq = q_near - q_rand;
    double extend = 0.01;
    q_new = q_rand + extend * (dq) / (dq.norm2());
    return checkCollisions(device, state, detector, q_new);
    // cuando no choca -> true
  }

  bool planner_rrt(double x, double y, double z, RPY<double> rpy)
  {
    // PlannerConstraint constraint =
    //     PlannerConstraint::make(&detector, device, state);

    //  QSampler::Ptr sampler = QSampler::makeConstrained(
    //      QSampler::makeUniform(device), constraint.getQConstraintPtr());
    // QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();

    // QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(
    //     constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

    // TODO Initialize this value at start (Get default_q from configuration)
    Q from(6, 0, -1.5708, 0, -1.57095, 0, -0.000153494);
    // Q to(6,1.7,0.6,-0.8,0.3,0.7,-0.5); // Very difficult for planner
    // Q to(6, 1.07129, -0.803471, -0.431695, -1.91307, -2.6387, 0.775581);
    /*
    - obtener rand transf..
    */
    rw::math::Vector3D<double> pos(x, y, z);
    // auto rpy = RPY<double>(yaw, p, r);  /// OJO al yaw , r

    rw::math::Transform3D<double> baseTtool_desired(pos, rpy.toRotation3D());
    auto solver = new rw::invkin::JacobianIKSolver(device, state);
    rw::math::Q q_goal;
    std::vector<rw::math::Q> solutions = solver->solve(baseTtool_desired, state);
    std::vector<rw::math::Q> path;
    path.push_back(from);
    rw::math::Q diferencia;

    if (solutions.size() > 0)
    {
      q_goal = solutions[0];
    }

    do
    {
      auto x_rand = x - 0.1 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (0.1 + 0.1)));
      auto y_rand = y - 0.1 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (0.1 + 0.1)));
      auto z_rand = z - 0.1 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (0.1 + 0.1)));

      rw::math::Vector3D<double> pos_rand(x_rand, y_rand, z_rand);
      rw::math::Transform3D<double> baseTtool_desired_rand(pos_rand, rpy.toRotation3D());
      // Biased solution towards goal area
      std::vector<rw::math::Q> solutions_rand = solver->solve(baseTtool_desired_rand, state);
      rw::math::Q q_rand;

      if (solutions.size() > 0)
      {
        q_rand = solutions_rand[0];
      }

      Q q_near = nearest_neigbor(q_rand, path);
      Q q_new;

      if (NewConfig(q_rand, q_near, q_new))
      {
        path.push_back(q_new);
      }

      diferencia = q_new - q_goal;
    } while (diferencia.norm2() > 0.1);

    // if (!checkCollisions(device, state, detector, from)) return true;
    // if (!checkCollisions(device, state, detector, to)) return true;
    // cout << "Planning from " << from << " to " << to << endl;

    // planner->query(from, to, path, MAXTIME);

    cout << "Path of length " << path.size() << endl;

    for (QPath::iterator it = path.begin(); it < path.end(); it++)
    {
      // cout << *it << endl;
      cout << "set Q " << *it << endl;
      setQ(*it);
    }

    cout << "Program done." << endl;
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "URRobot");
  URRobot robot;

  /* std::cout << "Current joint config:" << std::endl
             << robot.getQ() << std::endl
             << std::endl;
 */
  /* std::cout << "Input destination joint config in radians:" << std::endl;
   float q1, q2, q3, q4, q5, q6;
   robot.getPose();*/

  /*std::cin >> q1 >> q2 >> q3 >> q4 >> q5 >> q6;
  rw::math::Q q(6, q1, q2, q3, q4, q5, q6);
  std::cout << "ANTES IF :" << std::endl;
  if (robot.setQ(q)){
          // std::cout << "dentro if :" << std::endl;
          std::cout << std::endl << "New joint config:" << std::endl <<
  robot.getQ() << std::endl;
  }
  else{
          // std::cout << "dentro else :" << std::endl;
          std::cout << std::endl << "Failed to move robot" << std::endl;
  }*/
  // std::cout << "antes get pose, despues if/else" << std::endl;

  /*std::cout << "POSICIONES Y ANGULOS INPUT:" << std::endl;
  float x, y, z, r, p, yaw;
  std::cin >> x >> y >> z >> r >> p >> yaw;
  rw::math::Vector3D<double> pos(x, y, z);
  auto rpy = RPY<double>(yaw, p, r);
  // std::cout << "datos recogidos, antes set pose" << endl;
  robot.setPose(pos, rpy);
  cout << std::endl << "final set pose  " << endl;
  robot.getPose();*/
  // float q1, q2, q3, q4, q5, q6;
  // cout << "introduce Q solution" << endl;
  // std::cin >> q1 >> q2 >> q3 >> q4 >> q5 >> q6;
  // rw::math::Q Q_in(6, q1, q2, q3, q4, q5, q6);

  // robot.setQ(Q_in);

  /* #########  PLANNER CONSTRAINT ######################################### */
  cout << "llamamos planner " << endl;
  auto rpy = RPY<double>(1.5, 1.5, 1.5);
  int xx = 1;
  int yy = 1;
  int zz = 1;
  bool num = robot.planner_rrt(xx, yy, zz, rpy);
  cout << " planer termiando " << endl;
  return 0;
}
