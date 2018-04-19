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
  rw::math::Transform3D<double> t_home;
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
    for (size_t k = 0; k < solution.size(); k++)
    {
      std::cout << "solution solve inkin " << k << " -> " << solution[k] << std::endl;
    }
    if (solution.size() > 0)
    {
      setQ(solution[0]);
    }

    /*rw::math::Q q_algo =
        algorithm1(device, state, tool_frame, baseTtool_desired, getQ());*/

    return true;
  }

  rw::math::Transform3D<double> getPose()
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
    auto endTransform = device->baseTframe(endFrame, state);
    auto rpy = RPY<double>(endTransform.R());
    auto pos = endTransform.P();
    std::cout << "RPY UNO: " << rpy[0] << " " << rpy[1] << " " << rpy[2] << std::endl;
    std::cout << "POS UNO: " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;

    return endTransform;
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
    for (size_t k = 0; k < path.size(); k++)
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

  bool initialize()
  {
    q_home = getQ();
    t_home = getPose();

    return true;
  }

  bool planner_rrt(double x, double y, double z, RPY<double> rpy)
  {
    rw::math::Vector3D<double> pos(x, y, z);
    rw::math::Q q_goal;
    rw::math::Transform3D<double> baseTtool_desired(pos, rpy.toRotation3D());
    auto solver = new rw::invkin::JacobianIKSolver(device, state);
    std::vector<rw::math::Q> solutions = solver->solve(baseTtool_desired, state);
    std::vector<rw::math::Q> path;
    path.push_back(getQ());
    rw::math::Q diferencia;

    if (solutions.size() > 0)
    {
      q_goal = solutions[0];
    }

    do
    {
      // TODO Set as constants
      auto x_rand = x - 0.01 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (0.01 + 0.01)));
      auto y_rand = y - 0.01 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (0.01 + 0.01)));
      auto z_rand = z - 0.001 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (0.001 + 0.001)));

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
  robot.initialize();  // Otherwise we can forget to set the device or something...
  /* #########  PLANNER CONSTRAINT ######################################### */

  std::cout << robot.getQ() << std::endl;  // para mostrar nada mas
  // std::cout << robot.getPose() << std::endl;
  auto robot_transform = robot.getPose();
  std::cout << "Metete unas coordenadas loco: " << std::endl;
  float x, y, z;
  std::cin >> x >> y >> z;
  cout << "llamamos planner " << endl;
  auto rpy = RPY<double>(robot_transform.R());
  // TODO change roll/yaw -> NEEDED

  robot.planner_rrt(x, y, z, rpy);
  cout << " planer termiando " << endl;
  return 0;
}
