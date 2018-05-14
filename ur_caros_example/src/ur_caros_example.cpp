#include <chrono>
#include <iostream>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include "caros/serial_device_si_proxy.h"
#include "ros/package.h"
#include "rw/rw.hpp"
#include "ur_caros_example/ResetQ.h"
#include "ur_caros_example/Vision.h"

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

class URRobot {
  using Q = rw::math::Q;

 private:
  ros::NodeHandle nh;
  rw::models::WorkCell::Ptr wc;
  rw::models::Device::Ptr device;
  rw::kinematics::State state;
  caros::SerialDeviceSIProxy *robot;
  Q q_home;
  rw::math::Transform3D<double> t_home;
  ros::ServiceServer _vision_subscriber;
  ros::ServiceServer _reset_subscriber;

 public:
  URRobot(ros::NodeHandle n) : nh(n) {
    auto packagePath = ros::package::getPath("ur_caros_example");
    wc = rw::loaders::WorkCellLoader::Factory::load(
        packagePath + "/WorkStation_1/WC1_Scene.wc.xml");
    device = wc->findDevice("UR1");
    state = wc->getDefaultState();
    robot = new caros::SerialDeviceSIProxy(nh, "caros_universalrobot");

    // Wait for first state message, to make sure robot is ready
    ros::topic::waitForMessage<caros_control_msgs::RobotState>(
        "/caros_universalrobot/"
        "caros_serial_device_service_interface/"
        "robot_state",
        nh);
    ros::spinOnce();
  }

  Q getQ() {
    // spinOnce processes one batch of messages, calling all the callbacks
    ros::spinOnce();
    Q q = robot->getQ();
    device->setQ(q, state);
    return q;
  }
  ////////////////// FUNCTION INVERSE KINEMATICS ///////////////////////

  rw::math::Q inverseKinematics(rw::math::Vector3D<double> pos,
                                RPY<double> rpy) {
    rw::math::Q q_inverse;
    rw::math::Transform3D<double> baseTtool_desired(pos, rpy.toRotation3D());
    auto solver = new rw::invkin::JacobianIKSolver(device, state);
    std::vector<rw::math::Q> solutions =
        solver->solve(baseTtool_desired, state);
    if (solutions.size() < 1) ROS_ERROR("Solver hasn't found a solution!!");
    if (solutions.size() > 0) {
      q_inverse = solutions[0];
    }
    return q_inverse;
  }
  ////////////////////////////////////////////////////////////////////////

  bool setQ(Q q) {
    // Tell robot to move to joint config q
    float speed = 1;
    if (robot->moveServoQ(q, speed)) {
      Q qCurrent;
      do {
        ros::spinOnce();
        qCurrent = robot->getQ();
      } while ((qCurrent - q).norm2() > 0.01);
      device->setQ(q, state);
      return true;
    } else
      return false;
  }

  bool setPose(rw::math::Vector3D<double> pos, RPY<double> angles) {
    ros::spinOnce();
    rw::kinematics::Frame *tool_frame = device->getEnd();

    if (tool_frame == nullptr) {
      RW_THROW("Tool frame not found!");
    }
    rw::math::Q solution = inverseKinematics(pos, angles);
    setQ(solution);

    return true;
  }

  rw::math::Transform3D<double> getPose() {
    ros::spinOnce();
    auto endFrame = device->getEnd();
    auto endTransform = device->baseTframe(endFrame, state);

    return endTransform;
  }

  bool checkCollisions(Device::Ptr device, const State &state,
                       const CollisionDetector &detector, const Q &q_new,
                       const Q &q_near, double extend) {
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    double epsilon = extend / 5;
    rw::math::Q delta_q = q_new - q_near;
    double norm = delta_q.norm2();
    int n = ceil(norm / (epsilon)) - 1;
    rw::math::Q q;
    for (int i = 1; i <= n; i++) {
      q = i * epsilon * (delta_q / norm) + q_near;
      testState = state;
      device->setQ(q, testState);
      colFrom = detector.inCollision(testState, &data);
      if (colFrom) {
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
        }
        return false;
      }
    }

    return true;
  }

  rw::math::Q nearest_neigbor(rw::math::Q q_rand,
                              std::vector<rw::math::Q> path) {
    double min = 10000;
    double dist = 0;
    Q q_nearest = path[0];
    for (size_t k = 0; k < path.size(); k++) {
      Q dq = q_rand - path[k];
      dist = dq.norm2();
      if (dist < min) {
        min = dist;
        q_nearest = path[k];
      }
    }
    return q_nearest;
  }

  bool NewConfig(rw::math::Q q_rand, rw::math::Q q_near, rw::math::Q &q_new) {
    CollisionDetector detector(
        wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());

    Q dq = q_rand - q_near;
    double extend = dq.norm2();
    q_new = q_rand;
    return checkCollisions(device, state, detector, q_new, q_near, extend);
    // cuando no choca -> true
  }

  void sendHome() { setQ(q_home); }

  bool setHomeAndReset(ur_caros_example::ResetQ::Request &req,
                       ur_caros_example::ResetQ::Response &res) {
    q_home = getQ();
    cout << "New Q_HOME: " << q_home << endl;
    res.success = true;
    return true;
  }
  RPY<double> getRPY() {
    auto robot_transform = getPose();
    return RPY<double>(robot_transform.R());
  }
  bool planner_rrt(ur_caros_example::Vision::Request &req,
                   ur_caros_example::Vision::Response &res) {
    // INIT TIMER
    auto start = std::chrono::steady_clock::now();
    double x = req.point.x, y = req.point.y, z = req.point.z;
    auto rpy = getRPY();
    rw::math::Vector3D<double> pos(x, y, z);

    rw::math::Q q_goal = inverseKinematics(pos, rpy);

    std::vector<rw::math::Q> path;
    path.push_back(getQ());
    rw::math::Q diferencia;

    double region_q = 0.15;

    int count_collision = 0;
    bool keep = false;
    do {
      Q q_near = nearest_neigbor(q_goal, path);

      Q q_new;

      if (NewConfig(q_goal, q_near, q_new)) {
        path.push_back(q_new);
      }

      diferencia = q_new - q_goal;

    } while (diferencia.norm2() > region_q);
    auto finish = std::chrono::steady_clock::now();
    auto time =
        std::chrono::duration_cast<std::chrono::milliseconds>(finish - start)
            .count();
    ulong planner_time_1 = time;

    for (QPath::iterator it = path.begin(); it < path.end(); it++) {
      // cout << *it << endl;
      // cout << "set Q " << *it << endl;
      setQ(*it);
      // cout << "x, y, z" << getPose().P() << endl;
    }
    res.success = true;
    auto finish2 = std::chrono::steady_clock::now();
    auto time2 =
        std::chrono::duration_cast<std::chrono::milliseconds>(finish2 - finish)
            .count();
    ulong planner_time_2 = time2;
    cout << "time 1 " << planner_time_1 << endl;
    cout << "time 2 " << planner_time_2 << endl;
    ros::Duration(0.5).sleep();
    sendHome();
    return true;
  }
  void initialize() {
    // q_home = getQ();
    t_home = getPose();
    // cout << "New Q_HOME: " << q_home << endl;
    // q robot real inicial
    /*rw::math::Q q_ini(6, -0.384695, -1.08527, 1.37812, -0.309625, 1.56999,
                    0.0178859);*/
    rw::math::Q q_ini(6, 0.103016, -1.55723, 1.60104, -0.0104208, 1.69122,
                      0.687377);

    // rw::math::Q q_ini(6,);
    //  rw::math::Q q_ini(6, 1.435 ,-1.040, -4.380, -0.883, 1.551, 0 );
    // q robot simulacion
    // rw::math::Q q_ini(6, -1.6007, -1.7271, -2.203, -0.808, 1.5951, -0.031);
    q_home = q_ini;
    sendHome();
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "URRobot");
  ros::NodeHandle n;
  URRobot robot(n);
  robot.initialize();
  ros::ServiceServer _vision_service =
      n.advertiseService("/vision_coordinates", &URRobot::planner_rrt, &robot);
  ros::ServiceServer _reset_service =
      n.advertiseService("/reset_home", &URRobot::setHomeAndReset, &robot);
  ros::spin();

  return 0;
}
