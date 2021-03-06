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
/*This code is used for object avoidance and testing with simulator more than
 * anything. Please reffer to the other file attached for code used in real
 * trials*/
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
  double extend = 0.4;

 public:
  URRobot(ros::NodeHandle n) : nh(n) {
    auto packagePath = ros::package::getPath("ur_caros_example");
    wc = rw::loaders::WorkCellLoader::Factory::load(
        packagePath + "/WorkStation_1_modificada/WC1_Scene.wc.xml");
    device = wc->findDevice("UR1");
    state = wc->getDefaultState();
    robot = new caros::SerialDeviceSIProxy(nh, "caros_universalrobot");

    // Wait for first state message, to make sure robot is ready
    ros::topic::waitForMessage<caros_control_msgs::RobotState>(
        "/caros_universalrobot/"
        "caros_serial_device_service_interface/"
        "robot_state",
        nh);
    // init q_home to its basic configuration
    // q_home = {};
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
      // cout << "Q inverse " << q_inverse << endl;
    }
    return q_inverse;
  }
  ////////////////////////////////////////////////////////////////////////

  bool setQ(Q q) {
    // Tell robot to move to joint config q
    float speed = 1;
    /*std::cout << "dentro de setQ .>  q :" << std::endl << q << std::endl <<
     * std::endl;*/
    if (robot->moveServoQ(q, speed)) {
      // std::cout << "If setQ :" << std::endl;
      // moveServoQ terminates before the robot is at the destination, so wait
      // until it is
      Q qCurrent;
      do {
        // std::cout << "Current joint config:" << std::endl << robot->getQ() <<
        // std::endl << std::endl;
        ros::spinOnce();
        qCurrent = robot->getQ();
      } while ((qCurrent - q).norm2() > 0.01);
      // std::cout << "antes de device (SET Q) " << std::endl;
      device->setQ(q, state);
      return true;
    } else
      return false;
  }

  bool setPose(rw::math::Vector3D<double> pos, RPY<double> angles) {
    ros::spinOnce();
    // std::cout <<"RPY UNO: "<<angles[0]<<" "<<angles[1]<<"
    // "<<angles[2]<<std::endl;  std::cout <<"POS UNO: "<<pos[0]<<" "<<pos[1]<<"
    // "<<pos[2]<<std::endl;
    rw::kinematics::Frame *tool_frame =
        device->getEnd();  // = wc->findFrame("PG70.TCP");
                           //
    /*std::vector<rw::kinematics::Frame*> frames = wc->getFrames();

    for(int i = 0; i < frames.size(); i++){
            std::cout<<frames[i]->name<<std::endl;
    }*/
    if (tool_frame == nullptr) {
      RW_THROW("Tool frame not found!");
    }
    rw::math::Q solution = inverseKinematics(pos, angles);
    setQ(solution);
    /*
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
    for (size_t k = 0; k < solution.size(); k++) {
      std::cout << "solution solve inkin " << k << " -> " << solution[k]
                << std::endl;
    }
    if (solution.size() > 0) {

    }*/

    /*rw::math::Q q_algo =
        algorithm1(device, state, tool_frame, baseTtool_desired, getQ());*/

    return true;
  }

  rw::math::Transform3D<double> getPose() {
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
    auto endFrame = device->getEnd();  // wc->findFrame("PG70.TCP");
    auto endTransform = device->baseTframe(endFrame, state);
    // cout << "end transform . R " <<endTransform.R()<< endl;
    /*auto rpy = RPY<double>(endTransform.R());
    auto pos = endTransform.P();
    std::cout << "POS: " << pos[0] << " " << pos[1] << " " << pos[2] <<
    std::endl; std::cout << "RPY: " << rpy[2] << " " << rpy[1] << " " << rpy[0]
    << std::endl; cout << "RPY_intercambiado -> debe coincidir con los valores
    del simulador" << endl;*/
    return endTransform;
  }

  bool checkCollisions(Device::Ptr device, const State &state,
                       const CollisionDetector &detector, const Q &q_new,
                       const Q &q_near, double extend) {
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    // algoritmo 1 check collision // dividimos el edge en 3 y comprobamos esos
    // tres puntos
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
        //  cerr << "Configuration in collision: " << q << endl;
        // cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
          //    cerr << (*it).first->getName() << " " << (*it).second->getName()
          //       << endl;
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
    q_new = q_near + extend * (dq) / (dq.norm2());
    return checkCollisions(device, state, detector, q_new, q_near, extend);
    // cuando no choca -> true
  }

  void sendHome() { setQ(q_home); }

  bool setHomeAndReset(ur_caros_example::ResetQ::Request &req,
                       ur_caros_example::ResetQ::Response &res) {
    /*if (req.data) {
      q_home = getQ();
      // cout << "New Q_HOME: " << q_home << endl;
      sendHome();
    }*/
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
    rw::math::Q dif1;
    rw::math::Q dif2;
    rw::math::Q dif3;
    rw::math::Q dif4;
    rw::math::Q deltaQ_max;
    rw::math::Q deltaQ_min;

    // double region = 0.1;
    double region = 0.05;
    double region_q = 0.15;
    int fit = 0;
    int max_iteration;

    rw::math::Vector3D<double> pos1(x + region, y + region, z);
    rw::math::Vector3D<double> pos2(x - region, y + region, z);
    rw::math::Vector3D<double> pos3(x - region, y - region, z);
    rw::math::Vector3D<double> pos4(x + region, y - region, z);

    rw::math::Q q1_limit = inverseKinematics(pos1, rpy);
    rw::math::Q q2_limit = inverseKinematics(pos2, rpy);
    rw::math::Q q3_limit = inverseKinematics(pos3, rpy);
    rw::math::Q q4_limit = inverseKinematics(pos4, rpy);

    auto qMin = device->getBounds().first;
    auto qMax = device->getBounds().second;
    // rw::math::Q q_zero(6, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05);
    /* FORWARD KINEMATICS
      State testState;
[100%] Linking CXX executable /home/noelia/cat
      testState = state;

      device->setQ(qMin, testState);
      auto endFrame = device->getEnd();
      auto endTransform = device->baseTframe(endFrame, testState);
      auto pos_min = endTransform.P();

      device->setQ(qMax, testState);
      endTransform = device->baseTframe(endFrame, testState);
      auto pos_max = endTransform.P();*/

    int count_collision = 0;
    bool keep = false;
    max_iteration = 100000;
    do {
      keep = false;
      // TODO Set as constants
      auto x_rand = x - 0.01 +
                    static_cast<float>(rand()) /
                        (static_cast<float>(RAND_MAX / (0.01 + 0.01)));
      auto y_rand = y - 0.01 +
                    static_cast<float>(rand()) /
                        (static_cast<float>(RAND_MAX / (0.01 + 0.01)));
      auto z_rand = z - 0.01 +
                    static_cast<float>(rand()) /
                        (static_cast<float>(RAND_MAX / (0.01 + 0.01)));

      rw::math::Vector3D<double> pos_rand(x_rand, y_rand, z_rand);

      rw::math::Q q_rand = inverseKinematics(pos_rand, rpy);

      if (count_collision > 3) {
        deltaQ_max = qMax - q_rand;
        deltaQ_min = q_rand - qMin;

        auto min_delta_norm = (q_rand - qMin).norm2();
        auto max_delta_norm = (qMax - q_rand).norm2();
        auto min_delta =
            max_delta_norm < min_delta_norm ? deltaQ_max : deltaQ_min;

        q_rand = Math::ranQ(q_rand - min_delta, q_rand + min_delta);

        for (int i = 0; i < 6; i++) {
          if (q_rand[i] > 2 * 3.1415) {
            q_rand[i] -= 2 * 3.1415;
            q_rand[i] *= -1;
          } else if (q_rand[i] < -2 * 3.1415) {
            q_rand[i] += 2 * 3.1415;
            q_rand[i] *= -1;
          }
        }
      }

      Q q_near = nearest_neigbor(q_rand, path);
      Q q_new;

      if (NewConfig(q_rand, q_near, q_new)) {
        path.push_back(q_new);
        cout << " count collison " << count_collision << endl;
        count_collision = 0;
      } else {
        // if no new configuration
        keep = true;
        count_collision++;
      }

      diferencia = q_new - q_goal;
      dif1 = q_new - q1_limit;
      dif2 = q_new - q2_limit;
      dif3 = q_new - q3_limit;
      dif4 = q_new - q4_limit;

      if (!keep &&
          (diferencia.norm2() < 0.25 || dif1.norm2() < 0.25 ||
           dif2.norm2() < 0.25 || dif3.norm2() < 0.25 || dif4.norm2() < 0.25)) {
        fit++;
        if (fit > 5) {
          extend -= 0.01;
          fit = 0;
          ROS_INFO("FIT Changed!");
        }
      }
      max_iteration--;
    } while ((diferencia.norm2() > region_q && dif1.norm2() > region_q &&
              dif2.norm2() > region_q && dif3.norm2() > region_q &&
              dif4.norm2() > region_q) ||
             keep && max_iteration > 0);
    auto finish = std::chrono::steady_clock::now();
    auto time =
        std::chrono::duration_cast<std::chrono::milliseconds>(finish - start)
            .count();
    ulong planner_time = time;
    cout << "Time: " << planner_time << endl;
    cout << "Path of length: " << path.size() << endl;
    cout << "Iteration: " << max_iteration << endl;
    cout << "Extend: " << extend << endl;

    for (QPath::iterator it = path.begin(); it < path.end(); it++) {
      setQ(*it);
      cout << "x, y, z" << getPose().P() << endl;
    }
    res.success = true;

    return true;
  }
  void initialize() {
    t_home = getPose();

    // rw::math::Q q_ini(6,);
    rw::math::Q q_ini(6, 1.435, -1.040, -4.380, -0.883, 1.551, 0);
    // q robot
    // rw::math::Q q_ini(6, -1.6007, -1.7271, -2.203, -0.808, 1.5951, -0.031);
    // rw::math::Q q_ini(6,1.435, -1.040, -4.380 ,-0.883, 2.295, 0 );

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
  //_reset_subscriber = nh.subscribe("/reset_home", 10,
  //&URRobot::setHomeAndReset,this);
  ros::ServiceServer _reset_service =
      n.advertiseService("/reset_home", &URRobot::setHomeAndReset, &robot);
  ros::spin();
  // Otherwise we can forget to set the device or
  // something..
  return 0;
}
