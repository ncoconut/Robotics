#include <iostream>
#include "caros/serial_device_si_proxy.h"
#include "ros/package.h"
#include "rw/rw.hpp"
//#include "rw/invkin.hpp"
#include <rw/invkin/JacobianIKSolver.hpp>

using namespace rw::math;
using namespace std;
class URRobot {
  using Q = rw::math::Q;

 private:
  ros::NodeHandle nh;
  rw::models::WorkCell::Ptr wc;
  rw::models::Device::Ptr device;
  rw::kinematics::State state;
  caros::SerialDeviceSIProxy *robot;
  Q q_home;
  rw::invkin::JacobianIKSolver *solver;

 public:
  URRobot() {
    auto packagePath = ros::package::getPath("ur_caros_example");
    wc = rw::loaders::WorkCellLoader::Factory::load(packagePath +
                                                    "/WorkStation_1/WC1_Scene.wc.xml");
    device = wc->findDevice("UR1");
    state = wc->getDefaultState();
    robot = new caros::SerialDeviceSIProxy(nh, "caros_universalrobot");

    // Wait for first state message, to make sure robot is ready
    ros::topic::waitForMessage<caros_control_msgs::RobotState>(
        "/caros_universalrobot/caros_serial_device_service_interface/"
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

  bool setQ(Q q) {
    // Tell robot to move to joint config q
    float speed = 1;
    std::cout << "dentro de setQ .>  q :" << std::endl
              << q << std::endl
              << std::endl;
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

  //////////////////////////////////////////////////////////////////////////////

  rw::math::VelocityScrew6D<double> calculateDeltaU(
      const rw::math::Transform3D<double> &baseTtool,
      const rw::math::Transform3D<double> &baseTtool_desired) {
    // Calculate the positional difference, dp
    // cout << "dentro calcula de delta U " << endl;
    rw::math::Vector3D<double> dp = baseTtool_desired.P() - baseTtool.P();

    // Calculate the rotational difference, dw
    rw::math::EAA<double> dw(baseTtool_desired.R() *
                             rw::math::inverse(baseTtool.R()));

    return rw::math::VelocityScrew6D<double>(dp, dw);
  }

  rw::math::Q algorithm1(const rw::models::Device::Ptr device,
                         rw::kinematics::State state,
                         const rw::kinematics::Frame *tool,
                         const rw::math::Transform3D<double> baseTtool_desired,
                         rw::math::Q q) {
    // We need an initial base to tool transform and the positional error at the
    // start (deltaU)
    rw::math::Transform3D<> baseTtool = device->baseTframe(tool, state);
    rw::math::VelocityScrew6D<double> deltaU =
        calculateDeltaU(baseTtool, baseTtool_desired);

    // This epsilon is the desired tolerance on the final position.
    const double epsilon = 0.1;
   
    while (deltaU.norm2() > epsilon) {
      rw::math::Jacobian J = device->baseJframe(
          tool, state);  // This line does the same as the function from
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
      device->setQ(q,state);

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

  bool setPose(rw::math::Vector3D<double> pos, RPY<double> angles) {
    ros::spinOnce();
    // std::cout <<"RPY UNO: "<<angles[0]<<" "<<angles[1]<<"
    // "<<angles[2]<<std::endl;  std::cout <<"POS UNO: "<<pos[0]<<" "<<pos[1]<<"
    // "<<pos[2]<<std::endl;
    rw::kinematics::Frame *tool_frame = wc->findFrame("UR1.TCP");
    /*std::vector<rw::kinematics::Frame*> frames = wc->getFrames();

    for(int i = 0; i < frames.size(); i++){
            std::cout<<frames[i]->name<<std::endl;
    }*/
    if (tool_frame == nullptr) {
      RW_THROW("Tool frame not found!");
    }
    rw::math::Transform3D<double> baseTtool_desired(pos, angles.toRotation3D());
    // The inverse kinematics algorithm needs to know about the device, the tool
    // frame and the desired pose. These parameters are const since they are not
    // changed by inverse kinematics We pass the state and the configuration, q,
    // as value so we have copies that we can change as we want during the
    // inverse kinematics.


    solver = new rw::invkin::JacobianIKSolver(device, state);
    //auto solver = rw::invkin::JacobianIKSolver(device, state);
    //auto solution = solver.solve(baseTtool_desired,state);
    std::vector<rw::math::Q> solution = solver->solve(baseTtool_desired, state);
    for(int k=0; k<solution.size(); k++)
    std::cout << "solution solve inkin " << k << " -> "<< solution[k] << std::endl;
    /*rw::math::Q q_algo =
        algorithm1(device, state, tool_frame, baseTtool_desired, getQ());*/
    /*if(solution.size()==1){
       device->setQ(solution[0],state);
       cout << "size = 1 setQ device" << endl; 
        cout << "haciendo robot. setQ"<<endl;
      // robot->setQ(solution[0]);
    }*/
   

    return true;
  }

  bool getPose() {
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
    auto endFrame = wc->findFrame("UR1.TCP");;
    auto endTransform = device->baseTframe(endFrame, state);
    auto rpy = RPY<double>(endTransform.R());
    auto pos = endTransform.P();
    std::cout << "RPY UNO: " << rpy[0] << " " << rpy[1] << " " << rpy[2]
              << std::endl;
    std::cout << "POS UNO: " << pos[0] << " " << pos[1] << " " << pos[2]
              << std::endl;

    /*endFrame = endFrame->getParent();
    //auto endFrame = tcp;
    endTransform = endFrame->wTf(state);
    rpy = RPY<double>(endTransform.R());
    pos = endTransform.P();
    std::cout <<"RPY DOS: "<<rpy[0]<<" "<<rpy[1]<<" "<<rpy[2]<<std::endl;
    std::cout <<"POS DOS: "<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;

    endFrame = endFrame->getParent();
    //auto endFrame = tcp;
    endTransform = endFrame->wTf(state);
    rpy = RPY<double>(endTransform.R());
    pos = endTransform.P();
    std::cout <<"RPY TREES: "<<rpy[0]<<" "<<rpy[1]<<" "<<rpy[2]<<std::endl;
    std::cout <<"POS TRES: "<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;*/

    return true;
  }

  /*bool moveHome() {
    device->setQ(q_home);
    ros::spinOnce();
    return true;
  }*/
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "URRobot");
  URRobot robot;

  std::cout << "Current joint config:" << std::endl
            << robot.getQ() << std::endl
            << std::endl;

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
  
  std::cout << "POSICIONES Y ANGULOS INPUT:" << std::endl;
  float x, y, z, r, p, yaw;
  std::cin >> x >> y >> z >> r >> p >> yaw;
  rw::math::Vector3D<double> pos(x, y, z);
  auto rpy = RPY<double>(r, p, yaw);
  //std::cout << "datos recogidos, antes set pose" << endl;
  robot.setPose(pos, rpy);
  cout << std::endl << "final set pose  " << endl;
  robot.getPose();
  float q1, q2, q3, q4, q5, q6;
  cout << "introduce Q solution" << endl;
  std::cin >> q1 >> q2 >> q3 >> q4 >> q5 >> q6;
  rw::math::Q Q_in(6, q1, q2, q3, q4, q5, q6);
  
  robot.setQ(Q_in);

  return 0;
}
