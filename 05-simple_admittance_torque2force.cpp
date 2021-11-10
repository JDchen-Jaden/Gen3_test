/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <DeviceManagerClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <Common.pb.h>

#include "utilities.h"

namespace k_api = Kinova::Api;

#define PORT 10000

///////////////////////////////////////////////////////////////////////////
// VAC test
#include <fstream>
#include <eigen3/Eigen/Dense> 

using Vector7 = Eigen::Matrix<double,7,1>; 
using Vector6 = Eigen::Matrix<double,6,1>;
using Vector3 = Eigen::Matrix<double,3,1>; 
///////////////////////////////////////////////////////////////////////////

#define DURATION 5             // Network timeout (seconds)
float time_duration = DURATION; // Duration of the example (seconds)

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_DURATION = std::chrono::seconds(20);

// Actuator speed (deg/s)
const float SPEED = 5.0f;

// Global Vars
 float L1= 0.1564,L=0.1284,L3= 0.2104,L4= 0.2104,L5 = 0.2084,L6 =0.1059,L7= 0.1059,L8= 0.0615+0.215, D1= 5.4e-3,D2 = 6.4e-3;
 float a = L3+L4;
 float b = L5+L6;
 float c = L7+L8;
 float d = D1+D2;

/*****************************
 * Example related function *
 *****************************/
int64_t GetTickUs()
{
#if defined(_MSC_VER)
    LARGE_INTEGER start, frequency;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);

    return (start.QuadPart * 1000000)/frequency.QuadPart;
#else
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000); // return microsecond
#endif
}
//=======================================================================================================
// User Defined Function  
//=======================================================================================================
//------------
//   J_3dof()
//------------
Eigen::Matrix3d J_3dof(Eigen::Vector3d q)
{
 double J11,J12,J13,J21,J22,J23;
 Eigen::Matrix3d J;
 double q1,q4,q6;
 q1 = q(0); q4 = q(1); q6 =q(2);
 //  Jacobian
 J11 = -d*cos(q1) - a*sin(q1) - b*sin(q1-q4) - c*sin(q1-q4-q6);
 J12 = b*sin(q1-q4) + c*sin(q1-q4-q6);
 J13 = c*sin(q1-q4-q6);
 J21 = d*sin(q1) - a*cos(q1) - b*cos(q1-q4) - c*cos(q1-q4-q6);
 J22 = b*cos(q1-q4) + c*cos(q1-q4-q6);
 J23 = c*cos(q1-q4-q6);

 J << J11,J12,J13,
      J21,J22,J23,
      -1,1,1;
 return J;
}
//*******************************************************************************************************
//   J_inv_3dof()
//*******************************************************************************************************
Eigen::Matrix3d J_inv_3dof(Eigen::Vector3d q)
{
 Eigen::Matrix3d IJ; 
 double IJ11,IJ12,IJ13,IJ21,IJ22,IJ23,IJ31,IJ32,IJ33;
 double q1,q4,q6;
 q1 = q(0); q4 = q(1); q6 =q(2);
 // Inverse Jacobian
 IJ11 = -cos(q1-q4)/(d*cos(q4) + a*sin(q4));
 IJ12 = sin(q1-q4)/(d*cos(q4) + a*sin(q4));
 IJ13 = -c*sin(q6)/(d*cos(q4) + a*sin(q4));
 IJ21 = -(a*cos(q1) + b*cos(q1-q4) - d*sin(q1))/(b*d*cos(q4) + a*b*sin(q4) );
 IJ22 = (d*cos(q1) + a*sin(q1) + b*sin(q1-q4))/(b*d*cos(q4) + a*b*sin(q4));
 IJ23 = -c*(d*cos(q4+q6) + b*sin(q6) + a*sin(q4+q6))/(b*d*cos(q4) + a*b*sin(q4));
 IJ31 = (a*cos(q1) - d*sin(q1))/(b*d*cos(q4) + a*b*sin(q4) );
 IJ32 = -(d*cos(q1) + a*sin(q1))/(b*d*cos(q4) + a*b*sin(q4) );
 IJ33 = (b*d*cos(q4) + c*d*cos(q4+q6) + a*b*sin(q4) + a*c*sin(q4+q6))/(b*d*cos(q4) + a*b*sin(q4) );

 IJ << IJ11,IJ12,IJ13,
      IJ21,IJ22,IJ23,
      IJ31,IJ32,IJ33;
 return IJ;
}
//*******************************************************************************************************
//   my_Trans()
//*******************************************************************************************************
Eigen::Vector3d my_Trans(Eigen::Vector3d q)
{
 Eigen::Vector3d T; 
 double T1,T2,R3;
 double q1,q4,q6;
 q1 = q(0); q4 = q(1); q6 =q(2);
 //  Translation and Rotation
  T1 = (L3+L4)*cos(q1)+(L5+L6)*cos(q1-q4)+(L7+L8)*cos(q1-q4-q6)-(D1+D2)*sin(q1);
  T2 = -(D1+D2)*cos(q1)-(L3+L4)*sin(q1)-(L5+L6)*sin(q1-q4)-(L7+L8)*sin(q1-q4-q6);
  R3 = -q1+q4+q6;
  
  T << T1,T2,R3;
 return T;
}

// Create an event listener that will set the promise action event to the exit value
// Will set to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)> 
    create_action_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

bool example_move_to_start_position(k_api::Base::BaseClient* base, Vector7 q0)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Move arm to ready position
    std::cout << "Moving the arm to the start position..." << std::endl;
    auto constrained_joint_angles = k_api::Base::ConstrainedJointAngles();
    auto joint_angles = constrained_joint_angles.mutable_joint_angles();

    auto actuator_count = base->GetActuatorCount();

    for (size_t i = 0; i < q0.rows(); ++i) 
    {
        auto joint_angle = joint_angles->add_joint_angles();
        joint_angle->set_joint_identifier(i);
        joint_angle->set_value(q0[i]);
    }

    // Connect to notification action topic
    std::promise<k_api::Base::ActionEvent> promise;
    auto future = promise.get_future();
    auto notification_handle = base->OnNotificationActionTopic(
        create_action_event_listener_by_promise(promise),
        k_api::Common::NotificationOptions{}
    );

    base->PlayJointTrajectory(constrained_joint_angles);

    // Wait for action to finish
    const auto status = future.wait_for(TIMEOUT_DURATION);
    base->Unsubscribe(notification_handle);

    if(status != std::future_status::ready)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        return false;
    }

    return true;
}


bool example_send_joint_speeds(k_api::Base::BaseClient* base, Vector7 qdot_cmd)
{
    k_api::Base::JointSpeeds joint_speeds;  
    int actuator_count = base->GetActuatorCount().count();

    if (actuator_count == 7)
    { 
            joint_speeds.clear_joint_speeds();
            for (size_t i = 0 ; i < qdot_cmd.rows(); ++i)
            {
                auto joint_speed = joint_speeds.add_joint_speeds();
                joint_speed->set_joint_identifier(i);
                joint_speed->set_value(qdot_cmd[i]);
                joint_speed->set_duration(1);
            }
            
            base->SendJointSpeedsCommand(joint_speeds);
    }
    return true;
}


int main(int argc, char **argv)
{

    auto parsed_args = ParseExampleArguments(argc, argv);

    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(parsed_args.username);
    create_session_info.set_password(parsed_args.password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating session for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    std::cout << "Session created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router);
    k_api::Base::JointSpeeds joint_speeds;  
    int actuator_count = base->GetActuatorCount().count();

    //===============================================================================================================================
    // Creat files to store data.
    std::ofstream my_time_data {"/home/jd-vmware/kortex/api_cpp/examples/My_Data/my_time.txt"};
    std::ofstream my_joint_po_data {"/home/jd-vmware/kortex/api_cpp/examples/My_Data/my_joint_po.txt"};
    std::ofstream my_joint_ve_data {"/home/jd-vmware/kortex/api_cpp/examples/My_Data/my_joint_ve.txt"};
    std::ofstream my_joint_tau_data {"/home/jd-vmware/kortex/api_cpp/examples/My_Data/my_joint_tau.txt"};
    std::ofstream my_EE_F_3_data {"/home/jd-vmware/kortex/api_cpp/examples/My_Data/my_EE_F_3.txt"};
    std::ofstream my_EE_F_6_data {"/home/jd-vmware/kortex/api_cpp/examples/My_Data/my_EE_F_6.txt"};
    std::ofstream my_EE_po_data {"/home/jd-vmware/kortex/api_cpp/examples/My_Data/my_EE_po.txt"};
    std::ofstream my_EE_ve_data {"/home/jd-vmware/kortex/api_cpp/examples/My_Data/my_EE_ve.txt"};
    std::ofstream my_EE_x_dot_r_3dof_data {"/home/jd-vmware/kortex/api_cpp/examples/My_Data/my_EE_x_dot_r_3dof.txt"};
    std::ofstream my_q_dot_r_data {"/home/jd-vmware/kortex/api_cpp/examples/My_Data/my_q_dot_r.txt"};
    //===============================================================================================================================

    //======================
    // My simple Admittance
    //======================
    // Variable Declaration
    //-------------------------------------------------------------------------------
    bool success = true;
    Vector3 q{}, q_dot{}, q_dot_r{};
    Vector7 q_dot_command;
    Vector3 joint_tau{};
    Vector6 EE_x{}, EE_x_dot{}, EE_F_6{};
    Vector3 EE_F_3{};
    Vector3 EE_x_3dof_compute{}, EE_x_dot_3dof_compute{}, EE_x_dot_r_3dof{}, EE_x_dot_3dof_compute_old{};

    Eigen::Matrix3d M{}, D{};
    M << 10.0,0.0,0.0,
         0.0,10.0,0.0,
         0.0,0.0,10.0;
    D << 400.0,0.0,0.0, 
         0.0,400.0,0.0,
         0.0,0.0,400.0;

    double loop_dur = 600.0;  //loop duration 10 minutes
    double t{0.0}, t0{0.0};           
    int64_t now = 0, start = 0;
    double h {0.0};  // time step

    // Move to 3 DPF config.
    //-------------------------------------------------
    Vector7 q0;
    q0 << -140.0, 90.0, -90.0, -40.0, 0.0, -20.0, 0.0;
    success &= example_move_to_start_position(base,q0);

    // Main Loop
    //-------------------------------------------------
    start = GetTickUs();

    while(t <= loop_dur)
    {
        now = GetTickUs();
        t = (double(now - start))/1000000; // to sec
        h = t - t0; // time step
        //****************** 
        // VAC computation
        //*******************
        // read actuators' information
        auto feedback = base_cyclic->RefreshFeedback();
        q[0] = feedback.actuators(0).position();   // in degree
        q[1] = feedback.actuators(3).position();
        q[2] = feedback.actuators(5).position();


        q_dot[0] = feedback.actuators(0).velocity();  // in degree/s
        q_dot[1] = feedback.actuators(3).velocity();
        q_dot[2] = feedback.actuators(5).velocity();


        joint_tau[0] = feedback.actuators(0).torque(); // in N.m
        joint_tau[1] = feedback.actuators(3).torque();
        joint_tau[2] = feedback.actuators(5).torque();


        EE_x[0] = feedback.base().tool_pose_x();  // in meter
        EE_x[1] = feedback.base().tool_pose_y();
        EE_x[2] = feedback.base().tool_pose_theta_z();;
        EE_x[3] = feedback.base().tool_pose_theta_x();  // in degree
        EE_x[4] = feedback.base().tool_pose_theta_y();
        EE_x[5] = feedback.base().tool_pose_theta_z();

        EE_x_dot[0] = feedback.base().tool_twist_linear_x();  // in meter/s
        EE_x_dot[1] = feedback.base().tool_twist_linear_y();
        EE_x_dot[2] = feedback.base().tool_twist_linear_z();
        EE_x_dot[3] = feedback.base().tool_twist_angular_x();  // in degree/s
        EE_x_dot[4] = feedback.base().tool_twist_angular_y();
        EE_x_dot[5] = feedback.base().tool_twist_angular_z();

        EE_F_6[0] = feedback.base().tool_external_wrench_force_x();  // in Newton
        EE_F_6[1] = feedback.base().tool_external_wrench_force_y();
        EE_F_6[2] = feedback.base().tool_external_wrench_force_z();
        EE_F_6[3] = feedback.base().tool_external_wrench_torque_x();  // in N.m
        EE_F_6[4] = feedback.base().tool_external_wrench_torque_y();
        EE_F_6[5] = feedback.base().tool_external_wrench_torque_z();

        EE_F_3 << EE_F_6(0),EE_F_6(1),EE_F_6(5);

        // convert to EE velocity
        EE_x_dot_3dof_compute = J_3dof(q)*q_dot;
        // convert to EE position
        EE_x_3dof_compute = my_Trans(q);
        // sovling a simple admittance equation: M*x_2dot + D*x_dot = F
        EE_x_dot_r_3dof = EE_x_dot_3dof_compute_old + M.inverse()*(EE_F_3 - D*EE_x_dot_3dof_compute)*h;
        // simple inverse kinematics
	    q_dot_r = J_inv_3dof(q)*EE_x_dot_r_3dof;

        q_dot_command << q_dot_r(0), 0.0, 0.0, q_dot_r(1), 0.0, q_dot_r(2), 0.0;

        // Send velocity command
        success &=example_send_joint_speeds(base, q_dot_command);
        /* copy from  example_send_joint_speeds()  (Note: executing this block will reduce 25 ms) 
            joint_speeds.clear_joint_speeds();
            for (size_t i = 0 ; i < q_dot_command.rows(); ++i)
            {
                auto joint_speed = joint_speeds.add_joint_speeds();
                joint_speed->set_joint_identifier(i);
                joint_speed->set_value(q_dot_command[i]);
                joint_speed->set_duration(1);
            }  
            base->SendJointSpeedsCommand(joint_speeds);
        */
        // Write data to file
        my_time_data << now <<" " << t <<" " << h <<std::endl;
        my_joint_po_data <<q(0) <<" "<<q(1) <<" "<<q(2) <<std::endl;
        my_joint_ve_data <<q_dot(0) <<" "<<q_dot(1) <<" "<<q_dot(2) <<std::endl;
        my_joint_tau_data << joint_tau(0) <<" "<< joint_tau(1) <<" "<< joint_tau(2) <<std::endl;
        my_EE_F_3_data <<EE_F_3(0)<<" "<<EE_F_3(1)<<" "<<EE_F_3(2)<<std::endl;
        my_EE_F_6_data <<EE_F_6(0)<<" "<<EE_F_6(1)<<" "<<EE_F_6(2)<<" "<<EE_F_6(3)<<" "<<EE_F_6(4)<<" "<<EE_F_6(5)<<std::endl;
        my_EE_po_data << EE_x(0) << " " << EE_x(1) << " " << EE_x(2) << " " << EE_x(3) << " " << EE_x(4) << " " << EE_x(5) <<std::endl;
        my_EE_ve_data << EE_x_dot(0) << " " << EE_x_dot(1) << " " << EE_x_dot(2) << " " << EE_x_dot(3) << " " << EE_x_dot(4) << " " << EE_x_dot(5) <<std::endl;
        my_EE_x_dot_r_3dof_data << EE_x_dot_r_3dof(0) << " " << EE_x_dot_r_3dof(1) << " " << EE_x_dot_r_3dof(2) <<std::endl;
        my_q_dot_r_data << q_dot_r(0) << " " << q_dot_r(1) << " " << q_dot_r(2) <<std::endl;

        EE_x_dot_3dof_compute_old = EE_x_dot_3dof_compute;
        t0 = t;
    } // while(loop_dur <= t)


    // Close API session
    session_manager->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();

    // Destroy the API
    delete base;
    delete session_manager;
    delete router;
    delete transport;

    return success ? 0: 1;
}; // int main()

