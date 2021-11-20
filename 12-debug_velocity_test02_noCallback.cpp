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

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <cmath>

#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include <ActuatorConfigClientRpc.h>

#include <google/protobuf/util/json_util.h>

#include "utilities.h"

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>
///////////////////////////////////////////////////////////////////////////
// eigen
#include <fstream>
#include <eigen3/Eigen/Dense> 

using Vector7 = Eigen::Matrix<float,7,1>; 
using Vector6 = Eigen::Matrix<float,6,1>;
using Vector3 = Eigen::Matrix<float,3,1>; 
///////////////////////////////////////////////////////////////////////////

namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_REAL_TIME 10001

#define DURATION 5             // Network timeout (seconds)

float velocity = 5.0f;         // Default velocity of the actuator (degrees per seconds)
float time_duration = DURATION; // Duration of the example (seconds)

// Waiting time during actions
const auto ACTION_WAITING_TIME = std::chrono::seconds(1);
// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_DURATION = std::chrono::seconds(20);

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)> 
    create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
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
    std::promise<k_api::Base::ActionEvent> finish_promise;
    auto finish_future = finish_promise.get_future();
    auto promise_notification_handle = base->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise),
        k_api::Common::NotificationOptions()
    );

    base->PlayJointTrajectory(constrained_joint_angles);

    // Wait for action to finish
    const auto status = finish_future.wait_for(TIMEOUT_DURATION);
    base->Unsubscribe(promise_notification_handle);

    if(status != std::future_status::ready)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        return false;
    }

    return true;
}

//*********************
// Global Vars
//*********************
    Vector3 q{}, q_dot{};
    Vector7 q_dot_command{}, q_cmd{};
    Vector3 joint_tau{};

    double loop_dur = 5.0;  //loop duration 20 s
    double t{0.0}, t0{0.0};           
    int64_t now = 0, start = 0, last =0, my_sleep = 0;
    double h {0.0};  // time step 
///////////////////////////////////////////////////////////////
// call back functions
void my_vac(const k_api::BaseCyclic::Feedback feedback){

        // read actuators' information
        q[0] = feedback.actuators(0).position()*M_PI/180.0;   // in rad
        q[1] = feedback.actuators(3).position()*M_PI/180.0;
        q[2] = feedback.actuators(5).position()*M_PI/180.0;

        q_dot[0] = feedback.actuators(0).velocity()*M_PI/180.0;  // in rad/s
        q_dot[1] = feedback.actuators(3).velocity()*M_PI/180.0;
        q_dot[2] = feedback.actuators(5).velocity()*M_PI/180.0;

        joint_tau[0] = feedback.actuators(0).torque(); // in N.m
        joint_tau[1] = feedback.actuators(3).torque();
        joint_tau[2] = feedback.actuators(5).torque();

        //q_dot_command << -5.0, 0.0, 0.0, 5.0, 0.0, 5.0, 0.0;//q_dot_command = {5.0f, 0.0f, 0.0f, 5.0f, 0.0f, 5.0f, 0.0f};//
}

void my_callback(const k_api::Error& err,const k_api::BaseCyclic::Feedback& feedback)
{
    my_vac(feedback);
}



int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);

    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    
    std::cout << "Creating transport objects" << std::endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);

    std::cout << "Creating transport real time objects" << std::endl;
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(parsed_args.ip_address, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(parsed_args.username);
    create_session_info.set_password(parsed_args.password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);

    // Move to start position 
    try
    {
        base->ClearFaults();
    }
    catch(...)
    {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }

    Vector7 q0;
    q0 << -140.0, 90.0, -90.0, -40.0, 0.0, -20.0, 0.0;
    example_move_to_start_position(base,q0);

    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "Ready to interact with the robot." << std::endl;
    try
    {
        base->ClearFaults();
    }
    catch(...)
    {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }

    k_api::BaseCyclic::Feedback feedback;
    k_api::BaseCyclic::Command  base_command;
    auto servoingMode = k_api::Base::ServoingModeInformation();

    try{

         // Set the base in low-level servoing mode
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        feedback = base_cyclic->RefreshFeedback();
        int actuator_count = base->GetActuatorCount().count();

        for(int i = 0; i < actuator_count; i++)
        {
            q_cmd[i] = feedback.actuators(i).position();
            base_command.add_actuators()->set_position(feedback.actuators(i).position());
        }

        feedback = base_cyclic->Refresh(base_command);

        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::VELOCITY);

        actuator_config->SetControlMode(control_mode_message,1);
        actuator_config->SetControlMode(control_mode_message,4);
        actuator_config->SetControlMode(control_mode_message,6);

        q_dot_command << -5.0, 0.0, 0.0, 5.0, 0.0, 5.0, 0.0;
        start = GetTickUs();
        while(t <= loop_dur)
        {
            now = GetTickUs();
            t = (double(now - start))/1000000; // to sec
            h = t - t0; // time step
            std::cout<<t<<std::endl;
            if(now - last > 1000)
            {
                for(int i = 0; i < 7; i++)
                {
                    if(i==0 || i==3 || i ==5)
                    {
                    base_command.mutable_actuators(i)->set_position(feedback.actuators(i).position());
                    base_command.mutable_actuators(i)->set_velocity(q_dot_command[i]);
                    }
                }

                base_command.set_frame_id(base_command.frame_id() + 1);
                if (base_command.frame_id() > 65535)
                    base_command.set_frame_id(0);

                for (int idx = 0; idx < actuator_count; idx++)
                {
                    base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
                }

                try
                {
                    feedback = base_cyclic->Refresh(base_command, 0);
                }
                catch (k_api::KDetailedException& ex)
                {
                    std::cout << "Kortex exception: " << ex.what() << std::endl;

                    std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
                }
                catch (std::runtime_error& ex2)
                {
                    std::cout << "runtime error: " << ex2.what() << std::endl;
                }
                catch(...)
                {
                    std::cout << "Unknown error." << std::endl;
                }
                last = GetTickUs();
                t0 = t;
            }
        } // while(loop_dur <= t)
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message,1);
        actuator_config->SetControlMode(control_mode_message,4);
        actuator_config->SetControlMode(control_mode_message,6);
    }

    catch (k_api::KDetailedException& ex)
    {
        std::cout << "API error: " << ex.what() << std::endl;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Error: " << ex2.what() << std::endl;
    }
        // Set the servoing mode back to Single Level
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete actuator_config;
    delete transport;
    delete transport_real_time;
}
