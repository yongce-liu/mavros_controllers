/****************************************************************************
 *
 *   Copyright (c) 2018-2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Geometric Controller
 *
 * Geometric controller
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "geometric_controller/geometric_controller.h"

#include "geometric_controller/jerk_tracking_control.h"
#include "geometric_controller/nonlinear_attitude_control.h"
#include "geometric_controller/nonlinear_geometric_control.h"

using namespace Eigen;
using namespace std;
// Constructor
geometricCtrl::geometricCtrl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), node_state(WAITING_FOR_HOME_POSE) {
    referenceSub_ = nh_.subscribe("reference/setpoint", 1, &geometricCtrl::targetCallback,
                                  this, ros::TransportHints().tcpNoDelay());
    flatreferenceSub_ =
        nh_.subscribe("reference/flatsetpoint", 1, &geometricCtrl::flattargetCallback,
                      this, ros::TransportHints().tcpNoDelay());
    yawreferenceSub_ =
        nh_.subscribe("reference/yaw", 1, &geometricCtrl::yawtargetCallback, this,
                      ros::TransportHints().tcpNoDelay());
    multiDOFJointSub_ =
        nh_.subscribe("command/trajectory", 1, &geometricCtrl::multiDOFJointCallback,
                      this, ros::TransportHints().tcpNoDelay());
    mavstateSub_ = nh_.subscribe("/mavros/state", 1, &geometricCtrl::mavstateCallback,
                                 this, ros::TransportHints().tcpNoDelay());
    mavposeSub_ =
        nh_.subscribe("/mavros/local_position/pose", 1, &geometricCtrl::mavposeCallback,
                      this, ros::TransportHints().tcpNoDelay());
    mavtwistSub_ = nh_.subscribe("/mavros/local_position/velocity_local", 1,
                                 &geometricCtrl::mavtwistCallback, this,
                                 ros::TransportHints().tcpNoDelay());
    ctrltriggerServ_ = nh_.advertiseService("trigger_rlcontroller",
                                            &geometricCtrl::ctrltriggerCallback, this);
    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &geometricCtrl::cmdloopCallback,
                                     this); // Define timer for constant loop rate
    statusloop_timer_ =
        nh_.createTimer(ros::Duration(1), &geometricCtrl::statusloopCallback,
                        this); // Define timer for constant loop rate
    hold_before_land_timer_ = nh_.createTimer(
        ros::Duration(2.5),
        [this](const ros::TimerEvent&) {
            this->setFlightState(LANDING);
            ROS_INFO("HOLD phase finished, switching to LAND");
        },
        true, false);

    angularVelPub_ =
        nh_.advertise<mavros_msgs::AttitudeTarget>("command/bodyrate_command", 1);
    referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 1);
    target_pose_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    posehistoryPub_ = nh_.advertise<nav_msgs::Path>("geometric_controller/path", 10);
    tracking_error_history_pub_ =
        nh_.advertise<std_msgs::Float32>("geometric_controller/tracking_error", 10);
    systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>(
        "/mavros/companion_process/status", 1);
    homePosePub_ = nh_.advertise<geometry_msgs::Pose>("home_pose", 10, true);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // set_mode_client_ =
    // nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    takeoff_service_ =
        nh_.advertiseService("takeoff", &geometricCtrl::takeoffCallback, this);
    // takeoff_client_ =
    // nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    land_service_ = nh_.advertiseService("land", &geometricCtrl::landCallback, this);
    land_client = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    hold_service_ = nh_.advertiseService("hold", &geometricCtrl::holdCallback, this);
    mission_service_ =
        nh_.advertiseService("mission", &geometricCtrl::missionCallback, this);
    goto_service_ = nh_.advertiseService("goto", &geometricCtrl::gotoCallback, this);
    trajectory_trigger_client_ = nh_.serviceClient<std_srvs::SetBool>(
        "command/trajectory_start"); // built-in traj trigger
    gcopter_traj_trigger_client_ =
        nh_.serviceClient<gcopter::getTraj>("get_traj"); // gcopter traj trigger

    /**** Param Init *****/
    // nh_private_.param<string>("mavname", mav_name_, "iris");
    nh_private_.param<int>("ctrl_mode", ctrl_mode_, ERROR_QUATERNION);
    // nh_private_.param<bool>("enable_sim", sim_enable_, true);
    nh_private_.param<bool>("velocity_yaw", velocity_yaw_, false);
    nh_private_.param<double>("max_acc", max_fb_acc_, 9.0);
    nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);

    double dx, dy, dz;
    nh_private_.param<double>("drag_dx", dx, 0.0);
    nh_private_.param<double>("drag_dy", dy, 0.0);
    nh_private_.param<double>("drag_dz", dz, 0.0);
    D_ << dx, dy, dz;

    double attctrl_tau;
    nh_private_.param<double>("attctrl_constant", attctrl_tau, 0.1);
    nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const_,
                              0.05); // 1 / max acceleration
    nh_private_.param<double>("normalizedthrust_offset", norm_thrust_offset_,
                              0.1); // 1 / max acceleration
    nh_private_.param<double>("normalizedthrust_max", norm_thrust_max_, 0.5);
    nh_private_.param<double>("Kp_x", Kpos_x_, 8.0);
    nh_private_.param<double>("Kp_y", Kpos_y_, 8.0);
    nh_private_.param<double>("Kp_z", Kpos_z_, 10.0);
    nh_private_.param<double>("Kv_x", Kvel_x_, 1.5);
    nh_private_.param<double>("Kv_y", Kvel_y_, 1.5);
    nh_private_.param<double>("Kv_z", Kvel_z_, 3.3);
    nh_private_.param<int>("posehistory_window", posehistory_window_, 200000);
    nh_private_.param<int>("tracking_error_history_window",
                           tracking_error_history_window_, 200000);
    // nh_private_.param<double>("init_pos_x", initTargetPos_x_, 0.0);
    // nh_private_.param<double>("init_pos_y", initTargetPos_y_, 0.0);
    // nh_private_.param<double>("init_pos_z", initTargetPos_z_, 1.0);

    // targetPos_ << initTargetPos_x_, initTargetPos_y_,
    // initTargetPos_z_;  // Initial Position
    targetPos_ << 0.0, 0.0, 0.0; // Initial Position
    targetVel_ << 0.0, 0.0, 0.0;
    mavPos_ << 0.0, 0.0, 0.0;
    mavVel_ << 0.0, 0.0, 0.0;
    Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
    Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;

    if (ctrl_mode_ == ERROR_GEOMETRIC) {
        controller_ = std::make_shared<NonlinearGeometricControl>(attctrl_tau);
    } else if (ctrl_mode_ == ERROR_JERK) {
        controller_ = std::make_shared<JerkTrackingControl>();
    } else {
        controller_ = std::make_shared<NonlinearAttitudeControl>(attctrl_tau);
    }
}
geometricCtrl::~geometricCtrl() {
    // Destructor
}

void geometricCtrl::pubHoldPose(const Vector3d& hold_point) {
    geometry_msgs::PoseStamped hold_pose;
    hold_pose.header.stamp = ros::Time::now();
    hold_pose.header.frame_id = "map";
    hold_pose.pose.position = toGeoPoint(hold_point);
    target_pose_pub_.publish(hold_pose);
}

void geometricCtrl::targetCallback(const geometry_msgs::TwistStamped& msg) {
    // ROS_INFO_STREAM("Received TwistStamped message");
    reference_request_last_ = reference_request_now_;
    targetPos_prev_ = targetPos_;
    targetVel_prev_ = targetVel_;

    reference_request_now_ = ros::Time::now();
    reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

    targetPos_ = toEigen(msg.twist.angular);
    targetVel_ = toEigen(msg.twist.linear);

    if (reference_request_dt_ > 0)
        targetAcc_ = (targetVel_ - targetVel_prev_) / reference_request_dt_;
    else
        targetAcc_ = Eigen::Vector3d::Zero();
}

void geometricCtrl::flattargetCallback(const controller_msgs::FlatTarget& msg) {
    // ROS_INFO_STREAM("Received FlatTarget message");
    received_target_pose = true;
    reference_request_last_ = reference_request_now_;

    targetPos_prev_ = targetPos_;
    targetVel_prev_ = targetVel_;

    reference_request_now_ = ros::Time::now();
    reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

    targetPos_ = toEigen(msg.position);
    targetVel_ = toEigen(msg.velocity);

    if (msg.type_mask == 1) {
        targetAcc_ = toEigen(msg.acceleration);
        targetJerk_ = toEigen(msg.jerk);
        targetSnap_ = Eigen::Vector3d::Zero();
    } else if (msg.type_mask == 2) {
        targetAcc_ = toEigen(msg.acceleration);
        targetJerk_ = Eigen::Vector3d::Zero();
        targetSnap_ = Eigen::Vector3d::Zero();
    } else if (msg.type_mask == 4) {
        targetAcc_ = Eigen::Vector3d::Zero();
        targetJerk_ = Eigen::Vector3d::Zero();
        targetSnap_ = Eigen::Vector3d::Zero();
    } else {
        targetAcc_ = toEigen(msg.acceleration);
        targetJerk_ = toEigen(msg.jerk);
        targetSnap_ = toEigen(msg.snap);
    }
}

/// @brief Yaw Control (need to be implemented)
/// @param msg Target yaw
void geometricCtrl::yawtargetCallback(const std_msgs::Float32& msg) {
    // ROS_INFO_STREAM("Received Yaw message");
    if (!velocity_yaw_) mavYaw_ = double(msg.data);
}

void geometricCtrl::multiDOFJointCallback(
    const trajectory_msgs::MultiDOFJointTrajectory& msg) {
    // ROS_INFO_STREAM("Received MultiDOFJointTrajectory message");
    received_target_pose = true;
    trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg.points[0];
    reference_request_last_ = reference_request_now_;

    targetPos_prev_ = targetPos_;
    targetVel_prev_ = targetVel_;

    reference_request_now_ = ros::Time::now();
    reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

    targetPos_ << pt.transforms[0].translation.x, pt.transforms[0].translation.y,
        pt.transforms[0].translation.z;
    targetVel_ << pt.velocities[0].linear.x, pt.velocities[0].linear.y,
        pt.velocities[0].linear.z;

    targetAcc_ << pt.accelerations[0].linear.x, pt.accelerations[0].linear.y,
        pt.accelerations[0].linear.z;
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();

    if (!velocity_yaw_) {
        Eigen::Quaterniond q(pt.transforms[0].rotation.w, pt.transforms[0].rotation.x,
                             pt.transforms[0].rotation.y, pt.transforms[0].rotation.z);
        Eigen::Vector3d rpy = Eigen::Matrix3d(q).eulerAngles(0, 1, 2); // RPY
        mavYaw_ = rpy(2);
    }
}

void geometricCtrl::mavposeCallback(const geometry_msgs::PoseStamped& msg) {
    if (!received_home_pose) {
        received_home_pose = true;
        home_pose_ = msg.pose;
        ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
        homePosePub_.publish(home_pose_);
    }
    mavPos_ = toEigen(msg.pose.position);
    mavAtt_(0) = msg.pose.orientation.w;
    mavAtt_(1) = msg.pose.orientation.x;
    mavAtt_(2) = msg.pose.orientation.y;
    mavAtt_(3) = msg.pose.orientation.z;
}

void geometricCtrl::mavtwistCallback(const geometry_msgs::TwistStamped& msg) {
    mavVel_ = toEigen(msg.twist.linear);
    mavRate_ = toEigen(msg.twist.angular);
}

bool geometricCtrl::takeoffCallback(geometric_controller::Takeoff::Request& request,
                                    geometric_controller::Takeoff::Response& response) {
    ros::spinOnce();
    arm_cmd_.request.value = true;
    if (!current_state_.armed) {
        if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
            ROS_INFO("Vehicle armed");
            response.success = true; // Set response fields properly
            response.message = "Vehicle armed";
        } else {
            ROS_WARN("Arming failed");
            response.success = false;
            response.message = "Arming failed";
            return true; // Return true to indicate service was processed, even if
                         // failed
        }
    } else {
        ROS_INFO("Already armed");
        response.success = true;
        response.message = "Already armed";
    }

    // Validate and set takeoff parameters
    takeoff_height_ = std::max(0.5, std::min(3.0, static_cast<double>(request.height)));

    node_state = TAKEOFF;

    return true; // Return true to indicate service was processed successfully
}

/*
bool geometricCtrl::landCallback(
    geometric_controller::Takeoff::Request &request,
    geometric_controller::Takeoff::Response &response) {
  land_height_ =
      std::max(0.05, std::min(0.3, static_cast<double>(request.height)));
  last_hold_point_ = mavPos_;
  node_state = LANDING;
  response.success = true;
  response.message = "Landing initiated";
  return true;
}
*/

bool geometricCtrl::landCallback(std_srvs::SetBool::Request& request,
                                 std_srvs::SetBool::Response& response) {
    std_srvs::SetBool::Request hold_req;
    std_srvs::SetBool::Response hold_res;
    hold_req.data = true;
    holdCallback(hold_req, hold_res);
    hold_before_land_timer_.start();
    response.success = true;
    response.message = "Entering HOLD first, will LAND after 5 seconds";
    return true;
}

bool geometricCtrl::holdCallback(std_srvs::SetBool::Request& request,
                                 std_srvs::SetBool::Response& response) {
    last_hold_point_ = mavPos_;
    node_state = HOLD;
    return true;
}

bool geometricCtrl::missionCallback(std_srvs::SetBool::Request& request,
                                    std_srvs::SetBool::Response& response) {
    last_hold_point_ = mavPos_;
    if (!received_target_pose) {
        ROS_ERROR("No target pose received yet");
        response.success = false;
        response.message = "No target pose received yet";
        return false;
    }

    else {
        std_srvs::SetBool srv;
        srv.request.data = true;
        if (!trajectory_trigger_client_.call(srv)) {
            ROS_ERROR("Failed to trigger trajectory");
            response.success = false;
            response.message = "Failed to trigger trajectory";
            return false;
        }
        node_state = MISSION_EXECUTION;
        response.success = true;
        response.message = "Trigger trajectory && Mission execution started";
    }
    return true;
}

bool geometricCtrl::gotoCallback(geometric_controller::Goto::Request& request,
                                 geometric_controller::Goto::Response& response) {
    geometry_msgs::Point start_point;
    start_point = toGeoPoint(mavPos_);

    gcopter::getTraj traj_srv;
    traj_srv.request.start_point = start_point;
    traj_srv.request.goal_point = request.target_point;

    if (!gcopter_traj_trigger_client_.call(traj_srv)) {
        ROS_ERROR("Failed to trigger trajectory");
        response.success = false;
        response.message = "Failed to trigger trajectory";
        return false;
    } else {
        node_state = GOTO;
        response.success = true;
        response.message = "Trigger trajectory && GOTO execution started";
        posehistory_vector_.clear();
        last_hold_point_ = toEigen(request.target_point);
        return true;
    }
}

void geometricCtrl::cmdloopCallback(const ros::TimerEvent& event) {
    switch (node_state) {
        case TAKEOFF: {
            double target_z = home_pose_.position.z + takeoff_height_;
            geometry_msgs::PoseStamped desired_pose;
            Vector3d desired_position(home_pose_.position.x, home_pose_.position.y,
                                      target_z);
            desired_pose = vector3d2PoseStampedMsg(desired_position, mavAtt_);
            target_pose_pub_.publish(desired_pose);

            double current_z = mavPos_(2);
            double dz = std::abs(current_z - target_z);
            if (dz < 0.05) {
                last_hold_point_ = toEigen(home_pose_.position);
                last_hold_point_(2) = target_z;
                node_state = HOLD;
                ROS_INFO("Takeoff complete, switching to HOLD");
            }
            break;
        }

        case WAITING_FOR_HOME_POSE: {
            waitForPredicate(&received_home_pose, "Waiting for home pose...");
            ROS_INFO("Got pose! Drone Ready to be armed.");
            last_hold_point_ = toEigen(home_pose_.position);
            node_state = HOLD;
            break;
        }

        case HOLD: {
            pubHoldPose(last_hold_point_);
            break;
        }

        case GOTO: {
            // here, targetPos_ is the target point for current step
            Eigen::Vector3d pos_error = mavPos_ - targetPos_;
            if (false) {
                pubHoldPose(last_hold_point_);
            } else if ((mavPos_ - last_hold_point_).norm() < 0.1) {
                // If the drone is close to the target position, hold it
                pubHoldPose(last_hold_point_);
            } else {
                geometry_msgs::PoseStamped goto_pose;
                goto_pose.header.stamp = ros::Time::now();
                goto_pose.header.frame_id = "map";
                goto_pose.pose.position = toGeoPoint(targetPos_);
                target_pose_pub_.publish(goto_pose);
                appendPoseHistory();
                appendTrackingErrorHistory();
                pubPoseHistory();
                pubTrackingErrorHistory();
            }
            break;
        }

        case MISSION_EXECUTION: {
            Eigen::Vector3d pos_error = mavPos_ - targetPos_;
            if (pos_error.norm() > 1.1) {
                ROS_WARN("Target position too far, holding at last position");
                pubHoldPose(last_hold_point_);
            } else {
                Eigen::Vector3d desired_acc;
                if (feedthrough_enable_) {
                    desired_acc = targetAcc_;
                } else {
                    desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_);
                }
                computeBodyRateCmd(cmdBodyRate_, desired_acc);
                pubReferencePose(targetPos_, q_des);
                pubRateCommands(cmdBodyRate_, q_des);
                appendPoseHistory();
                appendTrackingErrorHistory();
                pubPoseHistory();
                pubTrackingErrorHistory();
            }
            break;
        }

        case LANDING: {
            mavros_msgs::CommandTOL land_cmd;
            if (land_client.call(land_cmd) && land_cmd.response.success) {
                ROS_INFO("Landing command accepted by FCU");
                cmdloop_timer_.stop();
                statusloop_timer_.stop();
            } else {
                ROS_WARN("Landing command failed or was rejected");
            }
            break;
        }
            /*
            case LANDING: {
              double target_z = land_height_;
              last_hold_point_.z = target_z;

              geometry_msgs::PoseStamped land_pose;
              land_pose.header.stamp = ros::Time::now();
              land_pose.header.frame_id = "map";
              land_pose.pose.position = last_hold_point_;
              target_pose_pub_.publish(land_pose);

              double current_z = mavPos_(2);
              double dz = std::abs(current_z - target_z);
              if (dz < 0.05 && current_state_.) {
                ROS_INFO("Landing complete, disarming");
                arm_cmd_.request.value = false;
                if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
                  ROS_INFO("Vehicle disarmed");
                  cmdloop_timer_.stop();
                } else {
                  ROS_WARN("Disarming failed");
                }
              }
              break;
            }
              */
    }
}

void geometricCtrl::mavstateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}

void geometricCtrl::statusloopCallback(const ros::TimerEvent& event) {
    /*
    if (sim_enable_) {
      // Enable OFFBoard mode and arm automatically
      // This will only run if the vehicle is simulated
      mavros_msgs::SetMode offb_set_mode;
      arm_cmd_.request.value = true;
      offb_set_mode.request.custom_mode = "OFFBOARD";
      if (current_state_.mode != "OFFBOARD" &&
          (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
        if (set_mode_client_.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent) {
          ROS_INFO("Offboard enabled");
        }
        last_request_ = ros::Time::now();
      } else {
        if (!current_state_.armed &&
            (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
          if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
            ROS_INFO("Vehicle armed");
          }
          last_request_ = ros::Time::now();
        }
      }
    }
    */
    if (current_state_.mode != "OFFBOARD") {
        ROS_WARN("Not in OFFBOARD mode");
        return;
    }
    pubSystemStatus();
}

void geometricCtrl::pubReferencePose(const Eigen::Vector3d& target_position,
                                     const Eigen::Vector4d& target_attitude) {
    geometry_msgs::PoseStamped msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.position.x = target_position(0);
    msg.pose.position.y = target_position(1);
    msg.pose.position.z = target_position(2);
    msg.pose.orientation.w = target_attitude(0);
    msg.pose.orientation.x = target_attitude(1);
    msg.pose.orientation.y = target_attitude(2);
    msg.pose.orientation.z = target_attitude(3);
    referencePosePub_.publish(msg);
}

/// @brief Publish the body rate commands
/// @param cmd Body rate commands: {wx, wy, wz, Thrust(normalized)}
/// @param target_attitude Target attitude: {w, x, y, z}
void geometricCtrl::pubRateCommands(const Eigen::Vector4d& cmd,
                                    const Eigen::Vector4d& target_attitude) {
    mavros_msgs::AttitudeTarget msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.body_rate.x = cmd(0);
    msg.body_rate.y = cmd(1);
    msg.body_rate.z = cmd(2);
    // DO NOT USE RATE CONTROL
    // msg.type_mask = 128;  // Ignore orientation messages
    // USE ATTITUDE CONTROL
    msg.type_mask = msg.IGNORE_PITCH_RATE + msg.IGNORE_ROLL_RATE + msg.IGNORE_YAW_RATE;
    msg.orientation.w = target_attitude(0);
    msg.orientation.x = target_attitude(1);
    msg.orientation.y = target_attitude(2);
    msg.orientation.z = target_attitude(3);
    msg.thrust = cmd(3);

    angularVelPub_.publish(msg);
}

void geometricCtrl::pubPoseHistory() {
    nav_msgs::Path msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.poses = posehistory_vector_;

    posehistoryPub_.publish(msg);
}

void geometricCtrl::pubTrackingErrorHistory() {
    std_msgs::Float32 msg;
    msg.data = tracking_error_history_.front();
    tracking_error_history_pub_.publish(msg);
}

void geometricCtrl::pubSystemStatus() {
    mavros_msgs::CompanionProcessStatus msg;

    msg.header.stamp = ros::Time::now();
    msg.component = 196; // MAV_COMPONENT_ID_AVOIDANCE
    msg.state = (int)companion_state_;

    systemstatusPub_.publish(msg);
}

/// @brief Append the current pose to the history vector
/// @note The history vector is used to visualize the path of the drone
/// @note The history vector is limited to a certain size
/// @note The history vector is published as a nav_msgs::Path message
void geometricCtrl::appendPoseHistory() {
    posehistory_vector_.insert(posehistory_vector_.begin(),
                               vector3d2PoseStampedMsg(mavPos_, mavAtt_));
    if (posehistory_vector_.size() > posehistory_window_) {
        posehistory_vector_.pop_back();
    }
}

void geometricCtrl::appendTrackingErrorHistory() {
    tracking_error_history_.insert(tracking_error_history_.begin(),
                                   (mavPos_ - targetPos_).norm());
    if (tracking_error_history_.size() > tracking_error_history_window_) {
        tracking_error_history_.pop_back();
    }
}

geometry_msgs::PoseStamped
geometricCtrl::vector3d2PoseStampedMsg(Eigen::Vector3d& position,
                                       Eigen::Vector4d& orientation) {
    geometry_msgs::PoseStamped encode_msg;
    encode_msg.header.stamp = ros::Time::now();
    encode_msg.header.frame_id = "map";
    encode_msg.pose.orientation.w = orientation(0);
    encode_msg.pose.orientation.x = orientation(1);
    encode_msg.pose.orientation.y = orientation(2);
    encode_msg.pose.orientation.z = orientation(3);
    encode_msg.pose.position.x = position(0);
    encode_msg.pose.position.y = position(1);
    encode_msg.pose.position.z = position(2);
    return encode_msg;
}

Eigen::Vector3d geometricCtrl::controlPosition(const Eigen::Vector3d& target_pos,
                                               const Eigen::Vector3d& target_vel,
                                               const Eigen::Vector3d& target_acc) {
    /// Compute BodyRate commands using differential flatness
    /// Controller based on Faessler 2017
    const Eigen::Vector3d a_ref = target_acc;
    if (velocity_yaw_) {
        mavYaw_ = getVelocityYaw(mavVel_);
    }

    const Eigen::Vector4d q_ref = acc2quaternion(a_ref - gravity_, mavYaw_);
    const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

    const Eigen::Vector3d pos_error = mavPos_ - target_pos;
    const Eigen::Vector3d vel_error = mavVel_ - target_vel;

    // Position Controller
    const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);

    // Rotor Drag compensation
    const Eigen::Vector3d a_rd =
        R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel; // Rotor drag

    // Reference acceleration
    const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - gravity_;

    return a_des;
}

void geometricCtrl::computeBodyRateCmd(Eigen::Vector4d& bodyrate_cmd,
                                       const Eigen::Vector3d& a_des) {
    // Reference attitude
    q_des = acc2quaternion(a_des, mavYaw_);

    controller_->Update(mavAtt_, q_des, a_des,
                        targetJerk_); // Calculate BodyRate
    bodyrate_cmd.head(3) = controller_->getDesiredRate();
    double thrust_command = controller_->getDesiredThrust().z();
    bodyrate_cmd(3) = std::max(
        0.0,
        std::min(1.0,
                 norm_thrust_const_ * thrust_command +
                     norm_thrust_offset_)); // Calculate
                                            // thrustcontroller_->getDesiredThrust()(3);
}

Eigen::Vector3d geometricCtrl::poscontroller(const Eigen::Vector3d& pos_error,
                                             const Eigen::Vector3d& vel_error) {
    Eigen::Vector3d a_fb =
        Kpos_.asDiagonal() * pos_error +
        Kvel_.asDiagonal() * vel_error; // feedforward term for trajectory error

    if (a_fb.norm() > max_fb_acc_)
        a_fb = (max_fb_acc_ / a_fb.norm()) *
               a_fb; // Clip acceleration if reference is too large

    return a_fb;
}

Eigen::Vector4d geometricCtrl::acc2quaternion(const Eigen::Vector3d& vector_acc,
                                              const double& yaw) {
    Eigen::Vector4d quat;
    Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
    Eigen::Matrix3d rotmat;

    proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

    zb_des = vector_acc / vector_acc.norm();
    yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
    xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

    rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2),
        yb_des(2), zb_des(2);
    quat = rot2Quaternion(rotmat);
    return quat;
}

bool geometricCtrl::ctrltriggerCallback(std_srvs::SetBool::Request& req,
                                        std_srvs::SetBool::Response& res) {
    unsigned char mode = req.data;

    ctrl_mode_ = mode;
    res.success = ctrl_mode_;
    res.message = "controller triggered";
    return true;
}

void geometricCtrl::dynamicReconfigureCallback(
    geometric_controller::GeometricControllerConfig& config, uint32_t level) {
    if (max_fb_acc_ != config.max_acc) {
        max_fb_acc_ = config.max_acc;
        ROS_INFO("Reconfigure request : max_acc = %.2f ", config.max_acc);
    } else if (Kpos_x_ != config.Kp_x) {
        Kpos_x_ = config.Kp_x;
        ROS_INFO("Reconfigure request : Kp_x  = %.2f  ", config.Kp_x);
    } else if (Kpos_y_ != config.Kp_y) {
        Kpos_y_ = config.Kp_y;
        ROS_INFO("Reconfigure request : Kp_y  = %.2f  ", config.Kp_y);
    } else if (Kpos_z_ != config.Kp_z) {
        Kpos_z_ = config.Kp_z;
        ROS_INFO("Reconfigure request : Kp_z  = %.2f  ", config.Kp_z);
    } else if (Kvel_x_ != config.Kv_x) {
        Kvel_x_ = config.Kv_x;
        ROS_INFO("Reconfigure request : Kv_x  = %.2f  ", config.Kv_x);
    } else if (Kvel_y_ != config.Kv_y) {
        Kvel_y_ = config.Kv_y;
        ROS_INFO("Reconfigure request : Kv_y =%.2f  ", config.Kv_y);
    } else if (Kvel_z_ != config.Kv_z) {
        Kvel_z_ = config.Kv_z;
        ROS_INFO("Reconfigure request : Kv_z  = %.2f  ", config.Kv_z);
    }

    Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
    Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
}
