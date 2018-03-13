#ifndef PSES_TRAJECTORY_HPP
#define PSES_TRAJECTORY_HPP

#include <std_msgs/String.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Int16.h>
#include <stdlib.h>
#include <signal.h>
#include <dynamic_reconfigure/server.h>
#include <pses_control/controllerConfig.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <ctime>

class PsesTrajectory {

public:
    PsesTrajectory();

    // Variables
    ros::NodeHandle nh;	// use this for global access

    // Functions
    /*
     * NAME:        publishGoal
     * DESCRIPTION: Publishes new goal when robot is close to current goal
     * INPUT:
     * OUTPUT:      void
    */
    void publishGoal();

    /*
     * NAME:        driveTrajectory
     * DESCRIPTION: Computes velocity and steering control variables
     * INPUT:
     * OUTPUT:      void
    */
    void driveTrajectory();

    /*
     * NAME:        clearCostmap (not in use)
     * DESCRIPTION: Clears costmap when robot gets stuck
     * INPUT:
     * OUTPUT:      void
    */
    void clearCostmap();

    /*
     * NAME:        reset
     * DESCRIPTION: Set steering and velocity to zero
     * INPUT:
     * OUTPUT:      void
    */
    void reset();

private:

    // Variables [note: command = Stellgroesse, value = physikalische Groesse]

    std_msgs::Int16 m_velocity, m_steering;                 //velocity and steering command
    double m_ack_vel, m_ack_steering;                       //velocity and steering values
    geometry_msgs::PoseWithCovarianceStamped m_amcl_pose;   //AMCL pose
    int m_goal_counter, m_problem_counter;                  //respective counters for currently activated goal and detection if robot is stuck
    double m_goal_pos_x, m_goal_pos_y;                      //x and y position of current goal
    double m_old_ack_velocity;                              //velocity value of previous iteration
    ros::Time m_begin_change;                               //first time when robot is possibly stuck
    bool m_problem_detected;                                //true when robot is possibly stuck

    // Subscriber
    ros::Subscriber m_sub_ackermann_cmd;                    //subscriber for velocity value topic
    ros::Subscriber m_sub_amcl_pose;                        //subscriber for amcl pose topic

    // Publisher
    ros::Publisher m_pub_velocity;                          //publisher for velocity command
    ros::Publisher m_pub_steering;                          //publisher for steering command
    ros::Publisher m_pub_goal;                              //publisher for goal

    // Functions

    /*
     * NAME:        cmdCallback
     * DESCRIPTION: Stores steering and velocity data from TEB Local Planner in workspace variables
     * INPUT:       const geometry_msgs::Twist::ConstPtr& data - command velocity (including x,y,z data for steering and velocity)
     *              double* m_ack_steering - steering value (workspace)
     *              double* m_ack_vel - velocity value (workspace)
     * OUTPUT:      void
    */
    static void cmdCallback(const geometry_msgs::Twist::ConstPtr& data, double* m_ack_steering, double* m_ack_vel);

    /*
     * NAME:        amclPoseCallback
     * DESCRIPTION: Stores pose from AMCL package in workspace variables
     * INPUT:       const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose - AMCL pose
     *              geometry_msgs::PoseWithCovarianceStamped *m_amcl_pose - AMCL pose (workspace)
     * OUTPUT:      void
    */
    static void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose, geometry_msgs::PoseWithCovarianceStamped *m_amcl_pose);
};

/*
 * NAME:        signalHandler
 * DESCRIPTION: Sets stop request to true when program is aborted
 * INPUT:       int sig - aborting signal
 * OUTPUT:      void
*/
void signalHandler(int sig);
bool stop_request = false;

#endif // PSES_TRAJECTORY_HPP
