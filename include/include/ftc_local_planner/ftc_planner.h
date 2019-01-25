/**

Copyright (c) 2016, Marek Felix
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef FTC_LOCAL_PLANNER_FTC_PLANNER_H_
#define FTC_LOCAL_PLANNER_FTC_PLANNER_H_

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <tf/transform_listener.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <Eigen/Core>

#include <dynamic_reconfigure/server.h>

#include <asr_ftc_local_planner/FTCPlannerConfig.h>

#include <nav_core/base_local_planner.h>

#include <ftc_local_planner/transform_global_plan.h>

#include <ftc_local_planner/join_costmap.h>

namespace ftc_local_planner
{

    class FTCPlanner : public nav_core::BaseLocalPlanner
    {

    public:
        FTCPlanner();
        /**
         * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
         * @return True if a valid velocity command was found, false otherwise
         */
        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        /**
         * @brief  Check if the goal pose has been achieved by the local planner
         * @return True if achieved, false otherwise
         */
        bool isGoalReached();

        /**
         * @brief  Set the plan that the local planner is following
         * @param plan The plan to pass to the local planner
         * @return True if the plan was updated successfully, false otherwise
         */
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Constructs the local planner
         * @param name The name to give this instance of the local planner
         * @param tf A pointer to a transform listener
         * @param costmap_ros The cost map to use for assigning costs to local plans
         */
        void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

        ~FTCPlanner();

    private:

        /**
        *@brief Reconfigure config_
        */
        void reconfigureCB(FTCPlannerConfig &config, uint32_t level);

        /**
        *@brief Goes along global plan the max distance whith sim_time and max_x_vel allow
        *@param current pose of the robot
        *@return max point of the global plan with can reached
        */
        int checkMaxDistance(tf::Stamped<tf::Pose> current_pose);

        /**
        *@brief Goes backward along global plan the max angle whith sim_time and max_rotation_vel allow
        *@param point where starts to go backward
        *@param current pose of the robot
        *@return max point of the global plan with can reached
        */
        int checkMaxAngle(int points, tf::Stamped<tf::Pose> current_pose);

        /**
        *@brief Rotation at place
        *@param angle which is to rotate
        *@param velocity message which is calculate for rotation
        *@param accuracy of orientation
        *@return true if rotate, false if rotation goal reached
        */
        bool rotateToOrientation(double angle, geometry_msgs::Twist& cmd_vel, double accuracy);

        /**
        *@brief Publish the global plan for visulatation.
        *@param points where jused to calculate plan.
        */
        void publishPlan(int max_point);

        /**
        *@brief Drive along the global plan and calculate the velocity
        *@param current pose of the robot
        *@param velocity message
        *@return number of points of global plan which are used
        */
        int driveToward(tf::Stamped<tf::Pose> current_pose, geometry_msgs::Twist& cmd_vel);

        /**
        *@brief Calculate the orientation of the global plan
        *@param current robot pose
        *@param global plan
        *@param number of points which used for calculation
        */
        double calculateGlobalPlanAngle(tf::Stamped<tf::Pose> current_pose, const std::vector<geometry_msgs::PoseStamped>& plan, int points);

        /**
        *@brief Check if the considerd points are in local collision.
        *@param points of global plan which are considerd.
        *@return true if no collision.
        */
        bool checkCollision(int max_points);

        //used for transformation
        tf2_ros::Buffer* tf_;
        //costmap to get the current position
        costmap_2d::Costmap2DROS* costmap_ros_;
        //global plan which we run along
        std::vector<geometry_msgs::PoseStamped> global_plan_;
        //transformed global plan in global frame with only the points with are needed for calculation (max_points)
        std::vector<geometry_msgs::PoseStamped> transformed_global_plan_;
        //check if plan first at first time
        bool first_setPlan_;
        //last point of the global plan in global frame
        tf::Stamped<tf::Pose> goal_pose_;
        // true if the robot should rotate to gobal plan if new global goal set
        tf::Stamped<tf::Pose> old_goal_pose_;
        // true if the robot should rotate to gobal plan if new global goal set
        bool rotate_to_global_plan_;
        //for dynamic reconfigure
        dynamic_reconfigure::Server<FTCPlannerConfig> *dsrv_;
        //start config
        ftc_local_planner::FTCPlannerConfig default_config_;
        //reconfigure config
        ftc_local_planner::FTCPlannerConfig config_;
        //true if the goal point is reache and orientation of goal is reached
        bool goal_reached_;
        //true if the goal point is reache and orientation of goal isn't reached
        bool stand_at_goal_;
        //opublisher where the local plan for visulatation is published
        ros::Publisher local_plan_publisher_;

        //rotation velocity of previous round for the rotateToOrientation methode
        double cmd_vel_angular_z_rotate_;
        //x velocity of the previous round
        double cmd_vel_linear_x_;
        //rotation velocity of previous round for the dirveToward methode
        double cmd_vel_angular_z_;

        JoinCostmap *joinCostmap_;

    };
};
#endif
