/**

Copyright (c) 2016, Marek Felix
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <ftc_local_planner/transform_global_plan.h>

namespace ftc_local_planner
{
    bool getXPose(const tf2_ros::Buffer& tf,
                  const std::vector<geometry_msgs::PoseStamped>& global_plan,
                  const std::string& global_frame, tf::Stamped<tf::Pose>& goal_pose, int plan_point)
    {
        if (global_plan.empty())
        {
            ROS_ERROR("Received plan with zero length");
            return false;
        }
        if(plan_point >= (int)global_plan.size())
        {
            ROS_ERROR("Goal_functions: Plan_point %d to big. Plan size: %lu",plan_point, global_plan.size());
            return false;
        }

        const geometry_msgs::PoseStamped& plan_goal_pose = global_plan.at(plan_point);
        try
        {
            tf.canTransform(global_frame, ros::Time::now(),
                            plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                            plan_goal_pose.header.frame_id, ros::Duration(0.5));  

            geometry_msgs::TransformStamped tmp = tf.lookupTransform(global_frame, ros::Time(),
                                                                    plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                                                                    plan_goal_pose.header.frame_id, ros::Duration(0.5));
            tf::StampedTransform transform;
            transformStampedMsgToTF(tmp, transform);

            poseStampedMsgToTF(plan_goal_pose, goal_pose);    
            goal_pose.setData(transform * goal_pose);
            goal_pose.stamp_ = transform.stamp_;
            goal_pose.frame_id_ = global_frame;
        }
        catch(tf::LookupException& ex)
        {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch(tf::ConnectivityException& ex)
        {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch(tf::ExtrapolationException& ex)
        {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            if (global_plan.size() > 0)
                ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

            return false;
        }
        return true;
    }
}
