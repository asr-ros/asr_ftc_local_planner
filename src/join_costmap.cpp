/**

Copyright (c) 2016, Marek Felix
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#include <ros/ros.h>

#include <ftc_local_planner/join_costmap.h>

namespace ftc_local_planner
{
    JoinCostmap::JoinCostmap()
    {
    }

    void JoinCostmap::initialize(costmap_2d::Costmap2DROS* local_costmap_ros, costmap_2d::Costmap2DROS* global_costmap_ros){
        local_costmap_ros_ = local_costmap_ros;
        global_costmap_ros_ = global_costmap_ros;

        if(local_costmap_ros_->getCostmap()->getResolution() < global_costmap_ros->getCostmap()->getResolution()){
            ROS_ERROR("JoinCostmap: Resolution of local costmap is higher than global costmap resolution!");
            init = false;
        }
        //Init vector global to right size
        global.resize(global_costmap_ros_->getCostmap()->getSizeInCellsX());
        for (unsigned int i = 0; i < global_costmap_ros_->getCostmap()->getSizeInCellsX(); ++i){
           global[i].resize(global_costmap_ros_->getCostmap()->getSizeInCellsY());
        }

        //Push global costmap in global vector
        for(unsigned int i = 0; i < global_costmap_ros_->getCostmap()->getSizeInCellsX(); i++){
            for(unsigned int j = 0; j < global_costmap_ros_->getCostmap()->getSizeInCellsY(); j++){
                global[i][j] = global_costmap_ros_->getCostmap()->getCost(i,j);
            }
        }
        ROS_INFO("JoinCostmap: Initalize.");
        init = true;
    }

    void JoinCostmap::joinMaps(){
        if(!init){
            ROS_WARN("JoinCostmap: Dont join costmap, because init faild.");
        }else{
            for(unsigned int i = 0; i < global_costmap_ros_->getCostmap()->getSizeInCellsX(); i++){
                for(unsigned int j = 0; j < global_costmap_ros_->getCostmap()->getSizeInCellsY(); j++){
                    double wx = 0;
                    double wy = 0;
                    global_costmap_ros_->getCostmap()->mapToWorld(i,j, wx, wy);

                    unsigned int mx = 0;
                    unsigned int my = 0;
                    local_costmap_ros_->getCostmap()->worldToMap(wx, wy, mx, my);

                    //Copy the highest cost from global vector or local costmap in the global costmap
                    if(global[i][j] < local_costmap_ros_->getCostmap()->getCost(mx, my)){
                        global_costmap_ros_->getCostmap()->setCost(i,j,local_costmap_ros_->getCostmap()->getCost(mx, my));
                    }else{
                        global_costmap_ros_->getCostmap()->setCost(i,j,global[i][j]);
                    }
                }
            }
        }
    }

}
