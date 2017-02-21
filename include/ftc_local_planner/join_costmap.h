/**

Copyright (c) 2016, Marek Felix
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef FTC_LOCAL_PLANNER_JOIN_COSTMAP_H_
#define FTC_LOCAL_PLANNER_JOIN_COSTMAP_H_

#include <costmap_2d/costmap_2d_ros.h>

#include <tf/transform_listener.h>

#include <Eigen/Core>

#include <vector>
using std::vector;

namespace ftc_local_planner
{
    class JoinCostmap
    {
    public:
        /**
         * @brief JoinCostmap constructor.
         */
        JoinCostmap();

        /**
         * @brief initialize the two costmaps and check if resolution global costmap > resolution local costmap. With is neseccary.
         * @param local_costmap_ros
         * @param global_costmap_ros
         */
        void initialize(costmap_2d::Costmap2DROS* local_costmap_ros, costmap_2d::Costmap2DROS* global_costmap_ros);

        /**
         * @brief joinMaps join the local costmap in the global costmap.
         */
        void joinMaps();

    private:
        costmap_2d::Costmap2DROS* global_costmap_ros_;
        costmap_2d::Costmap2DROS* local_costmap_ros_;

        //Vector with all costvalues from the original global costmap.
        vector<vector<int> > global;

        //Is initialization true.
        bool init;
    };
}
#endif
