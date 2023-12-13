/**
 * Copyright (C) 2023  template
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <pivot_base/pivot_base.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "pivot_base_node");
  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  pivot_base::PivotBase pivot_base( buffer );

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return 0;
}

/*
 *int main(int argc, char** argv){
 *  ros::init(argc, argv, "pivot_base");
 *  PivotBase pivot_base;
 *  ros::spin();
 *  return 0;
 *}
 */