/**
 *  MIT License
 *
 *  Copyright (c) 2019 Rohan Singh
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

/** 
 *  @file    Walker.hpp
 *  @author  Rohan Singh
 *  @copyright MIT License
 *  @date    11/17/2019
 *  @version 1.0
 *  @brief  Class declaration for Walker class
 */

#ifndef INCLUDE_TURTLEBOT_WALKER_WALKER_HPP_
#define INCLUDE_TURTLEBOT_WALKER_WALKER_HPP_

#include <iostream>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief Class for implementing walker algorithms on turtlebot
 */
class Walker {
 public:
 /**
  * @brief  Constructor for Walker class
  */
  Walker();

 /**
  * @brief  Destructor for Walker class
  */
  ~Walker();

 /**
  * @brief  Callback function for subscriber to laserScan data
  *
  * @param  pointer to LaserScan mesage 
  *
  * @return void
  */
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

 /**
  * @brief  Function to check for upcoming collisions
  *
  * @return true if obstacle nearby, false otherwise
  */
  bool detectCollision();

 /**
  * @brief Function to implement walker algorithm and
  *        move the turtlebot around
  *
  * @return void
  */
  void walk();

 /**
  * @brief Function to return laser scan subscriber object
  *
  * @return Subscriber object
  */
  ros::Subscriber getLaserScanSub();

 /**
  * @brief Function to return command velocity publisher object
  *
  * @return Publisher object
  */
  ros::Publisher getCmdVelPub();

 /**
  * @brief Function to return node handle object
  *
  * @return Node handle object
  */
  ros::NodeHandle getNodeHandle();

 /**
  * @brief Function to return current laser scanner data
  *
  * @return Laser scan data
  */
  sensor_msgs::LaserScan getLaserScanData();

 /**
  * @brief Function to set collision detection threshold
  *
  * @param ct Collision Threshold
  *
  * @return void
  */
  void setCollisionThreshold(float ct);

 /**
  * @brief Function to return collision detection threshold
  *
  * @return Collision detection threshold
  */
  float getCollisionThreshold();



 private:
  /* ROS Node handle object */
  ros::NodeHandle n;

  /* ROS subscriber object for laser scanner data */
  ros::Subscriber laserScanSub;
  /* Message type to store laser scan data */
  sensor_msgs::LaserScan laserScanData;

  /* ROS publisher object for command velocity */
  ros::Publisher cmdVelPub;
  /* Message type to store command velocities */
  geometry_msgs::Twist cmdVel;

  /* Threshold for collision detection */
  float collisionThreshold = 0.3;
};

#endif    // INCLUDE_TURTLEBOT_WALKER_WALKER_HPP_
