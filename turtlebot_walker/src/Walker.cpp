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
 *  @file    Walker.cpp
 *  @author  Rohan Singh
 *  @copyright MIT License
 *  @date    11/17/2019
 *  @version 1.0
 *  @brief  Class definition for Walker class
 */

#include "turtlebot_walker/Walker.hpp"


Walker::Walker() {
  /* Initialize publisher object */
  cmdVelPub = n.advertise<geometry_msgs::Twist>
                ("/cmd_vel", 100);

  /* Initialize subscriber object */
  laserScanSub = n.subscribe("/m2wr/laser/scan", 100,
                   &Walker::laserScanCallback, this);

  /* Initialize Scan data */
  laserScanData.angle_min = 0;
  laserScanData.angle_max = 0;

  /* Initialize command velocites */
  cmdVel.linear.x = 0;
  cmdVel.linear.y = 0;
  cmdVel.linear.z = 0;
  cmdVel.angular.x = 0;
  cmdVel.angular.y = 0;
  cmdVel.angular.z = 0;

  /* Publish initial velocities */
  cmdVelPub.publish(cmdVel);

  ROS_INFO_STREAM("Walker object Initialized.");
}

Walker::~Walker() {
  /* Publish final command velocities to stop turtlebot
   * before destroying the object */
  cmdVel.linear.x = 0;
  cmdVel.linear.y = 0;
  cmdVel.linear.z = 0;
  cmdVel.angular.x = 0;
  cmdVel.angular.y = 0;
  cmdVel.angular.z = 0;

  cmdVelPub.publish(cmdVel);
}

void Walker::laserScanCallback(const
              sensor_msgs::LaserScan::ConstPtr& msg) {
  laserScanData = *msg;
}

bool Walker::detectCollision() {
  for (auto r : laserScanData.ranges) {
    if (r < collisionThreshold) {
      ROS_WARN_STREAM("Collision imminent.");
      return true;
    }
  }
  return false;
}

void Walker::walk() {
  /* Set publisher frequency */
  ros::Rate loopRate(100);

  if (!ros::ok()) {
    ROS_FATAL_STREAM("ROS node is not running.");
  }

  while (ros::ok()) {     // Till ros is running
    if (laserScanData.ranges.size() == 0) {
      ROS_WARN_STREAM("No laser scan data available.");
    } else {
      if (detectCollision()) {
        ROS_INFO_STREAM("Turning robot.");
        cmdVel.linear.x = 0.0;
        cmdVel.angular.z = -1.0;
      } else {
        ROS_INFO_STREAM("Going straight.");
        cmdVel.linear.x = -0.2;
        cmdVel.angular.z = 0.0;
      }
      /* Publish commanded velocity */
      cmdVelPub.publish(cmdVel);
    }
    ros::spinOnce();
    loopRate.sleep();
  }
}

ros::Subscriber Walker::getLaserScanSub() {
  return laserScanSub;
}

ros::Publisher Walker::getCmdVelPub() {
  return cmdVelPub;
}

ros::NodeHandle Walker::getNodeHandle() {
  return n;
}

sensor_msgs::LaserScan Walker::getLaserScanData() {
  return laserScanData;
}

void Walker::setCollisionThreshold(float ct) {
  collisionThreshold = ct;
}

float Walker::getCollisionThreshold() {
  return collisionThreshold;
}
