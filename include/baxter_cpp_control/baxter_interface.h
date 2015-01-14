/* 
 * File:   baxter_interface.h
 * Author: Andoni Aguirrezabal
 */

#ifndef BAXTER_INTERFACE_H
#define	BAXTER_INTERFACE_H

// Common Baxter Defines and Datatypes
#include <baxter_cpp_control/baxter_defines.hpp>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace baxter_cpp_control {
    
class BaxterInterface {
public:
    //*********************
    //*** CLASS METHODS ***
    //*********************
    // Constructor
    BaxterInterface();
    
    //Deconstructor
    ~BaxterInterface();
    
public:
    //*****************************
    //*** CONFIGURATION METHODS ***
    //*****************************
    // Initialization
    bool init();
    
public:
    //***************************
    //*** INFORMATION METHODS ***
    //***************************
    sensor_msgs::JointStateConstPtr getJointStates(void);
    
private:
    //*********************
    //*** CONFIGURATION ***
    //*********************
    // ROS Node Handle
    ros::NodeHandle nh;
    
    // Interface Status
    bool isInitialized;
    
private:
    //********************
    //*** ROS HANDLERS ***
    //********************
    // Subscribers
    ros::Subscriber jointStateSubscriber;
    
    // Subscriber Callbacks
    void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
    
    // Information Variables
    bool firstStateReceived;
    sensor_msgs::JointState lastJointState;

};
    
} // namespace


#endif	/* BAXTER_INTERFACE_H */

