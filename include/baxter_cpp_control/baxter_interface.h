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

// Baxter
#include <baxter_core_msgs/AssemblyState.h>

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
    //***********************
    //*** COMMAND METHODS ***
    //***********************
    
    
public:
    //***************************
    //*** INFORMATION METHODS ***
    //***************************
    // Baxter States
    bool isFirstBaxterStateReceived(void);
    baxter_core_msgs::AssemblyState getBaxterSatate(void);

    // Joint States
    bool isFirstJointStateReceived(void);
    sensor_msgs::JointState getJointStates(void);

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
    ros::Subscriber baxterStateSubscriber;
    ros::Subscriber jointStateSubscriber;
    
    // Subscriber Callbacks
    void baxterStateCallback(const baxter_core_msgs::AssemblyStateConstPtr& msg);
    void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
    
    // Baxter Information Variables
    bool firstBaxterStateReceived;
    baxter_core_msgs::AssemblyState lastBaxterState;
    
    // Joint Information Variables
    bool firstJointStateReceived;
    sensor_msgs::JointState lastJointState;
};
    
} // namespace


#endif	/* BAXTER_INTERFACE_H */

