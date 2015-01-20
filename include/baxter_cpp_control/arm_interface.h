/*
 * File:   arm_interface.h
 * Author: Andoni Aguirrezabal
 */

#ifndef ARM_INTERFACE_H
#define ARM_INTERFACE_H

// Common Baxter Defines and Datatypes
#include <baxter_cpp_control/baxter_defines.hpp>

// ROS
#include <ros/ros.h>

// Baxter
#include <baxter_core_msgs/JointCommand.h>

namespace baxter_cpp_control {

class BaxterArmInterface {
public:
    //*********************
    //*** CLASS METHODS ***
    //*********************
    // Constructor
    BaxterArmInterface(const std::string &limb);
    
    //Deconstructor
    ~BaxterArmInterface();

public:
    //*****************************
    //*** CONFIGURATION METHODS ***
    //*****************************
    // Initialization
    bool init();

public:
    //************************
    //*** POSITION CONTROL ***
    //************************
    // Set Joint Position
    void moveToJointPosition(const jointPositionList& desiredJointPositions,
                             bool isBlocking);
    void setJointPosition(const jointPositionList& desiredJointPositions);
    bool setJointPosition(const std::vector<double>& desiredJointPositions);
    
private:
    //*********************
    //*** CONFIGURATION ***
    //*********************
    // ROS Node Handle
    ros::NodeHandle nh;
    
    // Interface Status
    bool isInitialized;
    
    // Limb and Joint Names (adjusted for the limb selected)
    std::string baxterLimbName;
    std::vector<std::string> baxterJointNames;
    
private:
    //********************
    //*** ROS HANDLERS ***
    //********************
    // Publishers
    ros::Publisher jointCommandPublisher;
  
    // Outgoing Message
    baxter_core_msgs::JointCommand baxterJointMessage;

};

} //namespace

#endif	/* ARM_INTERFACE_H */

