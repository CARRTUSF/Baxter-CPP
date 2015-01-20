/* 
 * File:   baxter_interface.cpp
 * Author: Andoni Aguirrezabal
 */

#include "baxter_cpp_control/baxter_interface.h"

namespace baxter_cpp_control {

BaxterInterface::BaxterInterface() 
        : isInitialized(false), firstJointStateReceived(false) {
}

BaxterInterface::~BaxterInterface() {
    // Close the State Subscriber
    jointStateSubscriber.shutdown();
}

bool BaxterInterface::init() {
    // Load Baxter Subscriber
    baxterStateSubscriber = nh.subscribe<baxter_core_msgs::AssemblyState>("/robot/state", 1,
                            &BaxterInterface::baxterStateCallback, this);

    // Load Joint State Subscriber
    jointStateSubscriber = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1,
                           &BaxterInterface::jointStateCallback, this);

    if(baxterStateSubscriber && jointStateSubscriber) {
        isInitialized = true;
        return true;    
    } else {
        isInitialized = false;
        return false;
    }
}

void BaxterInterface::baxterStateCallback(const baxter_core_msgs::AssemblyStateConstPtr& msg) {
    if(!firstBaxterStateReceived) {
        firstBaxterStateReceived = true;
    }
    lastBaxterState = *msg;
}

void BaxterInterface::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg) {
    // Check to make sure the joint states match Baxter
    if(msg->name.size() != BAXTER_TOTAL_JOINT_COUNT) {
        return;
    } else {
        if(!firstJointStateReceived) {
            firstJointStateReceived = true;
        }
        lastJointState = *msg;
    }
}

bool BaxterInterface::isFirstBaxterStateReceived(void) {
    return firstBaxterStateReceived;
}

bool BaxterInterface::isFirstJointStateReceived(void) { 
    return firstJointStateReceived; 
}

sensor_msgs::JointState BaxterInterface::getJointStates(void) {
    if(firstJointStateReceived) {
        // Return the last received joint states list
        return lastJointState;
    } else {
        // Return blank Joint State
        return sensor_msgs::JointState();
    }
}

} // namespace

