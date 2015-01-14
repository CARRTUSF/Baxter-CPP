/* 
 * File:   baxter_interface.cpp
 * Author: Andoni Aguirrezabal
 */

#include <baxter_cpp_control/baxter_interface.h>

namespace baxter_cpp_control {

BaxterInterface::BaxterInterface() 
        : isInitialized(false), firstStateReceived(false) {
}

BaxterInterface::~BaxterInterface() {
    // Close the State Subscriber
    jointStateSubscriber.shutdown();
}

bool BaxterInterface::init() {
    // Load Joint State Subscriber
    jointStateSubscriber = nh.subscribe<sensor_msgs::JointState>("/robot/joint_states", 1,
                           &BaxterInterface::jointStateCallback, this);
    
    isInitialized = true; // TODO: Actually add error handling  and return
    return isInitialized;
}

void BaxterInterface::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg) {
    // Check to make sure the joint states match Baxter
    if(msg->name.size() != BAXTER_TOTAL_JOINT_COUNT) {
        return;
    } else {
        lastJointState = *msg;
    }
}

sensor_msgs::JointStateConstPtr BaxterInterface::getJointStates(void) {
    if(firstStateReceived) {
        return sensor_msgs::JointStateConstPtr(&lastJointState);
    } else {
        return sensor_msgs::JointStateConstPtr();
    }
}

} // namespace

