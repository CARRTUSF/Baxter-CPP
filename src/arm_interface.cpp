/*
 * File:   arm_interface.cpp
 * Author: Andoni Aguirrezabal
 */

#include <baxter_cpp_control/arm_interface.h>

namespace baxter_cpp_control {
    BaxterArmInterface::BaxterArmInterface(const std::string &limb)
            : baxterLimbName(limb), isInitialized(false) {
        // Create a list of the joints in this kinematics chain
        baxterJointNames.push_back(limb + "_e0");
        baxterJointNames.push_back(limb + "_e1");
        baxterJointNames.push_back(limb + "_s0");
        baxterJointNames.push_back(limb + "_s1");
        baxterJointNames.push_back(limb + "_w0");
        baxterJointNames.push_back(limb + "_w1");
        baxterJointNames.push_back(limb + "_w2");
    }
    
    BaxterArmInterface::~BaxterArmInterface() {
        // Close the Joint Publisher
        jointCommandPublisher.shutdown();
    }
    
    bool BaxterArmInterface::init() {
        // Load Joint Command Publisher
        jointCommandPublisher = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/" + baxterLimbName + "/joint_command", 10);
        
        isInitialized = true; // TODO: Actually add error handling  and return
        return isInitialized;
    }
    
    void BaxterArmInterface::setJointPosition(const jointPositionList& desiredJointPositions) {
        baxterJointMessage.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
        baxterJointMessage.names.clear();
        baxterJointMessage.command.clear();

        for(jointPositionList::const_iterator i = desiredJointPositions.begin();
                i != desiredJointPositions.end(); i++) {
            std::cout << "Adding " << i->first << ": " << i->second << "\n";
            baxterJointMessage.names.push_back(i->first);
            baxterJointMessage.command.push_back(i->second);
        }
        jointCommandPublisher.publish(baxterJointMessage);
    }
    
    bool BaxterArmInterface::setJointPosition(const std::vector<double>& desiredJointPositions) {
        int j = 0; // Iterator for desiredJointPositions
        
        if(desiredJointPositions.size() == BAXTER_LIMB_JOINT_COUNT) {
            baxterJointMessage.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
            baxterJointMessage.names.clear();
            baxterJointMessage.command.clear();

            for(std::vector<std::string>::const_iterator i = baxterJointNames.begin();
                    i != baxterJointNames.end(); i++) {
                std::cout << "Adding " << *i << ": " << desiredJointPositions[j] << "\n";
                baxterJointMessage.names.push_back(*i);
                baxterJointMessage.command.push_back(desiredJointPositions[j]);
                j++;
            }

            jointCommandPublisher.publish(baxterJointMessage);
            return true;
        } else {
            // Wrong amount of joint values given
            return false;
        }
    }
} // namespace

