/* 
 * File:   baxter_defines.hpp
 * Author: Andoni Aguirrezabal
 */

#ifndef BAXTER_DEFINES_HPP
#define	BAXTER_DEFINES_HPP

// C++ and STL
#include <string>
#include <map>

#define BAXTER_TOTAL_JOINT_COUNT 17
#define BAXTER_LIMB_JOINT_COUNT 7

namespace baxter_cpp_control {
    typedef std::map<std::string, double> jointPositionList; //Key Mapping
    typedef std::pair<std::string, double> jointPositionPair;
    
    enum BAXTER_LIMBS {
        ARM_LEFT,
        ARM_RIGHT
    };

} // namespace

#endif	/* BAXTER_DEFINES_HPP */

