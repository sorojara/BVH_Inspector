 /**
    @file platform.cpp
    @author(s) Rosolino Todaro
    @date 2016-09-10
    @version 0.01 
 */
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <vector>

#include "../include/joint.hpp"
#include "../include/motion.hpp"

using namespace std;

int main() {

    Motion m;
    
    m.addJoint("Hips", true, false, "");
    m.setNumberOfChannelsOfJoint("Hips",6);
    m.setRotationalChannelOrderOfJoint("Hips",2,1,3);
    m.addPositionalFrameToJoint("Hips",7.000,0.0000,0.22222);
    
    //for (int i = 0 ; i < 1000 ; i++) {
      //  m.addRotationalFrameToJoint("Hips", 0.1, 1.1, 2.2);
    //}

    m.addJoint("LeftLeg", true, false, "Hips");
    m.addPositionalFrameToJoint("LeftLeg",7.000,0.0000,0.22222);
    m.addJoint("LeftLeg2", true, false, "Hips"); 
    m.setOffsetOfJoint("LeftLeg", -0.2244242, -0.323232, 1.32322);
    m.addRotationalFrameToJoint("LeftLeg", 0.1, 1.1, 2.2);
    m.addRotationalFrameToJoint("LeftLeg", 4.0, 2.0, 0.0);
    
    m.setMotionParameters(7027,0.008333);
    m.printAllJoints();

    m.printData();

    m.printLocalMatrix("LeftLeg", 1);

    return 0;
}
