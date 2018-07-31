 /**
    @file motion.cpp
    @author(s) Rosolino Todaro
    @date 2016-09-10
    @version 0.01 
 */
 
//#include "../include/joint.hpp" 
#include "../include/motion.hpp"
#include <armadillo>

using namespace arma;

Motion::Motion(){
    numberOfJoints = 0;

    //Motion Parameters
    frames = 0;
    frameTime = 0.0;
    duration = 0.0;
}
Motion::~Motion(){
}

void Motion::addJoint(string name, bool isRoot, bool isEnd, string parent){
    joints.push_back(Joint(name, isRoot, isEnd, parent));
    jointList.insert(pair<string, int > (name, numberOfJoints));
    numberOfJoints++;    
}

void Motion::addPositionalFrameToJoint(string name, double x, double y, double z){
    joints[findJointIndex(name)].addPositionalFrame(x, y, z);  
}

void Motion::addRotationalFrameToJoint(string name, double one, double two, double three){
    joints[findJointIndex(name)].addRotationalFrame(one, two, three);  
}

void Motion::setMotionParameters(int framesLoadedFromFile, double frameTimeFromFile) {
    frames = framesLoadedFromFile;
    frameTime = frameTimeFromFile;
    duration = frames*frameTime; // T in seconds
}
void Motion::setNumberOfChannelsOfJoint(string name, int number){
   joints[findJointIndex(name)].numberOfChannels = number;        
}
void Motion::setOffsetOfJoint(string name, double x, double y, double z){
   joints[findJointIndex(name)].offset[0] = x;      
   joints[findJointIndex(name)].offset[1] = y;    
   joints[findJointIndex(name)].offset[2] = z;    
}
void Motion::setRotationalChannelOrderOfJoint(string name, int one, int two, int three){
    // 0 = NA, X = 1, Y = 2, Z = 3
    joints[findJointIndex(name)].rotationalChannelOrder[0] = one;
    joints[findJointIndex(name)].rotationalChannelOrder[1] = two;
    joints[findJointIndex(name)].rotationalChannelOrder[2] = three;
    
}

int Motion::findJointIndex(string name){ 
    map<string, int > ::iterator it;
    it = jointList.find(name);
    if (it != jointList.end()) {
        int jointKey = it->second;
      //  cout << "Joint " << name << " located at "<< jointKey << "!" << endl;
        return jointKey;
        
    } else {
        //cout << "Joint " << name << " does not exist!" << endl;
        return -1;
    }
    return 1;
}

void Motion::printAllJoints(){
    if (numberOfJoints == 0) { 
        cout << "No joints have been defined" << endl;
        return;
    }
    for (int i=0 ; i < numberOfJoints ; i++) {
        cout << "-------------- Joint #" << i << " --------------" << endl;
        joints[i].printJointData();
        joints[i].printRotationalData();
        joints[i].printPositionalData();
        cout << "--------------------------------------" << endl;
    } 
}

void Motion::printData(){
    map<string, int > ::iterator it;
    it = jointList.begin();
    
    cout << "------------ Motion Data -------------" << endl;
    cout << "Number of Joints: " << numberOfJoints << endl;
    
    for(; it != jointList.end(); it++) {
        cout << "Joint #" << it->second;
        cout << " " << it->first << endl;

    }
    
    cout << "Frames: " << frames << endl;
    cout << "Frame Time: " << frameTime << endl;
    cout << "Duration: " << duration << endl;
    cout << "-------------------------------------" << endl;
    
    int numberOfJoints; // index to coincide with vector joints, and be read from map jointList
    map<string, int > jointList;
    vector<class Joint> joints; 
     
    //Motion Parameters
    int frames; // This will determine how many frames we can store, also index key for rotational data
    double frameTime;
    double duration;
    
}

void Motion::printJoint(string name){
    int jointIndex = findJointIndex(name);
    if (jointIndex = -1) { 
        cout << "The specified joint does not exist!" << endl;
        return;
    } else {
        cout << "-------------- Joint #" << jointIndex << " --------------" << endl;
        joints[jointIndex].printJointData();
        joints[jointIndex].printRotationalData();
        joints[jointIndex].printPositionalData();
        cout << "--------------------------------------" << endl;
    } 
}

void Motion::printLocalMatrix(string name, int frame){
    int i = findJointIndex(name);
    joints[i].printLocalMatrix(frame);
}

//recursive method to return the global matrix
mat Motion::getGlobalMatrix(string name, int frame){
    int i = findJointIndex(name);
    if(joints[i].getIsRoot()){
        return joints[i].getLocalMatrix(frame);
    } else {
        return joints[i].getLocalMatrix(frame) * getGlobalMatrix(joints[i].getParent(), frame);
    }
}

void Motion::printGlobalMatrix(string name, int frame){
    mat glob = getGlobalMatrix(name, frame);
    glob.print("Global Matrix: ");
}