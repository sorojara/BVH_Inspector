 /**
    @file joint.cpp
    @author(s) Rosolino Todaro
    @date 2016-09-10
    @version 0.01
 */

#include "../include/joint.hpp"
#include <armadillo>
#include <math.h>
#include <iomanip>


#define PI 3.14159265

using namespace arma;

ThreeDVectorData::ThreeDVectorData() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
}

ThreeDVectorData::~ThreeDVectorData() {
}

void ThreeDVectorData::setData(double one, double two, double three) {
    x = one;
    y = two;
    z = three;
}

void ThreeDVectorData::printData() {
    cout << "<X, Y, Z> = " << x << ", " << y << ", " << z << " " << endl;
}

double ThreeDVectorData::getX(){
    return x;
}

double ThreeDVectorData::getY(){
    return y;
}

double ThreeDVectorData::getZ(){
    return z;
}

 Joint::Joint(string nameOfBone, bool rootJoint, bool endJoint, string padre) {

    boneName = nameOfBone;
    parent = padre;
    offset[0] = 0.0;
    offset[1] = 0.0;
    offset[2] = 0.0;
    numberOfChannels = 3;
    rotationalChannelOrder[0] = 1;
    rotationalChannelOrder[1] = 2;
    rotationalChannelOrder[2] = 3;
    isRoot = rootJoint;
    isEnd = endJoint;
    framesStored = 0;

}

Joint::~Joint(){
}

void Joint::addPositionalFrame(double x, double y, double z){

    if (boneName.compare("Hips") != 0) {
        cout << "You are trying to add positional data to something other than Hips!" << endl;
        cout << "This is currently not supported in the format!" << endl;
        return;
    }

    // add frame of positional data
    position.push_back(ThreeDVectorData());
    ++framesStored;
    position[framesStored-1].setData(x, y, z);
}

void Joint::addRotationalFrame(double one, double two, double three){
    // take in data with specified order in rotationalChannelOrder and reorder it

    // we know numberOfChannels changes with HIP this code needs to be fixed
    // for now this fix goes here 6 channels = HIP (3 pos, 3 rot), rest 3 rot

    int maxChannels = numberOfChannels;
    if (boneName.compare("Hips")==0)
        maxChannels -= 3;
    double x(0), y(0), z(0);
    for (int i = 0; i < maxChannels; i++) {
        if (rotationalChannelOrder[i] == 1) { // X
            if (i == 0)
                x = one;
            else if (i == 1)
                x = two;
            else if (i == 2)
                x = three;
        } else if (rotationalChannelOrder[i] == 2) { // Y
            if (i == 0)
                y = one;
            else if (i == 1)
                y = two;
            else if (i == 2)
                y = three;
        } else if (rotationalChannelOrder[i] == 3) { // Z
            if (i == 0)
                z = one;
            else if (i == 1)
                z = two;
            else if (i == 2)
                z = three;
        } else {
            cout << "Error adding frame!!" << endl;
        }
    }

    // add frame of rotational data
    rotation.push_back(ThreeDVectorData());
    ++framesStored;
    rotation[framesStored-1].setData(x, y, z);
}


void Joint::printJointData(){
    cout << "Bone Name: " << boneName << endl;
    cout << "Parent Joint: " << parent << endl;
    cout << "Offset: " << offset[0] << ", " << offset[1] << ", ";
    cout << offset[2] << endl;
    cout << "Channels: " << numberOfChannels << endl;
    cout << "Rotational Order: " << rotationalChannelOrder[0];
    cout << ", " << rotationalChannelOrder[1] << ", ";
    cout << rotationalChannelOrder[2] << endl;
    cout << "Root Joint: " << isRoot << endl;
    cout << "End Joint: " << isEnd << endl;
    cout << "Frames: " << framesStored << endl;
}

void Joint::printPositionalData(){
    if (position.empty()) {
        cout << "No positional data has been defined!" << endl;
    } else {
        for (int i=0 ; i < position.size() ; i++) {
            cout << "------ Positional Data - Frame #" << i+1 << " ------" << endl;
            position[i].printData();
            cout << "----------------------------------------" << endl;
        }
    }

}

void Joint::printRotationalData(){
    if (rotation.empty()) {
        cout << "No rotational data has been defined!" << endl;
    } else {
        for (int i=0 ; i < rotation.size() ; i++) {
            cout << "------ Rotational Data - Frame #" << i+1 << " ------" << endl;
            rotation[i].printData();
            cout << "----------------------------------------" << endl;
        }
    }
}

mat Joint::getLocalMatrix(int frame){
    if(frame < rotation.size()){
        //define rotational matrix
        mat rz(3,3);
        mat ry(3,3);
        mat rx(3,3);

        //define sin and cos of rotational data
        double cosz = cos(PI*rotation[frame].getZ()/180);
        double sinz = sin(PI*rotation[frame].getZ()/180);
        double cosy = cos(PI*rotation[frame].getY()/180);
        double siny = sin(PI*rotation[frame].getY()/180);
        double cosx = cos(PI*rotation[frame].getX()/180);
        double sinx = sin(PI*rotation[frame].getX()/180);

        //fill z rotational data
        rz(0,0)= cosz;
        rz(0,1)= -sinz;
        rz(0,2)= 0.0;
        rz(1,0)= sinz;
        rz(1,1)= cosz;
        rz(1,2)= 0.0;
        rz(2,0)= 0.0;
        rz(2,1)= 0.0;
        rz(2,2)= 1.0;

        //fill y rotational data
        ry(0,0)= cosy;
        ry(0,1)= 0.0;
        ry(0,2)= siny;
        ry(1,0)= 0.0;
        ry(1,1)= 1.0;
        ry(1,2)= 0.0;
        ry(2,0)= -siny;
        ry(2,1)= 0.0;
        ry(2,2)= cosy;

        //fill x rotational data
        rx(0,0)= 1.0;
        rx(0,1)= 0.0;
        rx(0,2)= 0.0;
        rx(1,0)= 0.0;
        rx(1,1)= cosx;
        rx(1,2)= -sinx;
        rx(2,0)= 0.0;
        rx(2,1)= sinx;
        rx(2,2)= cosx;

        //build the rotational matrix
        mat rxyz = rz * ry;
        rxyz = rxyz * rx;

        //define the local matrix
        mat matLocal(4,4);

        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++){
                matLocal(i,j) =rxyz(i,j);
            }
        }
        matLocal(3,0)= 0.0;
        matLocal(3,1)= 0.0;
        matLocal(3,2)= 0.0;
        matLocal(3,3)= 1.0;
        matLocal(0,3)= offset[0];
        matLocal(1,3)= offset[1];
        matLocal(2,3)= offset[2];

        return matLocal;

    }else{
        cout << "The frame requested does not exist in motion data!" << endl;
        mat matError(4,4);
        return matError;
    }
}

bool Joint::getIsRoot(){
    return isRoot;
}

string Joint::getParent(){
    return parent;
}

void Joint::printLocalMatrix(int frame){
    mat loc = getLocalMatrix(frame);
    loc.print("Local Matrix: ");
}