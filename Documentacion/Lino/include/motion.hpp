 /**
    @file motion.hpp
    @author(s) Rosolino Todaro
    @date 2016-09-10
    @version 0.02
    
    @section DESCRIPTION
    
    See class descriptions. To be filled in.
 */
 
 
#ifndef MOTION_HPP
#define MOTION_HPP

#include <iostream>
#include <string>
#include <map>
#include <vector>

#include "joint.hpp"
#include <armadillo>

using namespace std;

/*! \class Motion
    \brief A class designed for storing the essential elements of a BVH file.
  
    The Motion class provides a structure to store all of the information 
    contained within a BVH file. This class works along side the Joint class to 
    provide storage for the rotational and positional data related to each 
    joint.
 */

class Motion { 
 public: 
    //constructors
     Motion();
     ~Motion(); // we need to manage the memory of some things down below
     
    //mutators
    void addJoint(string, bool, bool, string);
    void addPositionalFrameToJoint(string, double, double, double);
    void addRotationalFrameToJoint(string, double, double, double);
    void setMotionParameters(int, double);
    void setNumberOfChannelsOfJoint(string, int);
    void setOffsetOfJoint(string, double, double, double);
    void setRotationalChannelOrderOfJoint(string, int, int, int);

    //accessors
    int findJointIndex(string);
    void printData();
    void printAllJoints();
    void printJoint(string);
    void printLocalMatrix(string, int);
    arma::mat getGlobalMatrix(string, int);
    void printGlobalMatrix(string, int);
 private: 
    int numberOfJoints; // index to coincide with vector joints, and be read from map jointList
    map<string, int > jointList;
    vector<class Joint> joints; 
     
    //Motion Parameters
    int frames; // This will determine how many frames we can store, also index key for rotational data
    double frameTime;
    double duration; // T in seconds
   
 };

 #endif