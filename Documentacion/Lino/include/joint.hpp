 /**
    @file joint.hpp
    @author(s) Rosolino Todaro
    @date 2016-09-10
    @version 0.02

    @section DESCRIPTION

    See class descriptions. To be filled in.
 */

#ifndef JOINT_HPP
#define JOINT_HPP

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <armadillo>

using namespace std;

/*! \class ThreeDVectorData
    \brief A class designed for storing three dimensional vectors.

    The TheeDVectorData class provides a mechanism to group a three
    dimensional vector such as x, y, and z (or i,j, and k) from the cartesian
    coordinate system.
 */

class ThreeDVectorData {
 public:
    //constructors
     ThreeDVectorData();
     ~ThreeDVectorData();

    //mutators
    void setData(double, double, double);

    //accessors
    void printData();
    double getX();
    double getY();
    double getZ();
 private:
  //  bool reordered;
    double x;
    double y;
    double z;


 };

/*! \class Joint
    \brief A class designed for storing Joints of a BVH file.

    The Joint class provides a data structure to store all of the data
    associated with a joint located in a BVH file. Data from the HIERARCHY and
    MOTION section related to a joint is contained within a Joint.
 */
class Joint {
 public:
    //constructors
     Joint(string, bool, bool, string);
     ~Joint(); // we need to manage the memory of some things down below

    friend class Motion;
    //mutators
    void addPositionalFrame(double, double, double);
    // receive the three parameters from file in order specified by

    void addRotationalFrame(double, double, double);


    //accessors
    void printJointData();
    void printPositionalData();
    void printRotationalData();
    arma::mat getLocalMatrix(int);
    void printLocalMatrix(int);
    bool getIsRoot();
    string getParent();
 private:
    string boneName;
    string parent;
    double offset[3]; // OFFSET Positional Data always X, Y, Z
    int numberOfChannels;
    // N/A = 0, X = 1, Y = 2, Z = 3 to determine sequence of rotational data
    // EXPAND THIS to 6 call it Channels with vector
    int rotationalChannelOrder[3];

    bool isRoot;
    bool isEnd; // This would be the End Site flag

    int framesStored;
    // Rotational X, Y, Z data in MOTION series, indexed by frameStored-1
    vector<class ThreeDVectorData> rotation;

    // Positional X, Y, Z data in MOTION series, indexed by framesStored-1 (The
    // root is the only one with this..)
    vector<class ThreeDVectorData> position;

    // Positional X, Y, Z data in MOTION series, indexed by frame-1 used by

//   vector<class ThreeDVectorData> localMatrixSeries;
    // not required by root?, calculated for every frame??!
 //   vector<class ThreeDVectorData> globalMatrixSeries;

 };


 #endif
