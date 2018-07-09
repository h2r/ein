#ifndef _EEPOSEH_
#define _EEPOSEH_


#include <geometry_msgs/Pose.h>
#include <cv.h>

#include <iostream>
#include <fstream>
using namespace std;

using namespace cv;




typedef struct _eePose{
  double px;
  double py;
  double pz;

  double qx;
  double qy;
  double qz;
  double qw;


  _eePose plusP(const _eePose& a) const;
  _eePose minusP(const _eePose& a) const;

  _eePose multP(const double& a) const;

  _eePose negativeP() const;
  _eePose negativeQ() const;

  _eePose multQ(const _eePose& a) const;
  _eePose invQ() const;

  void copyP(_eePose src);
  void copyQ(_eePose src);

  _eePose applyQTo(_eePose in) const;
  _eePose getPoseRelativeTo(_eePose in) const;
  _eePose applyAsRelativePoseTo(_eePose in) const;
  _eePose getInterpolation(_eePose inB, double lambda) const; 
  _eePose getNormalized() const; 

  _eePose applyRPYTo(double roll, double pitch, double yaw) const;

  void writeToFileStorage(FileStorage& fsvO) const;

  void readFromFileNodeIterator(FileNodeIterator& it);
  void readFromFileNode(FileNode& it);

  bool equals(_eePose pose);

  static void print(_eePose toPrint);
  static double squareDistance(_eePose pose1, _eePose pose2);
  static double distance(_eePose pose1, _eePose pose2);
  static double distanceQ(_eePose pose1, _eePose pose2);
  static double dotP(_eePose pose1, _eePose pose2);

  static void distanceXYZAndAngle(_eePose pose1, _eePose pose2, double * distance, double * angleDistance);

  void getRollPitchYaw(double * roll, double * pitch, double * yaw);
  double qmagnitude();

  static _eePose fromRectCentroid(Rect rect);
  static _eePose zero();
  static _eePose identity();
  static _eePose fromGeometryMsgPose(geometry_msgs::Pose);
  

  friend ostream & operator<<(ostream &, const _eePose &);

  _eePose(double _px, double _py, double _pz, double _qx, double _qy, double _qz, double _qw);
  _eePose();

  bool operator==(const _eePose& other);
  bool operator!=(const _eePose& other);
} eePose;

typedef struct _armPose{

  double joints[7];

  bool equals(_armPose pose);

  friend ostream & operator<<(ostream &, const _armPose &);

  _armPose(double a0, double a1, double a2, double a3, double a4, double a5, double a6);
  _armPose();

  void writeToFileStorage(FileStorage& fsvO) const;

  /*

  void readFromFileNodeIterator(FileNodeIterator& it);
  void readFromFileNode(FileNode& it);

  static void print(_armPose toPrint);
  static double distance(_armPose pose1, _armPose pose2);


  _armPose plusP(const Vector3d& a) const;
  _armPose minusP(const Vector3d& a) const;

  _armPose plusP(const _armPose& a) const;
  _armPose minusP(const _armPose& a) const;

  _armPose multP(const double& a) const;

  _armPose negativeP() const;
  _armPose negativeQ() const;

  _armPose multQ(const _armPose& a) const;
  _armPose invQ() const;

  void copyP(_armPose src);
  void copyQ(_armPose src);

  _armPose applyQTo(_armPose in) const;
  _armPose getPoseRelativeTo(_armPose in) const;
  _armPose applyAsRelativePoseTo(_armPose in) const;
  _armPose getInterpolation(_armPose inB, double lambda) const; 

  _armPose applyRPYTo(double roll, double pitch, double yaw) const;

  static double squareDistance(_armPose pose1, _armPose pose2);
  static double distanceQ(_armPose pose1, _armPose pose2);

  static void distanceXYZAndAngle(_armPose pose1, _armPose pose2, double * distance, double * angleDistance);

  void getRollPitchYaw(double * roll, double * pitch, double * yaw);


  static _armPose fromRectCentroid(Rect rect);
  static _armPose zero();
  static _armPose identity();
  static _armPose fromGeometryMsgPose(geometry_msgs::Pose);
  */
} armPose;

eePose rosPoseToEEPose(geometry_msgs::Pose pose);
geometry_msgs::Pose eePoseToRosPose(eePose);

#endif /* _EEPOSEH_ */
