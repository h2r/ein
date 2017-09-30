#include "eePose.h"
#include <iostream>

using namespace std;

ostream & operator<<(ostream & os, const _eePose& toPrint)
{
  FileStorage st;
  st.open("tmp.yml", FileStorage::WRITE | FileStorage::MEMORY);
  st << "eePose"; 
  toPrint.writeToFileStorage(st);
  string result = st.releaseAndGetString();
  os << result.substr(10, result.size());
  return os;
} 

_eePose _eePose::zero() {
  _eePose zeroOut = eePose(0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0);
  return zeroOut;
}

_eePose _eePose::identity() {
  _eePose idOut = eePose(0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 1.0);
  return idOut;
}


_eePose _eePose::fromGeometryMsgPose(geometry_msgs::Pose pose) {
  _eePose out;
  out.px = pose.position.x;
  out.py = pose.position.y;
  out.pz = pose.position.z;

  out.qx = pose.orientation.x;
  out.qy = pose.orientation.y;
  out.qz = pose.orientation.z;
  out.qw = pose.orientation.w;
  return out;
}

void _eePose::print(eePose toPrint) {
  cout << toPrint << endl;
}

void _eePose::distanceXYZAndAngle(_eePose pose1, _eePose pose2, double * distance, double * angleDistance)
{
  double dx = (pose1.px - pose2.px);
  double dy = (pose1.py - pose2.py);
  double dz = (pose1.pz - pose2.pz);
  *distance = dx*dx + dy*dy + dz*dz;
  
  double qx = (fabs(pose1.qx) - fabs(pose2.qx));
  double qy = (fabs(pose1.qy) - fabs(pose2.qy));
  double qz = (fabs(pose1.qz) - fabs(pose2.qz));
  double qw = (fabs(pose1.qw) - fabs(pose2.qw));
  *angleDistance = qx*qx + qy*qy + qz*qz + qw*qw;

}

double _eePose::squareDistance(eePose pose1, eePose pose2) {
  double dx = (pose1.px - pose2.px);
  double dy = (pose1.py - pose2.py);
  double dz = (pose1.pz - pose2.pz);
  double squareDistance = dx*dx + dy*dy + dz*dz;

  return squareDistance;
}

double _eePose::distance(eePose pose1, eePose pose2) {
  double squareDistance = eePose::squareDistance(pose1, pose2);
  double distance = pow(squareDistance, 0.5);

  return distance;
}

double _eePose::distanceQ(eePose pose1, eePose pose2) {
  double thisDot = fabs(pose1.qw*pose2.qw + pose1.qx*pose2.qx + pose1.qy*pose2.qy + pose1.qz*pose2.qz);
  thisDot = max( 0.0, min( thisDot, 1.0) );
  double distance = acos(thisDot);
  return distance;
}

double _eePose::dotP(eePose pose1, eePose pose2) {
  double dot = pose1.px * pose2.px + pose1.py * pose2.py + pose1.pz * pose2.pz;
  return dot;
}

eePose _eePose::fromRectCentroid(Rect rect) {
  eePose result;
  result.px = rect.x + rect.width * 0.5;
  result.py = rect.y + rect.width * 0.5;
  return result;
}

_eePose _eePose::plusP(const Vector3d& a) const {
  _eePose toReturn;
  toReturn.px = px + a.x();
  toReturn.py = py + a.y();
  toReturn.pz = pz + a.z();
  toReturn.qx = qx;
  toReturn.qy = qy;
  toReturn.qz = qz;
  toReturn.qw = qw;
  return toReturn;
}

_eePose _eePose::minusP(const Vector3d& a) const {
  _eePose toReturn;
  toReturn.px = px - a.x();
  toReturn.py = py - a.y();
  toReturn.pz = pz - a.z();
  toReturn.qx = qx;
  toReturn.qy = qy;
  toReturn.qz = qz;
  toReturn.qw = qw;
  return toReturn;
}

_eePose _eePose::plusP(const _eePose& a) const {
  _eePose toReturn;
  toReturn.px = px + a.px;
  toReturn.py = py + a.py;
  toReturn.pz = pz + a.pz;
  toReturn.qx = qx;
  toReturn.qy = qy;
  toReturn.qz = qz;
  toReturn.qw = qw;
  return toReturn;
}

_eePose _eePose::minusP(const _eePose& a) const {
  _eePose toReturn;
  toReturn.px = px - a.px;
  toReturn.py = py - a.py;
  toReturn.pz = pz - a.pz;
  toReturn.qx = qx;
  toReturn.qy = qy;
  toReturn.qz = qz;
  toReturn.qw = qw;
  return toReturn;
}

_eePose _eePose::multP(const double& a) const {
  _eePose toReturn;
  toReturn.px = px * a;
  toReturn.py = py * a;
  toReturn.pz = pz * a;
  toReturn.qx = qx;
  toReturn.qy = qy;
  toReturn.qz = qz;
  toReturn.qw = qw;
  return toReturn;
}

_eePose _eePose::negativeP() const {
  _eePose toReturn;
  toReturn.px = -px;
  toReturn.py = -py;
  toReturn.pz = -pz;
  toReturn.qx = qx;
  toReturn.qy = qy;
  toReturn.qz = qz;
  toReturn.qw = qw;
  return toReturn;
}

_eePose _eePose::negativeQ() const {
  _eePose toReturn;
  toReturn.px = px;
  toReturn.py = py;
  toReturn.pz = pz;
  toReturn.qx = -qx;
  toReturn.qy = -qy;
  toReturn.qz = -qz;
  toReturn.qw = -qw;
  return toReturn;
}

_eePose _eePose::multQ(const _eePose& a) const {
  Quaternionf thisQ(qw, qx, qy, qz); 
  Quaternionf aQ(a.qw, a.qx, a.qy, a.qz); 
  Quaternionf oQ = thisQ * aQ; 
  
  _eePose toReturn;
  toReturn.px = px;
  toReturn.py = py;
  toReturn.pz = pz;
  toReturn.qx = oQ.x();
  toReturn.qy = oQ.y();
  toReturn.qz = oQ.z();
  toReturn.qw = oQ.w();
  return toReturn;
}

_eePose _eePose::invQ() const {
  Quaternionf thisQ(qw, qx, qy, qz); 
  Quaternionf oQ = thisQ.inverse(); 
  
  _eePose toReturn;
  toReturn.px = px;
  toReturn.py = py;
  toReturn.pz = pz;
  toReturn.qx = oQ.x();
  toReturn.qy = oQ.y();
  toReturn.qz = oQ.z();
  toReturn.qw = oQ.w();
  return toReturn;
}

void _eePose::copyP(_eePose src) {
  px = src.px;
  py = src.py;
  pz = src.pz;
}

void _eePose::copyQ(_eePose src) {
  qx = src.qx;
  qy = src.qy;
  qz = src.qz;
  qw = src.qw;
}

_eePose _eePose::applyQTo(_eePose in) const {
  // Eigen says that when txing more than one point it is more efficient to first convert to Matrix3 then apply
  Quaterniond thisQ(qw, qx, qy, qz); 
  Vector3d inP(in.px, in.py, in.pz);
  Vector3d result = thisQ._transformVector(inP);

  _eePose outPose = eePose::zero();
  outPose.px = result.x();
  outPose.py = result.y();
  outPose.pz = result.z();
  return outPose;
}

_eePose _eePose::getPoseRelativeTo(_eePose in) const {
  _eePose thisAbsolute3dGrasp = (*this);
  _eePose txQ = in.invQ();
  txQ = txQ.multQ(thisAbsolute3dGrasp);

  _eePose thisAbsoluteDeltaP = thisAbsolute3dGrasp.minusP(in);
  _eePose thisRelative3dGrasp = in.invQ().applyQTo(thisAbsoluteDeltaP);
  thisRelative3dGrasp.copyQ(txQ);
  return thisRelative3dGrasp;
}


_eePose _eePose::applyAsRelativePoseTo(_eePose in) const {
  _eePose toApply = (*this);  
  _eePose thisAbsoluteGrasp = in;

  // this order is important because quaternion multiplication is not commutative
  thisAbsoluteGrasp = thisAbsoluteGrasp.plusP(thisAbsoluteGrasp.applyQTo(toApply));
  thisAbsoluteGrasp = thisAbsoluteGrasp.multQ(toApply);

  return thisAbsoluteGrasp;
}

_eePose _eePose::getInterpolation(_eePose inB, double mu) const {
  mu = max(mu, 0.0);
  mu = min(mu, 1.0);
  double lambda = 1.0 - mu;

  _eePose inA = (*this); 

  Quaternionf q1(inA.qw, inA.qx, inA.qy, inA.qz);
  Quaternionf q2(inB.qw, inB.qx, inB.qy, inB.qz);
  Quaternionf tTerp = q1.slerp(mu, q2);

  _eePose out;
  
  out.qw = tTerp.w();
  out.qx = tTerp.x();
  out.qy = tTerp.y();
  out.qz = tTerp.z();

  out.px = (lambda*inA.px) + (mu*inB.px);
  out.py = (lambda*inA.py) + (mu*inB.py);
  out.pz = (lambda*inA.pz) + (mu*inB.pz);

  return out;
}

_eePose _eePose::getNormalized() const {
  double norm = dotP(*this,*this);
  if (norm > 0) {
    _eePose thisNormalized = this->multP(1.0 / norm);
    return thisNormalized;
  } else {
    return (*this);
  }
}

_eePose _eePose::applyRPYTo(double roll_z, double pitch_y, double yaw_x) const {

  Eigen::Vector3f localUnitX;
  {
    Eigen::Quaternionf qin(0, 1, 0, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(qw, qx, qy, qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitX.x() = qout.x();
    localUnitX.y() = qout.y();
    localUnitX.z() = qout.z();
  }

  Eigen::Vector3f localUnitY;
  {
    Eigen::Quaternionf qin(0, 0, 1, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(qw, qx, qy, qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitY.x() = qout.x();
    localUnitY.y() = qout.y();
    localUnitY.z() = qout.z();
  }

  Eigen::Vector3f localUnitZ;
  {
    Eigen::Quaternionf qin(0, 0, 0, 1);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(qw, qx, qy, qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitZ.x() = qout.x();
    localUnitZ.y() = qout.y();
    localUnitZ.z() = qout.z();
  }

  double sinBuff = 0.0;
  double angleRate = 1.0;
  Eigen::Quaternionf eeBaseQuat(qw, qx, qy, qz);
  sinBuff = sin(angleRate*yaw_x/2.0);
  Eigen::Quaternionf eeRotatorX(cos(angleRate*yaw_x/2.0), localUnitX.x()*sinBuff, localUnitX.y()*sinBuff, localUnitX.z()*sinBuff);
  sinBuff = sin(angleRate*pitch_y/2.0);
  Eigen::Quaternionf eeRotatorY(cos(angleRate*pitch_y/2.0), localUnitY.x()*sinBuff, localUnitY.y()*sinBuff, localUnitY.z()*sinBuff);
  sinBuff = sin(angleRate*roll_z/2.0);
  Eigen::Quaternionf eeRotatorZ(cos(angleRate*roll_z/2.0), localUnitZ.x()*sinBuff, localUnitZ.y()*sinBuff, localUnitZ.z()*sinBuff);

  eeRotatorX.normalize();
  eeRotatorY.normalize();
  eeRotatorZ.normalize();

  eeBaseQuat = eeRotatorX * eeRotatorY * eeRotatorZ * eeBaseQuat;
  eeBaseQuat.normalize();

  _eePose toreturn;
  toreturn.px = px;
  toreturn.py = py;
  toreturn.pz = pz;
  toreturn.qw = eeBaseQuat.w();
  toreturn.qx = eeBaseQuat.x();
  toreturn.qy = eeBaseQuat.y();
  toreturn.qz = eeBaseQuat.z();

  return toreturn;
}

void _eePose::writeToFileStorage(FileStorage& fsvO) const {
  fsvO << "{:";
  fsvO << "px" << px;
  fsvO << "py" << py;
  fsvO << "pz" << pz;
  fsvO << "qw" << qw;
  fsvO << "qx" << qx;
  fsvO << "qy" << qy;
  fsvO << "qz" << qz;
  fsvO << "}";
}

void _eePose::readFromFileNodeIterator(FileNodeIterator& it) {
  FileNode node = *it;
  readFromFileNode(node);
}

void _eePose::readFromFileNode(FileNode& it) {
  px = (double)(it)["px"];
  py = (double)(it)["py"];
  pz = (double)(it)["pz"];
  qw = (double)(it)["qw"];
  qx = (double)(it)["qx"];
  qy = (double)(it)["qy"];
  qz = (double)(it)["qz"];
}



bool _eePose::equals(_eePose pose)
{
  if (pose.px == this->px &&
      pose.py == this->py &&
      pose.pz == this->pz &&
      pose.qx == this->qx &&
      pose.qy == this->qy &&
      pose.qz == this->qz && 
      pose.qw == this->qw) {
    return true;
  } else {
    return false;
  }
}

_eePose::_eePose(double _px, double _py, double _pz, double _qx, double _qy, double _qz, double _qw) {
  px = _px;
  py = _py;
  pz = _pz;
  qx = _qx;
  qy = _qy;
  qz = _qz;
  qw = _qw;
}

_eePose::_eePose() {
  px = 0;
  py = 0;
  pz = 0;
  qx = 0;
  qy = 0;
  qz = 0;
  qw = 0;
}


void _eePose::getRollPitchYaw(double * roll, double * pitch, double * yaw) {
  Eigen::Quaternionf quat(qw, qx, qy, qz);

  Eigen::Matrix3f matrix = quat.toRotationMatrix();
  Eigen::Vector3f angles = matrix.eulerAngles(0,1,2);
  *roll = ((double) angles(0));
  *pitch = ((double) angles(1));
  *yaw = ((double) angles(2));

  
}

_armPose::_armPose(double a0, double a1, double a2, double a3, double a4, double a5, double a6) {
  joints[0] = a0;
  joints[1] = a1;
  joints[2] = a2;
  joints[3] = a3;
  joints[4] = a4;
  joints[5] = a5;
  joints[6] = a6;
}

_armPose::_armPose() {
  joints[0] = 0;
  joints[1] = 0;
  joints[2] = 0;
  joints[3] = 0;
  joints[4] = 0;
  joints[5] = 0;
  joints[6] = 0;
}

bool _armPose::equals(_armPose pose)
{
  if (pose.joints[0] == this->joints[0] &&
      pose.joints[1] == this->joints[1] &&
      pose.joints[2] == this->joints[2] &&
      pose.joints[3] == this->joints[3] &&
      pose.joints[4] == this->joints[4] &&
      pose.joints[5] == this->joints[5] && 
      pose.joints[6] == this->joints[6]) {
    return true;
  } else {
    return false;
  }
}

ostream & operator<<(ostream & os, const _armPose& toPrint)
{
  FileStorage st;
  st.open("tmp.yml", FileStorage::WRITE | FileStorage::MEMORY);
  st << "armPose"; 
  toPrint.writeToFileStorage(st);
  string result = st.releaseAndGetString();
  os << result.substr(10, result.size());
  return os;
} 

void _armPose::writeToFileStorage(FileStorage& fsvO) const {
  fsvO << "{:";
  fsvO << "a0" << joints[0];
  fsvO << "a1" << joints[1];
  fsvO << "a2" << joints[2];
  fsvO << "a3" << joints[3];
  fsvO << "a4" << joints[4];
  fsvO << "a5" << joints[5];
  fsvO << "a6" << joints[6];
  fsvO << "}";
}





eePose rosPoseToEEPose(geometry_msgs::Pose pose) {
  eePose result;
  result.px = pose.position.x;
  result.py = pose.position.y;
  result.pz = pose.position.z;
  result.qx = pose.orientation.x;
  result.qy = pose.orientation.y;
  result.qz = pose.orientation.z;
  result.qw = pose.orientation.w;
  return result;
}

