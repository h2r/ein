#ifndef _EEPOSEH_
#define _EEPOSEH_

#include <iostream>
#include "eigen_util.h"
#include <cv.h>
#include <ml.h>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/gpu/gpu.hpp>
using namespace cv;
using namespace std;

typedef struct _eePose{
  double px;
  double py;
  double pz;

  double qx;
  double qy;
  double qz;
  double qw;

  _eePose plusP(const Vector3d& a) const;
  _eePose minusP(const Vector3d& a) const;

  _eePose plusP(const _eePose& a) const;
  _eePose minusP(const _eePose& a) const;

  _eePose negativeP() const;
  _eePose negativeQ() const;

  _eePose multQ(const _eePose& a) const;
  _eePose invQ() const;

  void copyP(_eePose src);
  void copyQ(_eePose src);

  _eePose applyQTo(_eePose in) const;

  void writeToFileStorage(FileStorage& fsvO) const;

  void readFromFileNodeIterator(FileNodeIterator& it);
  void readFromFileNode(FileNode& it);

  bool equals(_eePose pose);

  static void print(_eePose toPrint);
  static double squareDistance(_eePose pose1, _eePose pose2);
  static _eePose fromRectCentroid(Rect rect);

  static _eePose zero();

  friend ostream & operator<<(ostream &, const _eePose &);

} eePose;


#endif /* _EEPOSEH_ */
