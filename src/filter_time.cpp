
#include <dirent.h>

#include "ros/ros.h"
#include "ros/package.h"
#include "ros/time.h"

#include "std_msgs/String.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "object_recognition_msgs/RecognizedObjectArray.h"
#include "object_recognition_msgs/RecognizedObject.h"

#include <sstream>
#include <iostream>
#include <math.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//#include <cv.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/nonfree/nonfree.hpp>
//#include <opencv2/opencv.hpp>

#include "../../bing/Objectness/stdafx.h"
#include "../../bing/Objectness/Objectness.h"
#include "../../bing/Objectness/ValStructVec.h"
#include "../../bing/Objectness/CmShow.h"

bool isFiniteNumber(double x) 
{
    return (x <= DBL_MAX && x >= -DBL_MAX); 
} 

double imageDecay = 0.9;
double pointCloudDecay = 0.9;

cv::Mat cam_img;
cv::Mat filtered_img;
bool real_img = false;
bool imInitialized = false;
bool pcInitialized = false;
pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
pcl::PointCloud<pcl::PointXYZRGB> filteredPointCloud;

ros::Publisher filtered_pc_publisher;
ros::Publisher filtered_im_publisher;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
cout << "hit imageCallback" << endl;

  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cam_img = cv_ptr->image;
    real_img = true;
  }catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (!imInitialized) {
    filtered_img = cam_img.clone();
    imInitialized = 1;
  }

  Size sz = cv_ptr->image.size();
  int imW = sz.width;
  int imH = sz.height;

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      filtered_img.at<cv::Vec3b>(y,x) = (imageDecay * filtered_img.at<cv::Vec3b>(y,x)) + 
					((1.0-imageDecay) * cam_img.at<cv::Vec3b>(y,x));
    }
  }

  cv::imshow("Current Image Frame", cv_ptr->image);
  cv::imshow("Filtered Image", filtered_img);

  cv_bridge::CvImagePtr out_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  out_ptr->image = filtered_img;
  sensor_msgs::Image im_to_send = *(out_ptr->toImageMsg()); 
  im_to_send.header = msg->header;
  filtered_im_publisher.publish(im_to_send);

  cv::waitKey(1);
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
cout << "hit pointCloudCallback" << endl;
  pcl::fromROSMsg(*msg, pointCloud);

  if (!pcInitialized) {
    pcl::fromROSMsg(*msg, filteredPointCloud);
    pcInitialized = 1;
  }

  int imW = filteredPointCloud.width;
  int imH = filteredPointCloud.height;


  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      pcl::PointXYZRGB p0 = filteredPointCloud.at(x,y);
      pcl::PointXYZRGB p1 = pointCloud.at(x,y);
      if (!isFiniteNumber(p0.x) ||
	  !isFiniteNumber(p0.y) ||
	  !isFiniteNumber(p0.z) ) {
	filteredPointCloud.at(x,y) = pointCloud.at(x,y);
      } else if (isFiniteNumber(p1.x) &&
	  isFiniteNumber(p1.y) &&
	  isFiniteNumber(p1.z) ) {

	filteredPointCloud.at(x,y).x = pointCloudDecay * filteredPointCloud.at(x,y).x +
					(1.0-pointCloudDecay) * pointCloud.at(x,y).x;
	filteredPointCloud.at(x,y).y = pointCloudDecay * filteredPointCloud.at(x,y).y +
					(1.0-pointCloudDecay) * pointCloud.at(x,y).y;
	filteredPointCloud.at(x,y).z = pointCloudDecay * filteredPointCloud.at(x,y).z +
					(1.0-pointCloudDecay) * pointCloud.at(x,y).z;
	filteredPointCloud.at(x,y).r = pointCloudDecay * filteredPointCloud.at(x,y).r +
					(1.0-pointCloudDecay) * pointCloud.at(x,y).r;
	filteredPointCloud.at(x,y).g = pointCloudDecay * filteredPointCloud.at(x,y).g +
					(1.0-pointCloudDecay) * pointCloud.at(x,y).g;
	filteredPointCloud.at(x,y).b = pointCloudDecay * filteredPointCloud.at(x,y).b +
					(1.0-pointCloudDecay) * pointCloud.at(x,y).b;
      } else {
/*
	filteredPointCloud.at(x,y).x = pointCloudDecay * filteredPointCloud.at(x,y).x;
	filteredPointCloud.at(x,y).y = pointCloudDecay * filteredPointCloud.at(x,y).y;
	filteredPointCloud.at(x,y).z = pointCloudDecay * filteredPointCloud.at(x,y).z;
	filteredPointCloud.at(x,y).r = pointCloudDecay * filteredPointCloud.at(x,y).r;
	filteredPointCloud.at(x,y).g = pointCloudDecay * filteredPointCloud.at(x,y).g;
	filteredPointCloud.at(x,y).b = pointCloudDecay * filteredPointCloud.at(x,y).b;
*/
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB> fPCtmp;// = filteredPointCloud;
  pcl::copyPointCloud(filteredPointCloud, fPCtmp);


  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      pcl::PointXYZRGB p0 = filteredPointCloud.at(x,y);
      pcl::PointXYZRGB p1 = pointCloud.at(x,y);
      if (isFiniteNumber(p0.x) &&
	  isFiniteNumber(p0.y) &&
	  isFiniteNumber(p0.z) && (!isFiniteNumber(p1.x) ||
	  !isFiniteNumber(p1.y) ||
	  !isFiniteNumber(p1.z) ) ) {

	pcl::PointXYZRGB pa = fPCtmp.at(min(x+1, imW-1),y);
	pcl::PointXYZRGB pb = fPCtmp.at(x,min(y+1, imH-1));
	pcl::PointXYZRGB pc = fPCtmp.at(max(x-1,0),y);
	pcl::PointXYZRGB pd = fPCtmp.at(x,max(y-1,0));

	int numValid = 0;
	pcl::PointXYZRGB pf;
	pf.x = 0;
	pf.y = 0;
	pf.z = 0;
	pf.r = 0;
	pf.g = 0;
	pf.b = 0;
	/*int numValid = 1;
	pcl::PointXYZRGB pf = filteredPointCloud.at(x,y);*/

	if (isFiniteNumber(pa.x) &&
	    isFiniteNumber(pa.y) &&
	    isFiniteNumber(pa.z) ) {
	  pf.x = pf.x + pa.x;
	  pf.y = pf.y + pa.y;
	  pf.z = pf.z + pa.z;
	  pf.r = pf.r + pa.r/4.0;
	  pf.g = pf.g + pa.g/4.0;
	  pf.b = pf.b + pa.b/4.0;
	  numValid++;
	}
	if (isFiniteNumber(pb.x) &&
	    isFiniteNumber(pb.y) &&
	    isFiniteNumber(pb.z) ) {
	  pf.x = pf.x + pb.x;
	  pf.y = pf.y + pb.y;
	  pf.z = pf.z + pb.z;
	  pf.r = pf.r + pb.r/4.0;
	  pf.g = pf.g + pb.g/4.0;
	  pf.b = pf.b + pb.b/4.0;
	  numValid++;
	}
	if (isFiniteNumber(pc.x) &&
	    isFiniteNumber(pc.y) &&
	    isFiniteNumber(pc.z) ) {
	  pf.x = pf.x + pc.x;
	  pf.y = pf.y + pc.y;
	  pf.z = pf.z + pc.z;
	  pf.r = pf.r + pc.r/4.0;
	  pf.g = pf.g + pc.g/4.0;
	  pf.b = pf.b + pc.b/4.0;
	  numValid++;
	}
	if (isFiniteNumber(pd.x) &&
	    isFiniteNumber(pd.y) &&
	    isFiniteNumber(pd.z) ) {
	  pf.x = pf.x + pd.x;
	  pf.y = pf.y + pd.y;
	  pf.z = pf.z + pd.z;
	  pf.r = pf.r + pd.r/4.0;
	  pf.g = pf.g + pd.g/4.0;
	  pf.b = pf.b + pd.b/4.0;
	  numValid++;
	}
	if (numValid > 0) {
//cout << double(pf.r)/double(numValid) << " " << int(filteredPointCloud.at(x,y).r) << " " << numValid << endl;
	  filteredPointCloud.at(x,y).x = pointCloudDecay * filteredPointCloud.at(x,y).x +
					(1.0-pointCloudDecay) * (pf.x / numValid);
	  filteredPointCloud.at(x,y).y = pointCloudDecay * filteredPointCloud.at(x,y).y +
					(1.0-pointCloudDecay) * (pf.y / numValid);
	  filteredPointCloud.at(x,y).z = pointCloudDecay * filteredPointCloud.at(x,y).z +
					(1.0-pointCloudDecay) * (pf.z / numValid);
	  filteredPointCloud.at(x,y).r = ((pointCloudDecay * double(filteredPointCloud.at(x,y).r)) +
					(1.0-pointCloudDecay) * 4*(double(pf.r) / (numValid)));
	  filteredPointCloud.at(x,y).g =((pointCloudDecay * double(filteredPointCloud.at(x,y).g)) +
					(1.0-pointCloudDecay) * 4*(double(pf.g) / (numValid)));
	  filteredPointCloud.at(x,y).b = ((pointCloudDecay * double(filteredPointCloud.at(x,y).b)) +
					(1.0-pointCloudDecay) * 4*(double(pf.b) / (numValid)));
	}
//cout << numValid << " ";

/*
	  filteredPointCloud.at(x,y).x = (fPCtmp.at(x+1,y).x + fPCtmp.at(x-1,y).x +
					  fPCtmp.at(x,y+1).x + fPCtmp.at(x,y-1).x)/4;
	  filteredPointCloud.at(x,y).y = (fPCtmp.at(x+1,y).y + fPCtmp.at(x-1,y).y +
					  fPCtmp.at(x,y+1).y + fPCtmp.at(x,y-1).y)/4;
	  filteredPointCloud.at(x,y).z = (fPCtmp.at(x+1,y).z + fPCtmp.at(x-1,y).z +
					  fPCtmp.at(x,y+1).z + fPCtmp.at(x,y-1).z)/4;
*/


      }
    }
  }


  sensor_msgs::PointCloud2 pc2_to_send;
  pcl::toROSMsg(filteredPointCloud, pc2_to_send);
  pc2_to_send.header = msg->header;
  //pc2_to_send.header.stamp = ros::Time::now();
  //pc2_to_send.header.seq = msg->header.seq;
  filtered_pc_publisher.publish(pc2_to_send);
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "filter_time");
  ros::NodeHandle n("~");

  image_transport::Subscriber image_sub;
  image_transport::ImageTransport it(n);

  image_sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);

  ros::Subscriber points = n.subscribe("/camera/depth_registered/points", 1, pointCloudCallback);


  filtered_pc_publisher = n.advertise<sensor_msgs::PointCloud2>("filtered_pointcloud", 10);
  filtered_im_publisher = n.advertise<sensor_msgs::Image>("filtered_image", 10);


  cv::namedWindow("Current Image Frame");
  cv::namedWindow("Filtered Image");

  ros::spin();
  return 0;
}
