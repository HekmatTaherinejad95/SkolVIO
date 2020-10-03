#include <iostream>
#include<fstream>
#include <ros/ros.h>
#include <cmath>
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "../include/ros_mono_vo/vo_features.h"
#include "../include/ros_mono_vo/Drawtrajectory.h"

//Algorithm Outline:
//Capture images: ItIt, It+1It+1,
//Undistort the above images.
//Use FAST algorithm to detect features in ItIt, and track those features to It+1It+1. A new detection is triggered if the number of features drop below a certain threshold.
//Use Nister’s 5-point alogirthm with RANSAC to compute the essential matrix.
//Estimate R,tR,t from the essential matrix that was computed in the previous step.
//Take scale information from some external source (like a speedometer), and concatenate the translation vectors, and rotation matrices.


using namespace std;
using namespace Eigen;

inline float SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

inline float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}


#define MIN_NUM_FEAT 50 

cv::Mat init_1_c, init_2_c; //These are the two color images we will use// .
cv::Mat init_1, init_2; //These are the BW images that will be converted from the color images.
cv::Mat prevImage, currImage;
cv::Mat prevImage_c, currImage_c;
cv::Mat E, R, t, mask, R_f, t_f; //variables for the function.
//cv::Mat prevPts, currPts;
std::vector<cv::Point2f> points1, points2, prevFeatures, currFeatures; //Vectors to store the coordinates of feature points.
std::vector<uchar> status;
double focal_length;
double scale = 0.2;

bool init= false;

void imageCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info)
{


  try {

      cv::imshow("view", cv_bridge::toCvShare(image, "bgr8")->image);
      // Init logic to ensure two images are captured for initial features
      if (!init) {

          init_2_c = cv_bridge::toCvShare(image, "bgr8")->image;

          if (!init_1_c.empty() && !init_2_c.empty()) {
              std::cout << "Initializing..." << std::endl;

              //Convert to grayscale.
              cv::cvtColor(init_1_c, init_1, COLOR_BGR2GRAY);
              cv::cvtColor(init_2_c, init_2, COLOR_BGR2GRAY);

              //feature detection and tracking.
              featureDetection(init_1, points1); //detect features in Img1
              featureTracking(init_1, init_2, points1, points2, status); //track those features to init_2

              //focal_length = cam_info->K[0];
              focal_length = 1200.0;
              std::cout << "Focal Length: " << focal_length << std::endl;
              cv::Point2d ppoint(cam_info->K[2], cam_info->K[5]); //principle point
              std::cout << "Finding essential matrix..." << std::endl;
              E = cv::findEssentialMat(points1, points2, focal_length, ppoint, RANSAC, 0.999, 1.0, mask);
              std::cout << "E = " << std::endl << " " << E << std::endl << std::endl;
              cv::recoverPose(E, points1, points2, R, t, focal_length, ppoint, mask);


              prevImage = init_2;
              prevFeatures = points1;

              std::cout << "R = " << std::endl << " " << R << std::endl << std::endl;
              std::cout << "t = " << std::endl << " " << t << std::endl << std::endl;

              R_f = R.clone();
              t_f = t.clone();

              init = true;
          }

          init_1_c = init_2_c;
      }

      if (init) //Main portion of callback right now.
      {
          std::cout << "Visual Odom Publishing..." << std::endl;
          std::cout << "In main loop" << std::endl;
          currImage_c = cv_bridge::toCvShare(image, "bgr8")->image;
          cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
          std::cout << "In after colorchange" << std::endl;

          featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
          std::cout << "feat track" << std::endl;

          cv::Point2d ppoint(cam_info->K[2], cam_info->K[5]); //principle point
          std::cout << "principle point" << std::endl;

          E = cv::findEssentialMat(prevFeatures, currFeatures, focal_length, ppoint, RANSAC, 0.999, 1.0, mask);
          cv::recoverPose(E, prevFeatures, currFeatures, R, t, focal_length, ppoint, mask);

          cv::Mat prevPts(2, prevFeatures.size(), CV_64F);
          cv::Mat currPts(2, currFeatures.size(), CV_64F);

          for (int i = 0; i <
                          prevFeatures.size(); i++) {   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
              prevPts.at<double>(0, i) = prevFeatures.at(i).x;
              prevPts.at<double>(1, i) = prevFeatures.at(i).y;

              currPts.at<double>(0, i) = currFeatures.at(i).x;
              currPts.at<double>(1, i) = currFeatures.at(i).y;
          }

          //scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));

          if ((scale > 0.1) && (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
              //construct trajectory here:
              ifstream fin;
              ofstream fout;
              fin.open ("trajectory.txt", ios::out | ios::in);
              t_f = t_f + scale * (R_f * t); //translation
              R_f = R * R_f; //rotation
              //file << t_f;
              //file << R_f;
              std::cout << "R_f = " << R_f << std::endl;
              std::cout << "R_f(2,2) = " << R_f.at<double>(1, 1) << std::endl;
              std::cout << "t_f = " << t_f << std::endl;
              float r11 = R_f.at<double>(0, 0);
              float r12 = R_f.at<double>(0, 1);
              float r13 = R_f.at<double>(0, 2);
              float r21 = R_f.at<double>(1, 0);
              float r22 = R_f.at<double>(1, 1);
              float r23 = R_f.at<double>(1, 2);
              float r31 = R_f.at<double>(2, 0);
              float r32 = R_f.at<double>(2, 1);
              float r33 = R_f.at<double>(2, 2);
              float t1 = t_f.at<double>(0, 0);
              float t2 = t_f.at<double>(0, 1);
              float t3 = t_f.at<double>(0, 2);
              /*float q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
              float q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
              float q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
              float q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
              if (q0 < 0.0f) {
                  q0 = 0.0f;
              }
              if (q1 < 0.0f) {
                  q1 = 0.0f;
              }
              if (q2 < 0.0f) {
                  q2 = 0.0f;
              }
              if (q3 < 0.0f) {
                  q3 = 0.0f;
              }
              q0 = sqrt(q0);
              q1 = sqrt(q1);
              q2 = sqrt(q2);
              q3 = sqrt(q3);
              if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
                  q0 *= +1.0f;
                  q1 *= SIGN(r32 - r23);
                  q2 *= SIGN(r13 - r31);
                  q3 *= SIGN(r21 - r12);
              } else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
                  q0 *= SIGN(r32 - r23);
                  q1 *= +1.0f;
                  q2 *= SIGN(r21 + r12);
                  q3 *= SIGN(r13 + r31);
              } else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
                  q0 *= SIGN(r13 - r31);
                  q1 *= SIGN(r21 + r12);
                  q2 *= +1.0f;
                  q3 *= SIGN(r32 + r23);
              } else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
                  q0 *= SIGN(r21 - r12);
                  q1 *= SIGN(r31 + r13);
                  q2 *= SIGN(r32 + r23);
                  q3 *= +1.0f;
              } else {
                  printf("coding error\n");
              }
              float r = NORM(q0, q1, q2, q3);
              q0 /= r;
              q1 /= r;
              q2 /= r;
              q3 /= r;

              Mat res = (Mat_<float>(7, 1) << t1, t2, t3, q0, q1, q2, q3);
              Mat transposedres = res.t();
              cout << "Quaternion = " << transposedres << endl;
              fout.open("trajectory.txt", ios::app | ios::out);
              fout << t1 <<" "<< t2 << " " << t3 << " "<< q0 << " " << q1 << " " << q2 <<" " << q3 << endl;*/
	      Eigen::Matrix3d m; m<< r11,r12,r13,r21,r22,r23,r31,r32,r33;

              std::cout<<"Input matrix:"<<std::endl<<m<<std::endl;
              std::cout<<"Convert to quaternion q:"<<std::endl;
              Eigen::Quaterniond q(m);
              Print_Quaternion(q);
              std::cout<<"Convert back to rotation matrix m1="<<std::endl;
              Eigen::Matrix3d m1=q.normalized().toRotationMatrix();
              std::cout<<m1<<std::endl;
              std::cout<<"Convert again to quaternion q1="<<std::endl;
              Eigen::Quaterniond q1(m1);
              Print_Quaternion(q1);
              //Output_data(q, T)
              fout.open("trajectory.txt", ios::app | ios::out);
              fout << t1 <<" "<< t2 << " " << t3 << " "<< q1.w() << " " << q1.x() << " " << q1.y() <<" " << q1.z() << endl;

          }


          // try and find more keypoints if enough were found.
          if (prevFeatures.size() < MIN_NUM_FEAT) {
              std::cout << "Number of tracked features reduced to " << prevFeatures.size() << std::endl;
              std::cout << "trigerring redection" << std::endl;
              featureDetection(prevImage, prevFeatures);
              featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

          }

          prevImage = currImage.clone();
          prevFeatures = currFeatures;

          // sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));

          // std::cout << text << std::endl;
      }
      cv::waitKey(30);



  }

catch (cv_bridge::Exception& e)
{
  ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
}


}

int main(int argc, char **argv)
{

    std::cout << "Starting Mono VO node with Cam info...  " << std::endl;
    ros::init(argc, argv, "mono_vo");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/usb_cam/image_mono", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "usb_cam/camera_info", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> sync(image_sub, info_sub, 10);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2));
 //   image_transport::ImageTransport it(nh);
 //   image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("view");
    string trajectory_file = "/home/hekmat/catkin_ws/trajectory.txt";
    vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
    ifstream fin(trajectory_file);
    if (!fin) {
        cout << "cannot find trajectory file at " << trajectory_file << endl;
        return 1;
    }

    while (!fin.eof()) {
        double tx, ty, tz, qx, qy, qz, qw;
        fin >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
        Twr.pretranslate(Vector3d(tx, ty, tz));
        poses.push_back(Twr);
    }
    cout << "read total " << poses.size() << " pose entries" << endl;

    // draw trajectory in pangolin
    DrawTrajectory(poses);

}
