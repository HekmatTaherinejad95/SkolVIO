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
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "../include/ros_mono_vo/vo_features.h"
#include "../include/ros_mono_vo/Drawtrajectory.h"
#include "../include/ros_mono_vo/SkolVIO.h"
#include "../include/ros_mono_vo/util.h"
#include "../include/ros_mono_vo/util.cpp"


using namespace std;
using namespace Eigen;


inline float SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

inline float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}

SkolVIO::SkolVIO(int max_frame, int min_num_pts, std::string dataset_path)
        : max_frame_(max_frame), min_num_pts_(min_num_pts), dataset_path_(dataset_path), id_(0)
{

    data_calib_ = dataset_path_ + "/sequences/02/calib.txt";
    data_poses_ = dataset_path_ + "/poses/02.txt";
    data_images_ = dataset_path_ + "/sequences/02/image_1/";


    img_traj_ = cv::Mat::zeros(600, 600, CV_8UC3);


    SkolVIO::FetchIntrinsicParams();

    SkolVIO::Initialization();
}


void SkolVIO::ReduceVector(std::vector<int> &v) {
    int j=0;

    for(int i=0; i<int(v.size()); i++) {
        if(status_[i]) {
            v[j++] = v[i];
        }
    }
    v.resize(j);
}
void SkolVIO::FeatureTracking(cv::Mat img1,
                             cv::Mat img2,
                             std::vector<cv::Point2f> &p1,
                             std::vector<cv::Point2f> &p2)
{
    std::vector<float> error;
    cv::Size window_size = cv::Size(21,21);
    cv::TermCriteria tc = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

    // Feature tracking using lucas-kanade tracker.
    cv::calcOpticalFlowPyrLK(img1, img2, p1, p2, status_, error, window_size, 3, tc, 0, 0.001);

    // Getting rid of points for which the KLT tracking failed or those who have gone outside the frame.
    int index_correction=0;

    for(int i=0; i<status_.size(); i++) {
        cv::Point2f pt = p2.at(i - index_correction);

        if((status_.at(i)==0) || (pt.x<0) || (pt.y<0)) {
            if((pt.x<0) || (pt.y<0)) {
                status_.at(i) = 0;
            }

            p1.erase(p1.begin() + (i-index_correction));
            p2.erase(p2.begin() + (i-index_correction));
            index_correction++;
        }
    }
}
void SkolVIO::FeatureDetection(cv::Mat img, std::vector<cv::Point2f> &p)
{
    std::vector<cv::KeyPoint> kpt;

    // Extract features using FAST algorithm.
    cv::FAST(img, kpt, 20, true);

    // Sort keypoints ascending by response.
    std::sort(kpt.begin(), kpt.end(), [](const cv::KeyPoint &a, const cv::KeyPoint &b) {
        return a.response > b.response;
    });

    // Take the minimum number of feature point.
    for(int i=0; i<min_num_pts_; i++) {
        p.push_back(kpt[i].pt);
    }
}


void SkolVIO::FetchIntrinsicParams() {
    // Read the calib.txt file.
    std::ifstream fin(data_calib_);
    std::string line;

    if(fin.is_open()) {
        std::getline(fin, line);
        std::istringstream iss(line);

        for(int j=0; j<13; j++) {
            std::string token;
            iss >> token;
            if(j==1) {
                f_ = std::stod(token);   // focal length
            }
            if(j==3) {
                pp_.x = std::stod(token); // principal point x
            }
            if(j==7) {
                pp_.y = std::stod(token); // printcipal point y
            }
        }
        fin.close();
    }
    else {
        std::cout << "[-] Cannot read the calibration file: " << data_calib_ << std::endl;
        f_ = 0;
        pp_ = cv::Point2f(0,0);
    }
}

void SkolVIO::ComputeAbsoluteScale(int num_frame)
{
    std::string line;
    int i=0;
    std::ifstream fin(data_poses_);

    double x=0,y=0,z=0;
    double prev_x, prev_y, prev_z;

    if(fin.is_open()) {
        while((std::getline(fin, line)) && (i<=num_frame)) {
            prev_z = z;
            prev_y = y;
            prev_x = x;

            std::istringstream iss(line);
            for(int j=0; j<12; j++) {
                iss>>z;
                if(j==7) y=z;
                if(j==3) x=z;
            }
            i++;
        }
        fin.close();
    }
    else {
        std::cout << "[-] Unable to open file: " << data_poses_ << std::endl;
        scale_ = 0;
        return;
    }

    scale_ = std::sqrt( (x-prev_x)*(x-prev_x) +
                        (y-prev_y)*(y-prev_y) +
                        (z-prev_z)*(z-prev_z));
}

void SkolVIO::Initialization() {
    // initialize (using first two frames).
    scale_ = 1.0;

    std::string data1 = data_images_ + util::AddZeroPadding(0, 6) + ".png";
    std::string data2 = data_images_ + util::AddZeroPadding(1, 6) + ".png";

    // Load the fist two images.
    cv::Mat img1 = cv::imread(data1, cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread(data2, cv::IMREAD_GRAYSCALE);

    // Feature extraction and tracking.
    std::vector<cv::Point2f> p1, p2;
    FeatureDetection(img1, p1);
    FeatureTracking(img1, img2, p1, p2);

    // Find essential matrix E and recover R,t from E.
    E_ = cv::findEssentialMat(p2, p1, f_, pp_, cv::RANSAC, 0.999, 1.0, mask_);
    cv::recoverPose(E_, p2, p1, curr_R_, curr_t_, f_, pp_, mask_);

    prev_img_ = img2;
    prev_pts_ = p2;

    prev_R_ = curr_R_.clone();
    prev_t_ = curr_t_.clone();
}
void SkolVIO::Visualize(int num_frame) {
    cv::Mat effect = cv::Mat::zeros(cv::Size(dst_.cols, dst_.rows), CV_8UC3);

    // Draw tracking image.
    int visual_limit = 5000;
    if (visual_limit > min_num_pts_) {
        visual_limit = min_num_pts_;
    }

    for (size_t i = 0; i < visual_limit; i++) {
        cv::circle(effect, prev_pts_[i], 2, cv::Scalar(0, 0, 255), 2);  // previous features (red).
        cv::circle(effect, curr_pts_[i], 2, cv::Scalar(255, 0, 0), 2);  // current features (blue).
    }

    std::map<int, cv::Point2f>::iterator mit;
    for (size_t i = 0; i < visual_limit; i++) {
        int id = idx_[i];
        mit = prev_pts_map_.find(id);

        if (mit != prev_pts_map_.end()) {
            cv::arrowedLine(effect, curr_pts_[i], mit->second, cv::Scalar(255, 255, 255), 1, 16, 0,
                            0.1); // LK optical flow (white)
        }
    }
}


void SkolVIO::PoseTracking(int num_frame) {
    // pose tracking starts from the third image.
    if(num_frame > max_frame_) {
        return;
    }

    std::string data = data_images_ + util::AddZeroPadding(num_frame, 6) + ".png";
    cv::Mat curr_img_color = cv::imread(data);
    cv::cvtColor(curr_img_color, curr_img_, cv::COLOR_BGR2GRAY);

    // Feature tracking.
    FeatureTracking(prev_img_, curr_img_, prev_pts_, curr_pts_);

    ReduceVector(idx_);

    // Find essential matrix E and recover R,t from E.
    E_ = cv::findEssentialMat(curr_pts_, prev_pts_, f_, pp_, cv::RANSAC, 0.999, 1.0, mask_);
    cv::recoverPose(E_, curr_pts_, prev_pts_, curr_R_, curr_t_, f_, pp_, mask_);

    // Get the scale.
    ComputeAbsoluteScale(num_frame);

    if((scale_>0.1) &&
       (curr_t_.at<double>(2) > curr_t_.at<double>(0)) &&
       (curr_t_.at<double>(2) > curr_t_.at<double>(1)))
    {
        prev_t_ += scale_ * (prev_R_*curr_t_);
        prev_R_ = curr_R_ * prev_R_;
    }

    // If we have not enough features, try feature extraction again.
    if(prev_pts_.size() < min_num_pts_) {
        FeatureDetection(prev_img_, prev_pts_);
        FeatureTracking(prev_img_, curr_img_, prev_pts_, curr_pts_);
    }

    for(int i=0; i<curr_pts_.size(); i++) {
        idx_.push_back(id_++);
    }

    for(int i=0; i<curr_pts_.size(); i++) {
        prev_pts_map_.insert(std::make_pair(idx_[i], curr_pts_[i]));
    }

    dst_ = curr_img_color.clone();

    // Draw result.
    SkolVIO::Visualize(num_frame);

    prev_img_ = curr_img_.clone();
    prev_pts_ = curr_pts_;
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
void Print_Quaternion(Eigen::Quaterniond &q){
    std::cout<<"["<<q.w()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<"]"<<std::endl;
}


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
              focal_length = 51.0 ;
              std::cout << "Focal Length: " << focal_length << std::endl;
              cv::Point2d ppoint(0, 0); //principle point

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

          cv::Point2d ppoint(0,  0); //principle point
          std::cout << "principle point" << std::endl;
          cout << *cam_info << endl;
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
    //cv::namedWindow("view");
    //cv::startWindowThread();
    ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 1);
    ros::Publisher pub_image = nh.advertise<sensor_msgs::Image>("image", 1);
    ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("trajectory",1, true);
    int max_frame  = 4600;          // maximum frame to play in KITTI sequence.
    int min_num_pts = 400;       // minimum number of feature points.
    std::string dataset_path = "/home/hekmat/DATA/dataset";

    tf::TransformBroadcaster tf_broadcaster;
    tf::StampedTransform transform;
    transform.frame_id_ = "world";
    transform.child_frame_id_ = "/camera";

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    nav_msgs::Path path;
    path.header.stamp=current_time;
    path.header.frame_id="/world";
/*
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/usb_cam/image_mono", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "usb_cam/camera_info", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> sync(image_sub, info_sub, 10);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2));
 //   image_transport::ImageTransport it(nh);
 //   image_transport::Subscriber sub = it.subscribe("usb_cam/image_raw", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("view");
*/

    SkolVIO* vo = new SkolVIO(max_frame, min_num_pts, dataset_path);

    // Starts from the third image (first and second images are used for the initialization).
    int num_frame = 2;
    while(ros::ok()) {
        if(num_frame >= max_frame) {
            break;
        }
        current_time = ros::Time::now();
        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        // Perform pose tracking for the n-th frame.
        vo->PoseTracking(num_frame);

        // Get rotation and translation from vo.
        cv::Mat R = vo->GetRotation();
        cv::Mat t = vo->GetTranslation();
        cv::Vec3f euler = util::RotMatToEuler(R);
        tf::Matrix3x3 _R(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                         R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                         R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
        tf::Quaternion quat;
        _R.getRotation(quat); // convert rotation matrix to quaternion.

        // Set the digit precision.
        cv::Ptr<cv::Formatter> round = cv::Formatter::get(cv::Formatter::FMT_DEFAULT);
        round->set64fPrecision(3);
        round->set32fPrecision(3);

        std::cout << std::endl << "[+] " << util::AddZeroPadding(num_frame,6) << std::endl;
        std::cout << "Rotation(euler):  " <<  std::setprecision(3) << "[" << euler.val[0] << ", " << euler.val[1] << ", " << euler.val[2] << "]" << std::endl;
        std::cout << "Translate(x,y,z): " << round->format(t.t()) << std::endl;

        transform.stamp_ = ros::Time::now();
        transform.setRotation(tf::Quaternion(quat[0], quat[1], quat[2], quat[3]));
        transform.setOrigin(tf::Vector3(t.at<double>(0), t.at<double>(1), t.at<double>(2)));

        // Braodcast the transform between /world and /camera.
        tf_broadcaster.sendTransform(transform);



        geometry_msgs::PoseStamped this_pose_stamped;
        geometry_msgs::Quaternion odom_quat;
        // Set the position and rotation.
        this_pose_stamped.pose.position.x = t.at<double>(0);
        this_pose_stamped.pose.position.z = 0.0; // t.at<double>(1);
        this_pose_stamped.pose.position.y = t.at<double>(2);
        this_pose_stamped.pose.orientation.x = quat[0];
        this_pose_stamped.pose.orientation.y = quat[1];
        this_pose_stamped.pose.orientation.z = quat[2];
        this_pose_stamped.pose.orientation.w = quat[3];

        this_pose_stamped.header.stamp=current_time;
        this_pose_stamped.header.frame_id="world";
        path.poses.push_back(this_pose_stamped);


        // publish to /odom.
        pub_path.publish(path);

        // Get the result image from vo.
        cv::Mat dst = vo->GetResultImage();
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        cv_ptr->encoding = "bgr8";
        cv_ptr->header.stamp = ros::Time::now();
        cv_ptr->header.frame_id = "/world";
        cv_ptr->image = dst;

        // publish to /image.
        pub_image.publish(cv_ptr->toImageMsg());

        ifstream fin;
        ofstream fout;
        fin.open ("trajectory.txt", ios::out | ios::in);
        fout.open("trajectory.txt", ios::app | ios::out);
        fout << t.at<double>(0) <<" "<< 0.0 << " " << t.at<double>(2) << " "<< quat[0] << " " << quat[1] << " " << quat[2] <<" " << quat[3] << endl;
        last_time = current_time;
        num_frame++;
    }

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
    return 0;

}
