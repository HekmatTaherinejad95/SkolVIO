/* Compute the odometry using images from a Webcam */

#include "VisualOdometry/VisualOdometry.h"

int main(int argc, char **argv)
{
    ros::init (argc, argv, "drone_visual_odometry");
    ros::NodeHandle node;
    namedWindow("RGB", CV_WINDOW_AUTOSIZE);
    namedWindow("OUTPUT", CV_WINDOW_AUTOSIZE);

    VisualOdometry odom;

    VideoCapture cap(0);// "rtsp://192.168.1.254/sjcam.mov");
    //VideoCapture cap("/home/lsi/test.avi");
    if (!cap.isOpened())
    {
        cout << "error in VideoCapture: check device" << endl;
        return 1;
    }

    // Grab the first frame and init the odometry first values
    Mat frame;
    cap >> frame;
    odom.init_frame(frame);

    char keypressed = 0;

    while (keypressed != 'q')
    {
        cap >> frame;
        cout << "frame type: " << frame.type() << endl;
        cout << "frame dims: " << frame.dims << endl;
        odom.compute_next(frame);
        keypressed = waitKey(1) & 0x00ff;
    }

    destroyAllWindows();
    return 0;
}