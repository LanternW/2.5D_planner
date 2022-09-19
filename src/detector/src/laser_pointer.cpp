#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <float.h>

const double pi = 3.1415926535;

using namespace std;
ros::Publisher target_pub;
bool has_odom = false;

Eigen::Vector3d odometry_pos;
double odometry_yaw = 0;   // from odom

double odometry_pitch = 0.9; // from imu

Eigen::Matrix4d Tw_c = Eigen::Matrix4d::Zero();
Eigen::Matrix3d K;

//camera/color/image_raw

void publishTarget(Eigen::Vector2d posi)
{
    double t;
    Eigen::Vector3d V, pc_ ,pw_ ,pw;

    // ->y  down:x
    V(0) = posi(1);
    V(1) = posi(0);
    V(2) = 1;

    pc_   = K.inverse() * V;
    pw_   = Tw_c.block(0,0,3,3) * pc_;
    if(pw_(2) >= 0)
    {
        return;
    }

    pw_  *= 2 / pw_(2);
    pw_  *= -1;


    pw = pw_ + odometry_pos;
    pw(2) = 0;
    //cout << pw_ <<endl;
    geometry_msgs::PoseStamped target;
    target.header.frame_id = "world";
    target.header.stamp    = ros::Time::now();

    if(pw_.norm() <= 10)
    {
        target.pose.position.x = pw(0);
        target.pose.position.y = pw(1);
        target.pose.position.z = 0;

        target_pub.publish(target);
    }
}

void OdomCallBack(nav_msgs::Odometry::ConstPtr msg)
{
   if(has_odom == false){ cout <<"[LASER_POINTER] has odometry "<<endl; }
   has_odom = true;
   odometry_pos[0] = msg->pose.pose.position.x;
   odometry_pos[1] = msg->pose.pose.position.y;
   odometry_pos[2] = msg->pose.pose.position.z;

   Eigen::Quaterniond q( msg->pose.pose.orientation.w,
			                msg->pose.pose.orientation.x,
		                  	msg->pose.pose.orientation.y,
		                  	msg->pose.pose.orientation.z );
   Eigen::Matrix3d R(q);
   odometry_yaw = atan2(R.col(0)[1],R.col(0)[0]);

   Eigen::Matrix3d Rw_c;

   Eigen::AngleAxisd rx( -(pi/2 + odometry_pitch), Eigen::Vector3d(1, 0, 0));
   Eigen::AngleAxisd rz( -(pi/2 + odometry_yaw), Eigen::Vector3d(0, 0, 1));
   Rw_c = rz.matrix() * rx.matrix() ;


   Tw_c.block(0,3,3,1) = odometry_pos;
   Tw_c.block(0,0,3,3) = Rw_c;

}

void ImgcallBack(sensor_msgs::Image::ConstPtr img)
{   
    static Eigen::Vector4d list_x = Eigen::Vector4d::Zero();
    static Eigen::Vector4d list_y = Eigen::Vector4d::Zero();
    static int index = 0;
    int list_n = 0;
    Eigen::Vector2d com = Eigen::Vector2d(0,0);
    int n = 0;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr    = cv_bridge::toCvCopy(img, img->encoding); 
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    cv::Mat cvColorImgMat = cv_ptr -> image;
    cv::Mat cvColorLABImgMat;
    cv::cvtColor(cvColorImgMat, cvColorLABImgMat, cv::COLOR_BGR2Lab);
    //cv::imshow("colorview",cvColorLABImgMat);

    std::vector<cv::Mat> rgbChannels(3);
    cv::split(cvColorLABImgMat, rgbChannels);

    //cv::imshow("L",rgbChannels[0]);

    cv::Mat output_pre = rgbChannels[2] ;
    cv::Mat output     = 0 * rgbChannels[2] ;

    for(int i = 0 ; i < output_pre.rows; i++)
    {
		for(int j = 0; j < output_pre.cols; j++)
		{

			//取出灰度图像中i行j列的点
            int t = output_pre.at<uchar>(i,j) - rgbChannels[1].at<uchar>(i,j);	
            if(t < 80)
            {
                output_pre.at<uchar>(i,j) = 0;
            }
            else
            {
                output_pre.at<uchar>(i,j) = t*2;
                n++;
                com = (n/(n+1.0)) * com + (1.0/(n+1.0)) * Eigen::Vector2d(i,j);
            }		
		}
	}

    //cv::imshow("B",output_pre);
    int bright = 0;
    for(int i = -2; i <= 2 ; i++)
    {
        for(int j = -2; j <= 2 ; j++)
        {
            if( com(0)+i >= 0 && com(0)+ i < output_pre.rows && com(1)+j >= 0 && com(1)+j < output_pre.cols)
            {
                bright += rgbChannels[0].at<uchar>( com(0)+i , com(1)+j );
            }
        }
    }
    list_x(index) = com(0);
    list_y(index) = com(1);
    index = (index + 1) % list_x.rows();

    com = Eigen::Vector2d(0,0);

    for(int i = 0 ; i < list_x.rows() ; i++)
    {
        if(list_x(i) > 0 && list_y(i) > 0)
        {
            com += Eigen::Vector2d( list_x(i) , list_y(i) );
            list_n++;
        }
    }

    com(0)  /= list_n;
    com(1)  /= list_n;


    if(bright > 25*100)
    {
        for(int i = -2; i <= 2 ; i++)
        {
            for(int j = -2; j <= 2 ; j++)
            {
                if( com(0)+i >= 0 && com(0)+ i < output_pre.rows && com(1)+j >= 0 && com(1)+j < output_pre.cols)
                {
                    output.at<uchar>(com(0)+i , com(1)+j) = 255;
                } 
            }
        }
    }
    cv::imshow("G",output);
    if(has_odom == true && !(com(0) != com(0)) && !(com(1) != com(1)))
    {
        publishTarget(com);
    }

    cv::waitKey(1);
    
}

void debugFunc()
{
    cv::Mat dst = cv::Mat(700 , 640, CV_8UC1, cv::Scalar(255));
    dst = 0*dst;
    cout << "K_inv = \n" << K.inverse() <<endl;

    Eigen::Matrix3d Rw_c;
    Eigen::AngleAxisd rx( -(pi/2 + 0.9), Eigen::Vector3d(1, 0, 0));
    Eigen::AngleAxisd rz( -(pi/2), Eigen::Vector3d(0, 0, 1));
    Rw_c = rz.matrix() * rx.matrix() ;

    Eigen::Vector3d V;
    V(2) = 1;

    V(0) = V(1) = 0;
    dst.at<uchar>(0,0) = 255;
    cout << "0,0 = \n" << K.inverse() * V <<endl<<endl;
    cout << "0,0 = \n" << Rw_c * K.inverse() * V <<"==============\n"<<endl;

    V(1) = 480;
    dst.at<uchar>(0,480) = 200;
    cout << "0,480 = \n" << K.inverse() * V <<endl<<endl;
    cout << "0,480 = \n" << Rw_c * K.inverse() * V <<"==============\n"<<endl;


    V(0) = 640;
    dst.at<uchar>(640,480) = 155;
    cout << "640,480 = \n" << K.inverse() * V <<endl<<endl;
    cout << "640,480 = \n" << Rw_c * K.inverse() * V <<"==============\n"<<endl;


    V(1) = 0;
    dst.at<uchar>(640,0) = 100;
    cout << "640,0 = \n" << K.inverse() * V <<endl<<endl;
    cout << "640,0 = \n" << Rw_c * K.inverse() * V <<"==============\n"<<endl;

    //cv::imshow("R",dst);
    //cv::waitKey(1000);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_pointer_node");
  ros::NodeHandle nh("~");

  ros::Subscriber coloredimg_sub  = nh.subscribe("raw_image", 1000, ImgcallBack); 
                      target_pub  = nh.advertise<geometry_msgs::PoseStamped>("target", 5); 

  ros::Subscriber odometry_sub    = nh.subscribe("odometry", 10, OdomCallBack); 

  Tw_c(3,3) = 1;

  K<< 370.5,  0,    320  , 
        0,  370.5, 240  , 
        0,   0,     1   ;

  ROS_INFO("laser pointer already.");
  //debugFunc();


  ros::spin();
  return 0;
}