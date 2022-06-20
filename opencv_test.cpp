#include <ros/ros.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/stitching.hpp>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/calib3d.hpp>
#include <tf/transform_listener.h>

geometry_msgs::Twist vel_cmd;
int weight=1920;
int height=1080;
//定义了图像大小

using namespace cv;
using namespace std;
//命名空间

char contorl_mode=0;
char last_mode=0;

//fps和模式
std_msgs::Float64 fps;
std_msgs::Int8 aruco_mode;
std_msgs::Int8 cv_mode;

//中间status
char cv_flag = 0;
int codec;
double EXPOSURE;

//两个回调函数
void control_mode_callback(const std_msgs::Int8& msg){
  contorl_mode = msg.data;
}
void last_mode_callback(const std_msgs::Int8& msg){
  last_mode = msg.data;
}

//定义两个Mat
Mat cameraMatrix= (Mat_<double>(3, 3) << 948.121121,0.000000,1002.985457,   0.000000,951.649971,550.980305,   0.000000,0.000000,1.000000);
Mat distCoeffs = (Mat_<double>(1, 5) << -0.251095, 0.040948, -0.001961, -0.002998, 0.000000);

//变量定义
double delaytime1;
double delaytime2;
int debug;
double angle1;
double angle2;
double angle3;

double pictime;
double vel;

double speed;
double yaw;
double aim_yaw;
double secs;
double secs1;
char a = 10;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom){

  //ROS_INFO("------");
  yaw=tf::getYaw(odom->pose.pose.orientation);
  
  //yaw = yaw<0 ? yaw : yaw - 6.28;
  //ROS_INFO("now_yaw:%f",yaw);

}




int main(int argc, char** argv)
{ ros::init(argc, argv, "opencv_test");//初始化ros节点
  ros::NodeHandle nh_("~");//开启一个节点
  ros::Rate loop_rate(100);//循环的频率

//发布话题
	ros::Publisher cv_mode_pub = nh_.advertise<std_msgs::Int8>("/cv_mode", 1000);
    ros::Publisher aruco_pub = nh_.advertise<std_msgs::Int8>("/aruco_mode", 1000);
	ros::Publisher vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	
	ros::Subscriber control_mode_sub = nh_.subscribe("/control_mode",10,control_mode_callback);
	ros::Subscriber last_mode_sub = nh_.subscribe("/last_mode",10,last_mode_callback);
	ros::Subscriber odom_sub = nh_.subscribe("/odom",1000,odomCallback);
	
	image_transport::ImageTransport transport(nh_);
    image_transport::Publisher image_pub;
    image_pub=transport.advertise("/OutImage", 10000);



    //launch文件中给了定义
  nh_.getParam("delaytime1",delaytime1);
  nh_.getParam("delaytime2",delaytime2);
  nh_.getParam("pictime",pictime);
  nh_.getParam("vel",vel);
  nh_.getParam("debug",debug);
  nh_.getParam("angle1",angle1);
  nh_.getParam("angle2",angle2);
  nh_.getParam("angle3",angle3);
  
  
  
//定义图像变量
  Mat image1;
  Mat image2;
  Mat image;
  Mat img;
  Mat imgC;
  Mat imgR;
  Rect rect(300, 0, 1320, 1080);
  Rect rect1(400, 400, 1200, 680);


  VideoCapture capture;//创造一个视频类
  /*VideoCapture capture1;
  VideoCapture capture;
  VideoCapture capture;*/
  
    capture.open(0);//打开默认的摄像头
    
    codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
    //是动态jpeg编解码器等
	capture.set(CAP_PROP_AUTOFOCUS, false);
	capture.set(CAP_PROP_FOURCC, codec);
	capture.set(CAP_PROP_FRAME_WIDTH , weight);
	capture.set(CAP_PROP_FRAME_HEIGHT , height);
	if(!capture.isOpened()){
    ROS_ERROR("cam do not opened");
    assert(0);
  }
  
  
	capture >> img;//获取原始图像
	if(img.empty()){
    ROS_ERROR("NO IMAGE in biginning");
    assert(0);
	}
	
	
  while(ros::ok()){

    capture >> img;

    flip(image1, image, 1);
    cv_bridge::CvImage cvi;
    cvi.header.stamp = time;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    cvi.image = image; 
    sensor_msgs::Image im;
    cvi.toImageMsg(im);
    image_pub.publish(im);
    
    
///////////////////此部分代码为aruco识别/////////////////////////////
    if(contorl_mode == 2){

	    ROS_INFO("aruco detect");
      if(cv_flag == 0){
				if(cv_mode.data != 1){
					cv_mode.data=0;
					cv_mode_pub.publish(cv_mode);
				}
				
				capture >> image1;//获取原始图像
	      if(image1.empty()){
          ROS_ERROR("NO IMAGE IN aruco");
          assert(0);
	      }
	      flip(image1, image2, 1);//原始图像翻转 image1 --> image2
	      undistort (image2, image, cameraMatrix, distCoeffs);//翻转后图像畸变矫正 image2 --> image
	      //resize(image,imgR,Size(640,360));//图片压缩
	      imgR = Mat(image, rect1);
	      
				cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
				cv::Ptr<cv::aruco::DetectorParameters> params = aruco::DetectorParameters::create();
				params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
				std::vector<int> ids;
				std::vector<std::vector<cv::Point2f>> corners, rejected;
				cv::aruco::detectMarkers(image, dictionary, corners, ids, params);

				// if at least one marker detected
        if (ids.size() > 0) {
          if(ids[0]<3){//应该为0、1、2之中的一个

						if(ids[0] == 0){
						  //system("play ~/ucar-master/src/mp3/shucai.mp3");
						  ROS_INFO("shu cai");
						  a = 0;
						}else if(ids[0] == 1){
						  //system("play ~/ucar-master/src/mp3/shuiguo.mp3");
						  ROS_INFO("shui guo");
						  a = 1;
						}else{
						  //system("play ~/ucar-master/src/mp3/roulei.mp3");
						  ROS_INFO("rou lei");
						  a = 2;
						}
						aruco_mode.data=ids[0];
						aruco_pub.publish(aruco_mode);
						cv_mode.data=1;
						cv_mode_pub.publish(cv_mode);
						cv_flag = 1;
						
            
            ROS_INFO("aruco_mode: %d",ids[0]);
          }
        }
        imwrite("/home/ucar/ucar-master/src/opencv_test/src/aruco/arucor.jpg", imgR);// save image
        imwrite("/home/ucar/ucar-master/src/opencv_test/src/aruco/image.jpg", image);// save image
				ROS_INFO("aruco picture saved");
			}
			if(cv_flag == 1){
				if(last_mode == 20){
				  
					//cv_mode.data=1;
					//cv_mode_pub.publish(cv_mode);
					if(a == 0){
					  system("play ~/ucar-master/src/mp3/B_shucai.mp3");
					  last_mode = 30;
					}else if(a == 1){
					  system("play ~/ucar-master/src/mp3/B_shuiguo.mp3");
					  last_mode = 30;
					}else if(a == 2){
					  system("play ~/ucar-master/src/mp3/B_roulei.mp3");
					  last_mode = 30;
					}else{ROS_INFO("Waitting B Saying");}
				}
			}
    }
////////////////////////////////////////////////////////////////////////////////



///////////////////////此部分代码为获取识别图像///////////////////////////////////////
		if(contorl_mode == 4){
			if(cv_flag == 1){
				if(cv_mode.data != 3){
					cv_mode.data=2;
					cv_mode_pub.publish(cv_mode);
				}
			  //if(angle1 < 10){
          ////////////////// pic1 //////////////////////////
          ////////////////// pic1 //////////////////////////
          ////////////////// pic1 //////////////////////////
          /*aim_yaw = angle1;
          while(abs(aim_yaw - yaw) > 0.05){
            capture >> img;
            ROS_INFO("picture 1 save: nnnnOK aim_yaw:%2f yaw:%2f",aim_yaw,yaw);
  	  			vel_cmd.linear.x = 0;//纵向运动
  	  			vel_cmd.linear.y = 0;//横向运动，右手坐标系
  	  			vel_cmd.angular.z = (aim_yaw - yaw)*vel;//自旋运动，右手坐标系
  	  			vel_pub.publish(vel_cmd);
  	  			ros::spinOnce();
          }*/
          if(pictime > 0){
			  		secs = ros::Time::now().toSec();
			  		for(secs1 = ros::Time::now().toSec(); secs1 - secs< pictime; secs1 =ros::Time::now().toSec()){
			  			capture >> img;
			  			vel_cmd.linear.x = 0;//纵向运动
			  			vel_cmd.linear.y = 0;//横向运动，右手坐标系
			  			vel_cmd.angular.z = 0;//自旋运动，右手坐标系
			  			vel_pub.publish(vel_cmd);
			  		}
			  	}
	    		capture >> image1;//获取原始图像
	    		flip(image1, image2, 1);//原始图像翻转 image1 --> image2
	    		undistort(image2, image, cameraMatrix, distCoeffs);//原始图像畸变矫正 image2 --> image
	    		//imgC = Mat(image, rect);
	    		imwrite("/home/ucar/ucar-master/src/opencv_test/src/yoloP/1.jpg", image);// save image获取第一张图片
	    		ROS_INFO("picture 1 save: OK");
		  	//}
		  	
			  if(angle2 < 10){
          ////////////////// pic2 //////////////////////////
          ////////////////// pic2 //////////////////////////
          ////////////////// pic2 //////////////////////////
          aim_yaw = angle2;
          while(abs(aim_yaw - yaw) > 0.05){
            capture >> img;
            ROS_INFO("picture 2 save: nnnnOK aim_yaw:%2f yaw:%2f",aim_yaw,yaw);
  	  			vel_cmd.linear.x = 0;//纵向运动
  	  			vel_cmd.linear.y = 0;//横向运动，右手坐标系
  	  			vel_cmd.angular.z = (aim_yaw - yaw)*vel;//自旋运动，右手坐标系
  	  			vel_pub.publish(vel_cmd);
  	  			ros::spinOnce();
          }
          if(pictime > 0){
			  		secs = ros::Time::now().toSec();
			  		for(secs1 = ros::Time::now().toSec(); secs1 - secs< pictime; secs1 =ros::Time::now().toSec()){
			  			capture >> img;
			  			vel_cmd.linear.x = 0;//纵向运动
			  			vel_cmd.linear.y = 0;//横向运动，右手坐标系
			  			vel_cmd.angular.z = 0;//自旋运动，右手坐标系
			  			vel_pub.publish(vel_cmd);
			  		}
			  	}
	    		capture >> image1;//获取原始图像
	    		flip(image1, image2, 1);//原始图像翻转 image1 --> image2
	    		undistort(image2, image, cameraMatrix, distCoeffs);//原始图像畸变矫正 image2 --> image
	    		//imgC = Mat(image, rect);
	    		imwrite("/home/ucar/ucar-master/src/opencv_test/src/yoloP/2.jpg", image);// save image获取第一张图片
	    		ROS_INFO("picture 2 save: OK");
		  	}
		  	
			  if(angle3 < 10){
          ////////////////// pic3 //////////////////////////
          ////////////////// pic3 //////////////////////////
          ////////////////// pic3 //////////////////////////
          aim_yaw = angle3;
          while(abs(aim_yaw - yaw) > 0.05){
            capture >> img;
            ROS_INFO("picture 3 save: nnnnOK aim_yaw:%2f yaw:%2f",aim_yaw,yaw);
  	  			vel_cmd.linear.x = 0;//纵向运动
  	  			vel_cmd.linear.y = 0;//横向运动，右手坐标系
  	  			vel_cmd.angular.z = (aim_yaw - yaw)*vel;//自旋运动，右手坐标系
  	  			vel_pub.publish(vel_cmd);
  	  			ros::spinOnce();
          }
          if(pictime > 0){
			  		secs = ros::Time::now().toSec();
			  		for(secs1 = ros::Time::now().toSec(); secs1 - secs< pictime; secs1 =ros::Time::now().toSec()){
			  			capture >> img;
			  			vel_cmd.linear.x = 0;//纵向运动
			  			vel_cmd.linear.y = 0;//横向运动，右手坐标系
			  			vel_cmd.angular.z = 0;//自旋运动，右手坐标系
			  			vel_pub.publish(vel_cmd);
			  		}
			  	}
	    		capture >> image1;//获取原始图像
	    		flip(image1, image2, 1);//原始图像翻转 image1 --> image2
	    		undistort(image2, image, cameraMatrix, distCoeffs);//原始图像畸变矫正 image2 --> image
	    		//imgC = Mat(image, rect);
	    		imwrite("/home/ucar/ucar-master/src/opencv_test/src/yoloP/3.jpg", image);// save image获取第一张图片
	    		ROS_INFO("picture 3 save: OK");
		  	}
				
				cv_mode.data=3;
				cv_mode_pub.publish(cv_mode);
				cv_flag = 2;
			}
		}
///////////////////////////////////////////////////////////////////////////////


if(contorl_mode == 6){
  exit(EXIT_SUCCESS);
}
///////////////////////此部分代码为拍图///////////////////////////////////////
		if(contorl_mode == 14){
	  	/*secs = ros::Time::now().toSec();
	  	for(secs1 = ros::Time::now().toSec(); secs1 - secs< 1; secs1 =ros::Time::now().toSec()){
	  		capture >> img;
  			vel_cmd.linear.x = 0;//纵向运动
	  		vel_cmd.linear.y = 0;//横向运动，右手坐标系
		  	vel_cmd.angular.z = 1;//自旋运动，右手坐标系
				vel_pub.publish(vel_cmd);
			}*/
			secs = ros::Time::now().toSec();
			for(secs1 = ros::Time::now().toSec(); secs1 - secs< 1.3; secs1 =ros::Time::now().toSec()){
				capture >> img;
  			vel_cmd.linear.x = 0;//纵向运动
	  		vel_cmd.linear.y = 0;//横向运动，右手坐标系
		  	vel_cmd.angular.z = 0;//自旋运动，右手坐标系
				vel_pub.publish(vel_cmd);
			}
			
      capture >> image1;//获取原始图像
	    if(image1.empty()){
        ROS_ERROR("NO IMAGE IN photo");
        assert(0);
	    }
	    flip(image1, image2, 1);//原始图像翻转 image1 --> image2
	    undistort (image2, image, cameraMatrix, distCoeffs);//翻转后图像畸变矫正 image2 --> image
	    
	    string imgname, imgnamer;
      static int i = 0;
      i++;

      imgname = "/home/ucar/ucar-master/src/opencv_test/src/photo/"+to_string(i)+".jpg";
      imgnamer = "/home/ucar/ucar-master/src/opencv_test/src/photo/"+to_string(i)+"r.jpg";
      imwrite(imgname, image);
      imwrite(imgnamer, image2);
      
      imgname = "/home/ucar/ucar-master/src/opencv_test/src/photo/"+to_string(i)+"arcuo.jpg";
      resize(image,imgR,Size(640,360));
      imgR = Mat(imgR, rect1);
      imwrite(imgname, imgR);
      
      imgname = "/home/ucar/ucar-master/src/opencv_test/src/photo/"+to_string(i)+"yolo.jpg";
      imgC = Mat(image, rect);
      imwrite(imgname, imgC);
      
      ROS_INFO("picture save: OK");
      contorl_mode = 15;
      /*if(i > 800){
      contorl_mode = 15;}*/
		}
		
		
///////////////////////此部分代码为国塞识别任务增加的拍图///////////////////////////////////////
		if(last_mode > 1 && last_mode < 10){
		  ROS_INFO("begin");
      if(pictime > 0){
	  		secs = ros::Time::now().toSec();
	  		for(secs1 = ros::Time::now().toSec(); secs1 - secs< pictime; secs1 =ros::Time::now().toSec()){
	  			capture >> img;
	  			vel_cmd.linear.x = 0;//纵向运动
	  			vel_cmd.linear.y = 0;//横向运动，右手坐标系
	  			vel_cmd.angular.z = 0;//自旋运动，右手坐标系
	  			vel_pub.publish(vel_cmd);
	  		}
	  	}
			
			capture >> image1;//获取原始图像
			if(image1.empty()){
				ROS_ERROR("NO IMAGE IN photo");
				assert(0);
			}
			flip(image1, image2, 1);//原始图像翻转 image1 --> image2
			undistort (image2, image, cameraMatrix, distCoeffs);//翻转后图像畸变矫正 image2 --> image
			//imgC = Mat(image, rect);
			
			ROS_INFO("end");
		  if(last_mode == 2){
        imwrite("/home/ucar/ucar-master/src/opencv_test/src/yoloP/d1.jpg", image);// save image获取第一张图片
        ROS_INFO("picture d1 save: OK");
        last_mode = 0;
				cv_mode.data=10;
				cv_mode_pub.publish(cv_mode);
		  }if(last_mode == 3){
		    imwrite("/home/ucar/ucar-master/src/opencv_test/src/yoloP/d2.jpg", image);// save image获取第一张图片
		    ROS_INFO("picture d2 save: OK");
		    
        aim_yaw = 0.1;
        while(abs(aim_yaw - yaw) > 0.05){
          capture >> img;
          ROS_INFO("picture 2 save: nnnnOK aim_yaw:%2f yaw:%2f",aim_yaw,yaw);
  				vel_cmd.linear.x = 0;//纵向运动
  				vel_cmd.linear.y = 0;//横向运动，右手坐标系
  				vel_cmd.angular.z = (aim_yaw - yaw)*vel;//自旋运动，右手坐标系
  				vel_pub.publish(vel_cmd);
  				ros::spinOnce();
        }
		    last_mode = 0;
				cv_mode.data=11;
				cv_mode_pub.publish(cv_mode);
		  }
		  
			
		}
		ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

