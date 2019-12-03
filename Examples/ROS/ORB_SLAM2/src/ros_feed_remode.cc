/*

Written by Fangwen Shu (Fangwen.Shu@dfki.de), 02.10.2019.

This code is simply based on ROS, and it is used to feed data into REMODE, 
of which the images are geo-referenced by GPS directly.

No need to initialize ORB-SLAM, I just borrowed the environment, 
too lazy to create a new catkin workspace :)

Input:  argv[0]: Remode_feeder
        argv[1]: "path" of image_folder.
        argv[2]: "txt file" which saves corresponding poses.
        argv[3]: "loop rate" which control ros::rate()

Output: no explicit output,
        the data will be published/broadcasted through ROS node, 
        while REMODE subscribe/listen those data through its node.

Problem left: 
    If you run REMODE with this version of ORB-SLAM2, 
    the scene depth (max_z and min_z) will be calculated by Frame::getSceneDepth() which was added for REMODE.
    These two values are calculated from map points of current frame. 
    So, you can imagine, for now you can only set these values manually without calculate sparse point cloud through SLAM.

    How to propely hard-code max_z and min_z, this is a problem which not yet sovled.

    My suggestion is, even we have GPS referenced imagery, we can run the tracking module of ORB-SLAM to support REMODE, 
    meanwhile, the calculated poses can be used to verify the poses measured by GPS. It is a double guarantee.

    So, anyway, we can extract some functionality from SLAM as we do not need every single module of SLAM, 
    and conbine it with GPS as a new system. 
    
*/

#include <iostream>
#include<fstream>
#include<algorithm>
#include<chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <DenseInput.h>

using namespace std;

void LoadData(const string &strImagePath, const string &strPathPoseTxt,
                vector<string> &vstrImages, vector<double> &vTimeStamps,
                vector<double> &vtx, vector<double> &vty, vector<double> &vtz,
                vector<double> &vqx, vector<double> &vqy, vector<double> &vqz, vector<double> &vqw);
                
string modify_img_name(int index_num);


int main(int argc, char **argv)
{
	if(argc != 4)
	{
		cerr << endl << "Usage: rosrun ORB_SLAM2 Remode_feeder path_to_image_folder path_to_pose_txt_file loop_rate" << endl;        
		ros::shutdown();
		return 1;
	}

    // print and check
    cout << endl;
    cout << "--------------------------READ PATH-----------------------" << endl;
    cout << "image_folder: " << argv[1] << endl;
    cout << "pose_txt_file: " << argv[2] << endl;
    cout << "loop_rate: " << argv[3] << endl;
    cout << "----------------------------------------------------------" << endl;

    // load data: images, poses and timestamps
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    vector<double> vTx;
    vector<double> vTy;
    vector<double> vTz;
    vector<double> vQx;
    vector<double> vQy;
    vector<double> vQz;
    vector<double> vQw;

    LoadData(string(argv[1]), string(argv[2]),
                vstrImageFilenames, vTimestamps, 
                vTx, vTy, vTz, vQx, vQy, vQz, vQw);

    int nImages = vstrImageFilenames.size();
    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    // Publishing/broadcasting timestamped image and poses
    ros::init(argc, argv, "Remode_feeder");  // Initialize ROS
    ros::start();
    ros::NodeHandle nodeHandler;
    ros::Publisher pose_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("/camera_pose",1);
    ros::Publisher pub_dense = nodeHandler.advertise<svo_msgs::DenseInput>("/ORB/DenseInput",1);

    // Main loop
    cv::Mat im;
    ros::Rate rate(atoi(argv[3]));  // Hz, adjust it according to your dataset
    for (int ni=0; ni<nImages; ni++)
    {
        // read image
        im = cv::imread(vstrImageFilenames[ni]);
        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImageFilenames[ni] << endl;
            return 1;
        }

        // pose
        tf::Vector3 V(vTx[ni], vTy[ni], vTz[ni]);
  	    tf::Quaternion q(vQx[ni], vQy[ni], vQz[ni], vQw[ni]); //  (x,y,z,w)
        tf::Matrix3x3 M(q);

        cv::Mat TWC = (cv::Mat_<double>(4,4) << M[0][0], M[0][1], M[0][2], V[0],
                                               M[1][0], M[1][1], M[1][2], V[1],
                                               M[2][0], M[2][1], M[2][2], V[2],
                                               0, 0, 0, 1);

        cout << "Current T read from Pose.txt file: " << endl;
        cout << TWC.at<double>(0,0) << ' ' << TWC.at<double>(0,1) << ' ' << TWC.at<double>(0,2) << ' ' << TWC.at<double>(0,3) << endl;
        cout << TWC.at<double>(1,0) << ' ' << TWC.at<double>(1,1) << ' ' << TWC.at<double>(1,2) << ' ' << TWC.at<double>(1,3) << endl;
        cout << TWC.at<double>(2,0) << ' ' << TWC.at<double>(2,1) << ' ' << TWC.at<double>(2,2) << ' ' << TWC.at<double>(2,3) << endl;
        cout << TWC.at<double>(3,0) << ' ' << TWC.at<double>(3,1) << ' ' << TWC.at<double>(3,2) << ' ' << TWC.at<double>(3,3) << endl;
        cout << endl;

        TWC.inv();

        cv::Mat RWC= TWC.rowRange(0,3).colRange(0,3);  
        cv::Mat tWC= TWC.rowRange(0,3).col(3);

        tf::Matrix3x3 R(RWC.at<double>(0,0),RWC.at<double>(0,1),RWC.at<double>(0,2),
                        RWC.at<double>(1,0),RWC.at<double>(1,1),RWC.at<double>(1,2),
                        RWC.at<double>(2,0),RWC.at<double>(2,1),RWC.at<double>(2,2));
        tf::Vector3 t(tWC.at<double>(0), tWC.at<double>(1), tWC.at<double>(2));
        tf::Quaternion q_;
        R.getRotation(q_);

        static tf::TransformBroadcaster br;
        tf::Transform transform = tf::Transform(R, t);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "init_link", "camera_pose"));

        // msgs
        geometry_msgs::PoseStamped _pose;
        _pose.pose.position.x = transform.getOrigin().x();
        _pose.pose.position.y = transform.getOrigin().y();
        _pose.pose.position.z = transform.getOrigin().z();
        _pose.pose.orientation.x = transform.getRotation().x();
        _pose.pose.orientation.y = transform.getRotation().y();
        _pose.pose.orientation.z = transform.getRotation().z();
        _pose.pose.orientation.w = transform.getRotation().w();

        _pose.header.stamp = ros::Time::now();
        _pose.header.frame_id = "init_link";
        pose_pub.publish(_pose);
        
        // If you run my version of ORB-SLAM2, these two value are calculated by function Frame::getSceneDepth()
        // Here, we can only define it manually
        double min_z = 0;
        double max_z = 10;

        svo_msgs::DenseInput msg_dense;
		msg_dense.header.stamp = ros::Time::now();
		msg_dense.header.frame_id = "world";

        cv_bridge::CvImage img_msg;  
		img_msg.header.stamp=msg_dense.header.stamp;  
		img_msg.header.frame_id="camera";  
		img_msg.image= im;

		img_msg.encoding = sensor_msgs::image_encodings::BGR8;  
		msg_dense.image = *img_msg.toImageMsg();
		
		msg_dense.min_depth = min_z;
		msg_dense.max_depth = max_z;

		msg_dense.pose.position.x = tWC.at<double>(0,0);  
		msg_dense.pose.position.y = tWC.at<double>(1,0);  
		msg_dense.pose.position.z = tWC.at<double>(2,0);  
		msg_dense.pose.orientation.x = q_.x();
		msg_dense.pose.orientation.y = q_.y();
		msg_dense.pose.orientation.z = q_.z();
		msg_dense.pose.orientation.w = q_.w();
		pub_dense.publish(msg_dense); 

        ROS_INFO("ImageMsg Send.");

        ros::spinOnce();

        rate.sleep();
    }

    ros::spin();

    ros::shutdown();  // Shut down ROS.

	return 0;
}


void LoadData(const string &strImagePath, const string &strPathPoseTxt,
                vector<string> &vstrImages, vector<double> &vTimeStamps,
                vector<double> &vtx, vector<double> &vty, vector<double> &vtz,
                vector<double> &vqx, vector<double> &vqy, vector<double> &vqz, vector<double> &vqw)
{
    FILE *pFile;
    pFile = fopen (strPathPoseTxt.c_str(), "r");
    
    int index = 0;
    double timestamp;
    double PG_Tx, PG_Ty, PG_Tz;
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;

    while (fscanf(pFile, "%lf %lf %lf %lf %lf %lf %lf %lf", 
                            &timestamp, &PG_Tx, &PG_Ty, &PG_Tz,
                            &PG_Qx, &PG_Qy, &PG_Qz, &PG_Qw) != EOF)
    {
        vtx.push_back(PG_Tx);
        vty.push_back(PG_Ty);
        vtz.push_back(PG_Tz);
        vqx.push_back(PG_Qx);
        vqy.push_back(PG_Qy);
        vqz.push_back(PG_Qz);
        vqw.push_back(PG_Qw);
        
        vTimeStamps.push_back(timestamp);

        string temp = modify_img_name(index+1);
        vstrImages.push_back(strImagePath + "/IMG" + temp + ".png");

        // print and check
        cout << endl;
        cout << "IMG" + temp + ".png" << endl;
        cout << "translation t <x, y, z>: " << PG_Tx << "," << PG_Ty << "," << PG_Tz << endl;
        cout << "orienttation q <x, y, z, w>: " << PG_Qx << "," << PG_Qy << "," << PG_Qz << "," << PG_Qw << endl;

        index++;
    }

    cout << endl;
    cout << "LOAD DATA DOWN!" << endl;
}

std::string modify_img_name(int index_num) 
{   
    // So, the image should look like IMG000001, IMG000002 ...
    std::string temp;

    if (index_num < 10) 
    {
        temp = "00000" + std::to_string(index_num);
    }
    else if (index_num > 9 && index_num < 100) 
    {
        temp = "0000" + std::to_string(index_num);
    }
    else if (index_num > 99 && index_num < 1000) 
    {
        temp = "000" + std::to_string(index_num);
    }
    else if (index_num > 999 && index_num < 10000) 
    {
        temp = "00" + std::to_string(index_num);
    }
    else if (index_num > 9999 && index_num < 100000) 
    {
        temp = "0" + std::to_string(index_num);
    }

    return temp;
}