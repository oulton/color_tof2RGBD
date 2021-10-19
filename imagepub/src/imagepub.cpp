#include<ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

#include<stdio.h>
#include<math.h>
#include<vector>

ros::Publisher pubImage, pubDepth;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::PointCloud2> sync_policy_classification;

cv::Mat CreatDepthFromCloud( int width, int height,  Eigen::Matrix3f intrinsic, pcl::PointCloud<pcl::PointXYZ> cloud ){

	std::vector<int> indices;
    pcl::removeNaNFromPointCloud( cloud, cloud, indices);  //remove无效点

    cv::Mat depth=cv::Mat::zeros( height, width, CV_16UC1 );
    for(int i=0;i<cloud.points.size(); i++ ){

        Eigen::Vector3f v( cloud.points[i].x,  cloud.points[i].y , cloud.points[i].z);
        // std::cout<<v<<std::endl;
        if(v[0]==0&&v[1]==0&&v[2]==0)
            continue;;
        Eigen::Vector3f uv=intrinsic*v;
        int u_=round( uv[0]/uv[2] );
        int v_=round( uv[1]/uv[2] );
        int d= round( uv[2]*1000 );
        if(u_>=width||u_<0||v_>=height||v_<0)
            continue;
        depth.ptr<ushort>(v_)[u_]=d;
    }
    return depth;
}

Eigen::Vector3f cam2point( int m, int n, Eigen::Matrix3f  K_x_ )
{
    return Eigen::Vector3f(
            // ( n - cx ) / fx ,
            // ( m - cy ) / fy ,
            //         1
            ( n - K_x_(0,2) ) / K_x_(0,0) ,
            ( m - K_x_(1,2) ) / K_x_(1,1) ,
                    1
            );
}

cv::Mat CreateColorToDepth(cv::Mat color, cv::Mat depth, Eigen::Matrix3f intrinsic_color, Eigen::Matrix3f intrinsic_depth, Eigen::Matrix4f  T_depth_color)
{
	cv::Mat color_out = cv::Mat::zeros( depth.rows, depth.cols, CV_8UC3 );
	for(int m=0;m<depth.rows;m++)
    {
        for(int n=0;n<depth.cols;n++)
        {
            double d=depth.ptr<ushort>(m)[n];
            if(d<=0 || d>5000)
                continue;
            Eigen::Vector3f depthFramPoint = cam2point( m, n, intrinsic_depth );
            depthFramPoint = depthFramPoint * ( d / 1000.0 );

            int u_, v_;
            //color对齐到depth  
            Eigen::Vector3f colorPixPoint = intrinsic_color * (T_depth_color.block(0,0,3,3) * depthFramPoint + T_depth_color.block(0,3,3,1));
            u_=colorPixPoint[0]/colorPixPoint[2];
            v_=colorPixPoint[1]/colorPixPoint[2];
            if( u_<0||u_>=color.cols || v_<0||v_>=color.rows ){}
            else
            {
                color_out.ptr<uchar>(m)[n*3+1] = color.ptr<uchar>(v_)[u_*3+1];  //b
                color_out.ptr<uchar>(m)[n*3+2] = color.ptr<uchar>(v_)[u_*3+2];  //g
                color_out.ptr<uchar>(m)[n*3+3] = color.ptr<uchar>(v_)[u_*3+3];  //r
            }
		}
	}
	return color_out;
}

void callback(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	static int count = 1;
	ROS_INFO("Received count=%d \n", count++);

	// sensor_msgs::Image  --->  cv::Mat
	cv::Mat color_image_raw = cv_bridge::toCvShare(msg, "bgr8")->image; 

	// sensor_msgs::PointCloud2 ---> pcl::PointCloud<T>
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(*cloud_msg, cloud);    


	//color   depth  内外参
	Eigen::Matrix3f intrinsic_depth;
    intrinsic_depth  <<  508.1586608886719,0,322.898193359375,
                         0,508.1586608886719,250.3104248046875,
                         0,0,1;
	Eigen::Matrix3f intrinsic_color;
    intrinsic_color  <<  2187.838429, 0, 1050.0,
						 0, 2187.838429, 1005.266588,
						 0, 0, 1;
	Eigen::Matrix4f T_color_depth;  //此处是color2depth     下面代码传参地方需要depth2color   所以要转置一下
    T_color_depth  <<    0.9980969724486808, -0.0019868338822574493, -0.061631859292932197, -0.37329782506326975,
						 0.0024830583728700562, 0.9999651092489452, 0.00797588276393677, 0.007630703788648211,
						 0.061613862156956196, -0.00811373994354376, 0.9980670815201924, -0.026399685129054495,
						 0., 0., 0., 1.0;


	// cloud ---> depth   AND   color2depth
	cv::Mat depth_image = CreatDepthFromCloud( 640 , 480, intrinsic_depth, cloud );
	cv::Mat color_image = CreateColorToDepth( color_image_raw, depth_image, intrinsic_color, intrinsic_depth, T_color_depth.inverse() );


	// cv::Mat  --->  sensor_msgs::Image
	sensor_msgs::ImagePtr pubimg_col = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_image).toImageMsg();
	sensor_msgs::ImagePtr pubimg_dep = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_image).toImageMsg();
	pubimg_col->header = msg->header;
	pubimg_dep->header = msg->header;
	
	// publish color_img and depth_img
	pubImage.publish(pubimg_col);
	pubDepth.publish(pubimg_dep);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_convert");
	ros::NodeHandle nh;

	pubImage = nh.advertise<sensor_msgs::Image>("/image_color_convert", 1000, true);
	pubDepth = nh.advertise<sensor_msgs::Image>("/image_depth_convert", 1000, true);

	message_filters::Subscriber<sensor_msgs::Image> info_sub(nh, "/Preposition_NavigationCamera_left", 1000);
	message_filters::Subscriber<sensor_msgs::PointCloud2> pose_sub(nh, "/up_tof",1000);
    message_filters::Synchronizer<sync_policy_classification> sync(sync_policy_classification(1000), info_sub, pose_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

	ros::spin();
	return 0;
}
