/*   
* ---  Pointcloud Registration NODE  ---
* Created by: Tan You Liang (Nov 2018, Hope Technik Project)
* 
* Handle Pointcloud Summation for 5 initial frames based on odometry prior to ICP
* Before that, making use of IMU to elimate roll and pitch of robot (noise on flat surfaces)
* Odometry source: Encoder
* The node here also manage the tf transformation and publishing of summed pointcloud to icpMatcher
*/



#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// sensor msg
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

// icp testing
#include <pcl/registration/icp.h>


//** ====== TENTATIVE PARAM =========
extern float turningRegTresh = 0.3;   // max allowed angular vel tresh for turning robot to register pointcloud



// // =============================== Main IMU and PCD Transform Class ===========================================

// transfrom IMU according to imu/rpy topic
class PointCloudRegistration
{
  private:

    ros::Publisher _pub_cloudSummation; ///< (low frequency) filtered transformed cloud publisher
    // ros::Subscriber _sub_IMUrpy;          ///< (high frequency) imu vector3 subscriber
    // ros::Subscriber _sub_IMUmsg;          ///< (high frequency) imu msg subscriber
    ros::Subscriber _sub_encoderOdom;     ///< (high frequency) robot encoder msg subscriber
    ros::Subscriber _sub_pointCloud;   ///< (low frequency) velodyne raw cloud subscriber
    
    geometry_msgs::Vector3Stamped imu_rpy;
    // sensor_msgs::Imu imu_msg;             // to get imu_msg, and change its orientation
    // int is_encoderOdom_init;              // 1 if first odom value

    pcl::PointCloud<pcl::PointXYZ> cloudPrevFrame;    // Prev frame pointcloud
    tf::Transform tf_odom;
    float base_link_angVel;        // Base_link yaw angle turning angular velocity
    int cloud_count;               // track number of cloud being summed

    ros::Time odomTimeStmp;       // timestamp from encoder odom

  public:

    //setup pub and sub
    bool setup(ros::NodeHandle& node)
    {
      
      // --- Publish static orientation pointcloud with transformed input pcd based of IMUrpy.orientation
      _pub_cloudSummation =  node.advertise<sensor_msgs::PointCloud2> ("/cloud_summation", 1);

      // --- Source cloud from velodyne 
      _sub_pointCloud = node.subscribe<sensor_msgs::PointCloud2::Ptr>("/velodyne_points", 2, &PointCloudRegistration::cloudSummation_callback, this);
      
      // --- IMU rpy and rpy standard msg from IMU sensor
      // _sub_IMUrpy = node.subscribe<geometry_msgs::Vector3Stamped>("/imu/rpy", 10, &OdomHandler::imu_rpyCallback, this);
      // _sub_IMUmsg = node.subscribe<sensor_msgs::Imu>("/imu/imu", 10, &OdomHandler::imu_msgCallback, this);
      
      // --- Encoder odom msg from robot
      _sub_encoderOdom = node.subscribe<nav_msgs::Odometry::Ptr>("/odom", 10, &PointCloudRegistration::encoderOdom_Callback, this);

      cloud_count = 1;

      ROS_INFO("Done with Setup of Sub and Pub for node");
      return true;
    }



    // // sub to /imu/imu to get retrieve all info of sensor
    // void imu_msgCallback(const sensor_msgs::Imu::ConstPtr& imuIn){
    //   imu_msg = *imuIn;
    // }



    // // sub to /imu/ geometry_msgs/Vector3Stamped
    // void imu_rpyCallback(const geometry_msgs::Vector3Stamped::ConstPtr& imuIn){

    //   // ----------------------- IMU filtered --------------

    //   // // Managing TF publisher
    //   static tf::TransformBroadcaster br;
    //   tf::Transform transform;
    //   transform.setOrigin( tf::Vector3(0, 0, 0.0) );
    //   imu_rpy = *imuIn;
      
    //   // set value tf publisher
    //   tf::Quaternion q;
    //   q.setRPY(imu_rpy.vector.x, -imu_rpy.vector.y, -imu_rpy.vector.z); //rpy to quaternion
    //   transform.setRotation(q);
    //   // br.sendTransform(tf::StampedTransform(transform, imu_rpy.header.stamp, "odom", "base_link")); //use imu orientation as odom

    //   // // Managing IMU Publisher
    //   sensor_msgs::Imu imuOut;  // set value to new imu publisher
      
    //   imuOut.header = imu_rpy.header;

    //   imuOut.orientation.x = q.x();
    //   imuOut.orientation.y = q.y();
    //   imuOut.orientation.z = q.z();
    //   imuOut.orientation.w = q.w();

    //   imuOut.angular_velocity = imu_msg.angular_velocity;
    //   imuOut.linear_acceleration = imu_msg.linear_acceleration;
      
    //   ROS_INFO("publishing imu_filtered out");

    //   _pub_IMUfiltered.publish(imuOut);


    // }



    // transform pcd to imu rpy frame
    void cloudSummation_callback(const sensor_msgs::PointCloud2::Ptr _cloud){

      pcl::PointCloud<pcl::PointXYZ>::Ptr velo_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::fromROSMsg( *_cloud, *velo_cloud);

      Eigen::Affine3f transform = Eigen::Affine3f::Identity(); 


      ROS_INFO("Robot Turning Speed: %lf", base_link_angVel);  
      ROS_INFO("odomTime: %i cloudTime: %i", odomTimeStmp.nsec, _cloud->header.stamp.nsec);  
      
      float yaw_timeCompensation = transformPredictionOnTimeDiff(odomTimeStmp.nsec, _cloud->header.stamp.nsec);

      // ==================   Transformation =====================

      double x, y, z, roll, pitch, yaw;
      tf::Matrix3x3( tf_odom.getRotation() ).getRPY(roll, pitch, yaw);
      tf::Vector3 vec = tf_odom.getOrigin();

      // Transformation
      transform.translation() << vec.x(), vec.y(), vec.z();

      // The same rotation matrix as before; theta radians around Z axis
      transform.rotate (Eigen::AngleAxisf (roll, Eigen::Vector3f::UnitX()));
      transform.rotate (Eigen::AngleAxisf (pitch, Eigen::Vector3f::UnitY()));
      transform.rotate (Eigen::AngleAxisf (yaw + yaw_timeCompensation, Eigen::Vector3f::UnitZ()));

      // // Executing the transformation
      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::transformPointCloud (*velo_cloud, *transformed_cloud, transform);
      cloudPrevFrame =  *transformed_cloud;


      // ==================== end Transformation =====================




      if ( fabs(base_link_angVel) <= turningRegTresh){
        // only output =) TBC

        // Output msg
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*transformed_cloud, msg);
        msg.header.stamp = _cloud->header.stamp;
        msg.header.frame_id = _cloud->header.frame_id;
        _pub_cloudSummation.publish(msg);

      }

    }
  


    // Publish encoder odom msg to TF (camera_init to encoderOdom)
    void encoderOdom_Callback(const nav_msgs::Odometry::Ptr _encoderOdom){

      // // Managing TF publisher
      static tf::TransformBroadcaster br;

      float x, y, z, w;
      x = _encoderOdom->pose.pose.position.x;
      y = _encoderOdom->pose.pose.position.y;
      z = _encoderOdom->pose.pose.position.z;
      tf::Vector3 vec(x,y,z);

      x = _encoderOdom->pose.pose.orientation.x;
      y = _encoderOdom->pose.pose.orientation.y;
      z = _encoderOdom->pose.pose.orientation.z;
      w = _encoderOdom->pose.pose.orientation.w;
      tf::Quaternion quat(x,y,z,w);

      base_link_angVel = _encoderOdom->twist.twist.angular.z; // yaw angular vel
      odomTimeStmp = _encoderOdom->header.stamp;

      tf_odom.setOrigin(vec);
      tf_odom.setRotation(quat);

      br.sendTransform(tf::StampedTransform(tf_odom, _encoderOdom->header.stamp, "odom", "base_link")); //use imu orientation as odom

    }

    // ** PROTECTED
    // Compensate rotation diff based of diff in time stamp
    float transformPredictionOnTimeDiff(int t1, int t2){
      
      int time_diff = t2 - t1;
      float rotate_compensation = base_link_angVel*float(time_diff)/1000000000;

      ROS_INFO("- yaw compensation: %f", rotate_compensation);  

      return rotate_compensation;
    }


};

// // ==================================== End Class ===========================================





// // ======================================= Main ================================================

int main(int argc, char** argv){

  ros::init(argc, argv, "registration_node");
  ros::NodeHandle node;

  // Get Arguments
  int path = 0; // default
  if (argc > 1) {
    path = std::atoi(argv[1]);
    std::cout << "Pcd path is: " << path << std::endl;
  }

  ROS_INFO("Using msg geometry_msgs/Vector3Stamped!!");

  PointCloudRegistration registration;
  if (registration.setup(node)) {
    // successful initialization 
    ros::spin();
  }   

  return 0;
};
