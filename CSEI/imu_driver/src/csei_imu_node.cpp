#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "adis16470.h"

class ImuNode {
    public:
        Adis16470 imu; 
        ros::NodeHandle node_handle_;
        ros::Publisher imu_data_pub_;
        std::string device_;
        std::string frame_id_;
        double rate_;

    explicit ImuNode(ros::NodeHandle nh)
        : node_handle_(nh)
        {
            node_handle_.param("device", device_, std::string("/dev/i2c-1"));
            node_handle_.param("frame_id", frame_id_, std::string("imu"));
            node_handle_.param("rate", rate_, 100.0);

            ROS_INFO("device: %s", device_.c_str());
            ROS_INFO("frame_id: %s", frame_id_.c_str());
            ROS_INFO("rate: %f [Hz]", rate_);
        

        // Data publisher
        imu_data_pub_ = node_handle_.advertise<sensor_msgs::Imu>("imu", 100);
        }

    ~ImuNode()
    {
        imu.closePort();
    }

    bool is_opened(void)
    {
        return (imu.fd_>0);
    }

    bool open(void)
    {
        if (imu.openPort(device_) < 0)
        {
            ROS_ERROR("Failed to open device %s", device_.c_str());
        }
        
        //  Wait 
        return true;
    }

    int publish_imu_data()
    {
    sensor_msgs::Imu data;
    data.header.frame_id = frame_id_;
    data.header.stamp = ros::Time::now();

    // Linear acceleration
    data.linear_acceleration.x = imu.accl[0];
    data.linear_acceleration.y = imu.accl[1];
    data.linear_acceleration.z = imu.accl[2];

    // Angular velocity
    data.angular_velocity.x = imu.gyro[0];
    data.angular_velocity.y = imu.gyro[1];
    data.angular_velocity.z = imu.gyro[2];

    // Orientation (not provided)
    data.orientation.x = 0;
    data.orientation.y = 0;
    data.orientation.z = 0;
    data.orientation.w = 1;

    imu_data_pub_.publish(data);
    return 0;
    }

    bool spin()
    {

        ros::Rate loop_rate(rate_);
        while (ros::ok())
        {
            if (imu.update() == 0)
            {
                publish_imu_data();
            }
            else 
            {
                ROS_ERROR("Cannot update!");
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    return true;   
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu");
    ros::NodeHandle nh;
    ImuNode node(nh);

    node.open();
    while (!node.is_opened())
    {
        ROS_WARN("Keep trying to open the device in 1 second period...");
        sleep(1);
        node.open();
    }
    node.spin();
    return(0);
}
   