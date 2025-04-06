#include <vector>
#include <string>
#include <functional>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>


const double negative_obs_default_thres = 1.80;
double negative_obs_thres = 0.0;
std::vector<std::string> sensor_topics =
{
    "/wheel_drop_sensor/front_centre", "/wheel_drop_sensor/rear_centre",
    "/wheel_drop_sensor/front_left", "/wheel_drop_sensor/rear_left",
    "/wheel_drop_sensor/front_right", "/wheel_drop_sensor/rear_right"
};
std::vector<std::string> output_topics =
{
    "/wheel_drop_sensor/proccessed/front_centre", "/wheel_drop_sensor/proccessed/rear_centre",
    "/wheel_drop_sensor/proccessed/front_left", "/wheel_drop_sensor/proccessed/rear_left",
    "/wheel_drop_sensor/proccessed/front_right", "/wheel_drop_sensor/proccessed/rear_right"
};
std::vector<ros::Publisher> negative_obs_pub;

void range_sensor_callback(const sensor_msgs::Range::ConstPtr& msg, int sensor_index)
{
    ROS_INFO("wheel drop_sensing %d", sensor_index);
    sensor_msgs::Range negative_obs_range;

    negative_obs_range.header.stamp = msg->header.stamp;//ros::Time::now();
    negative_obs_range.header.frame_id = msg->header.frame_id;
    negative_obs_range.radiation_type = msg->radiation_type;
    negative_obs_range.field_of_view = msg->field_of_view;
    negative_obs_range.min_range = msg->min_range;
    negative_obs_range.max_range = msg->max_range;

    if (msg->range > negative_obs_default_thres)
    {
        negative_obs_range.range = msg->min_range;
    }
    else
    {
        negative_obs_range.range = msg->max_range;
    }

    negative_obs_pub[sensor_index].publish(negative_obs_range);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheel_drop_sensing_node");
    ros::NodeHandle nh;
    std::vector<ros::Subscriber> sensor_sub;

    // while (!ros::ok());

    // if (!nh.getParam("negative_obs_mark_threshold", negative_obs_thres))
    // {
    //     ROS_INFO("negative_obs_mark_threshold parameter is not defined, using default value %.2f", 
    //              negative_obs_default_thres);
    // }

    // if (!nh.getParam("sensor_topics", sensor_topics))
    // {
    //     ROS_ERROR("sensor_topics for wheel_drop_sensing is not defined");
    //     // return -1;
    // }

    // if (!nh.getParam("output_topics", output_topics))
    // {
    //     ROS_ERROR("output_topics for wheel_drop_sensing is not defined");
    //     // return -1;
    // }

    if (output_topics.size() != sensor_topics.size())
    {
        ROS_ERROR("number of sensor_topics must be same as number of output_topics");
        // return -1;
    }

    for (int i = 0; i < sensor_topics.size(); ++i)
    {
        auto callback = [i](const sensor_msgs::Range::ConstPtr& msg) {range_sensor_callback(msg, i);};
        sensor_sub.push_back(nh.subscribe<sensor_msgs::Range>(sensor_topics[i], 1, callback));

        negative_obs_pub.push_back(nh.advertise<sensor_msgs::Range>(output_topics[i], 1));
    }

    ROS_INFO("wheel_drop_sensing running...");

    ros::spin();
}
