#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace sensor_msgs;
using namespace livox_ros_driver2;
using namespace message_filters;

class SyncNode {
public:
    SyncNode(ros::NodeHandle& nh) : sync(MySyncPolicy(10), image_sub, lidar_sub) {
        // Get parameters from the parameter server
        std::string image_topic, lidar_topic, synced_image_topic, synced_lidar_topic;
        double publish_rate;

        nh.getParam("image_topic", image_topic);
        nh.getParam("lidar_topic", lidar_topic);
        nh.getParam("synced_image_topic", synced_image_topic);
        nh.getParam("synced_lidar_topic", synced_lidar_topic);
        nh.getParam("publish_rate", publish_rate);

        image_sub.subscribe(nh, image_topic, 10);
        lidar_sub.subscribe(nh, lidar_topic, 10);

        image_pub = nh.advertise<Image>(synced_image_topic, 10);
        lidar_pub = nh.advertise<CustomMsg>(synced_lidar_topic, 10);

        sync.registerCallback(boost::bind(&SyncNode::callback, this, _1, _2));

        // Set up a timer to publish synchronized messages at the specified rate
        ros::Duration period(1.0 / publish_rate); // Convert Hz to seconds
        timer = nh.createTimer(period, &SyncNode::timerCallback, this);
    }

private:
    ros::NodeHandle nh;
    Subscriber<Image> image_sub;
    Subscriber<CustomMsg> lidar_sub;
    typedef sync_policies::ApproximateTime<Image, CustomMsg> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync;
    ros::Publisher image_pub;
    ros::Publisher lidar_pub;
    ros::Timer timer;

    Image last_image_msg;
    CustomMsg last_lidar_msg;
    bool new_image_received = false;
    bool new_lidar_received = false;

    void callback(const ImageConstPtr& img_msg, const CustomMsgConstPtr& lidar_msg) {
        // Store the latest synchronized messages
        last_image_msg = *img_msg;
        last_lidar_msg = *lidar_msg;
        new_image_received = true;
        new_lidar_received = true;
    }

    void timerCallback(const ros::TimerEvent& event) {
        if (new_image_received && new_lidar_received) {
            image_pub.publish(last_image_msg);
            lidar_pub.publish(last_lidar_msg);
            new_image_received = false;
            new_lidar_received = false;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sync_node");
    ros::NodeHandle nh("~");

    SyncNode node(nh);

    ros::spin();

    return 0;
}



