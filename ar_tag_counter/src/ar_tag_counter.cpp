#include "ros/ros.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "ar_track_alvar_msgs/AlvarMarker.h"
#include "std_msgs/Int32.h"
#include "std_srvs/Empty.h"

#define NAME_OF_THIS_NODE "ar_tag_counter"

class ROSnode {
private: 
    ros::NodeHandle Handle;
    ros::Subscriber tagSub;
    ros::Publisher countPub;
    ros::Timer resetTimer;
    ros::ServiceServer resetService;
    int arTagNumber, totalTags;
    bool withTimer;
    double resetTime;
    std::vector<int> spottedTags;
    void tagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void resetTimerCallback(const ros::TimerEvent&);
    bool resetServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
public:    
    bool Prepare();
    void RunPeriodically();
};

bool ROSnode::Prepare() {
    Handle.param("/ar_tag_number", arTagNumber, 9);
    Handle.param("/use_timer", withTimer, true);
    Handle.param("/reset_time", resetTime, 120.0);
    spottedTags = std::vector<int>(arTagNumber, 0);
    totalTags = 0;
    tagSub = Handle.subscribe("ar_pose_marker", 10, &ROSnode::tagCallback, this);
    countPub = Handle.advertise<std_msgs::Int32>("/tag_count", 10);
    resetService = Handle.advertiseService("/reset", &ROSnode::resetServiceCallback, this);
    if(withTimer)
        resetTimer = Handle.createTimer(ros::Duration(resetTime), &ROSnode::resetTimerCallback, this);
    return true;
}

void ROSnode::resetTimerCallback(const ros::TimerEvent&) {
    ROS_INFO("Resetting tag count via timer");
    spottedTags.assign(arTagNumber, 0);
}

bool ROSnode::resetServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    ROS_INFO("Resetting tag count via service");
    spottedTags.assign(arTagNumber, 0);
    return true;
}

void ROSnode::tagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
    if(!msg->markers.empty()) {
        for (auto it = msg->markers.begin(); it != msg->markers.end(); it++) {
            ar_track_alvar_msgs::AlvarMarker mk = (ar_track_alvar_msgs::AlvarMarker) *it;
            spottedTags[mk.id] = 1;
        }
        totalTags = std::accumulate(spottedTags.begin(), spottedTags.end(), 0);
    }
    
}

void ROSnode::RunPeriodically() {
    ROS_INFO("Node %s running periodically.", ros::this_node::getName().c_str());
    
    ros::Rate r(10); //10 hz
    while (ros::ok()) {
        std_msgs::Int32 msg;
        msg.data = totalTags;
        countPub.publish(msg);
        ros::spinOnce();
        r.sleep();
    }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  ROSnode mNode;
   
  if(!mNode.Prepare())
      return -1;
  mNode.RunPeriodically();
    
  return 0;
}

