#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "tf/transform_listener.h"
#include "filters/filter_chain.h"

class GenericLaserScanFilterNode
{
protected:
  // Our NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;


  // Components for tf::MessageFilter
  tf::TransformListener tf_;
  message_filters::Subscriber<sensor_msgs::LaserScan> *scan_sub_;


  // Components for publishing
  sensor_msgs::LaserScan msg_;
  ros::Publisher output_pub_;

  ros::Timer deprecation_timer_;

  double min_th_;

  std::string sub_topic_;
  std::string pub_topic_;


public:
  // Constructor
  GenericLaserScanFilterNode():
    private_nh_("~"),
    min_th_(0.6), 
    sub_topic_("scan"), 
    pub_topic_("scan_filtered")
  {
    private_nh_.getParam("min_th", min_th_);
    private_nh_.getParam("laser_topic_in", sub_topic_);
    private_nh_.getParam("laser_topic_out", pub_topic_);
    
    scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, sub_topic_, 50);
    scan_sub_->registerCallback(boost::bind(&GenericLaserScanFilterNode::callback, this, _1));
    
    // Advertise output
    output_pub_ = nh_.advertise<sensor_msgs::LaserScan>(pub_topic_, 100);
  }
  
  // Callback
  void callback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
  {
    // Copy the incoming laser scan
    msg_ = *msg_in;

    // Iterate over the range readings and remove single readings
    for (unsigned int i = 1; i < msg_.ranges.size()-1; i++)
    {
	// if previous and next value are nan or max range assing max range to the current range value
        if( (isnan(msg_.ranges[i-1]) || (msg_.ranges[i-1] == msg_.range_max))  &&
            (isnan(msg_.ranges[i+1]) || (msg_.ranges[i+1] == msg_.range_max)))
	{
	    msg_.ranges[i] = msg_.range_max;
	}

    }

    // Publish the output
    output_pub_.publish(msg_);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "single_readings_filter_node");
  
  GenericLaserScanFilterNode t;
  ros::spin();
  
  return 0;
}



  



