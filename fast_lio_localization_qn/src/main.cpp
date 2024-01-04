#include "main.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fast_lio_localization_qn_node");
  ros::NodeHandle nh_private("~");

  FastLioLocalizationQnClass FastLioLocalizationQn_(nh_private);

  ros::AsyncSpinner spinner(3); // Use multi threads
  spinner.start();
  ros::waitForShutdown();
 
  return 0;
}