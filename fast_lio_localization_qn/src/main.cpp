#include "main.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fast_lio_localization_qn_node");
  ros::NodeHandle nh_private("~");

  FAST_LIO_LOCALIZATION_QN_CLASS fast_lio_localization_qn_(nh_private);

  ros::AsyncSpinner spinner(4); // Use multi threads
  spinner.start();
  ros::waitForShutdown();
 
  return 0;
}