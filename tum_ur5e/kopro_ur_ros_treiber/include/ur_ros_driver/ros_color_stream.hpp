#pragma once
#include <ros/ros.h>

// Defining your own ROS stream color
#define ROS_BLACK_STREAM(x)   ROS_INFO_STREAM("\033[1;30m" << x << "\033[0m")
#define ROS_RED_STREAM(x)     ROS_INFO_STREAM("\033[1;31m" << x << "\033[0m")
#define ROS_GREEN_STREAM(x)   ROS_INFO_STREAM("\033[1;32m" << x << "\033[0m")
#define ROS_YELLOW_STREAM(x)  ROS_INFO_STREAM("\033[1;33m" << x << "\033[0m")
#define ROS_BLUE_STREAM(x)    ROS_INFO_STREAM("\033[1;34m" << x << "\033[0m")
#define ROS_MAGENTA_STREAM(x) ROS_INFO_STREAM("\033[1;35m" << x << "\033[0m")
#define ROS_CYAN_STREAM(x)    ROS_INFO_STREAM("\033[1;36m" << x << "\033[0m")

#define ROS_BLACK_STREAM_COND(c, x)   ROS_INFO_STREAM_COND(c, "\033[1;30m" << x << "\033[0m")
#define ROS_RED_STREAM_COND(c, x)     ROS_INFO_STREAM_COND(c, "\033[1;31m" << x << "\033[0m")
#define ROS_GREEN_STREAM_COND(c, x)   ROS_INFO_STREAM_COND(c, "\033[1;32m" << x << "\033[0m")
#define ROS_YELLOW_STREAM_COND(c, x)  ROS_INFO_STREAM_COND(c, "\033[1;33m" << x << "\033[0m")
#define ROS_BLUE_STREAM_COND(c, x)    ROS_INFO_STREAM_COND(c, "\033[1;34m" << x << "\033[0m")
#define ROS_MAGENTA_STREAM_COND(c, x) ROS_INFO_STREAM_COND(c, "\033[1;35m" << x << "\033[0m")
#define ROS_CYAN_STREAM_COND(c, x)    ROS_INFO_STREAM_COND(c, "\033[1;36m" << x << "\033[0m")

#define ROS_BLACK_STREAM_THROTTLE(c, x)   ROS_INFO_STREAM_THROTTLE(c, "\033[1;30m" << x << "\033[0m")
#define ROS_RED_STREAM_THROTTLE(c, x)     ROS_INFO_STREAM_THROTTLE(c, "\033[1;31m" << x << "\033[0m")
#define ROS_GREEN_STREAM_THROTTLE(c, x)   ROS_INFO_STREAM_THROTTLE(c, "\033[1;32m" << x << "\033[0m")
#define ROS_YELLOW_STREAM_THROTTLE(c, x)  ROS_INFO_STREAM_THROTTLE(c, "\033[1;33m" << x << "\033[0m")
#define ROS_BLUE_STREAM_THROTTLE(c, x)    ROS_INFO_STREAM_THROTTLE(c, "\033[1;34m" << x << "\033[0m")
#define ROS_MAGENTA_STREAM_THROTTLE(c, x) ROS_INFO_STREAM_THROTTLE(c, "\033[1;35m" << x << "\033[0m")
#define ROS_CYAN_STREAM_THROTTLE(c, x)    ROS_INFO_STREAM_THROTTLE(c, "\033[1;36m" << x << "\033[0m")
