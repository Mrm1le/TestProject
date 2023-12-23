#pragma once

#define PARAM_HELPER_STRINGIFY(x) #x

#define PARAM_HELPER_TOSTRING(x) PARAM_HELPER_STRINGIFY(x)

#define READ_PARAM_BEGIN ros::NodeHandle ___nh_param_reader("~");

#define READ_PARAM_END ;

#define READ_STRING_PARAM_WITH_DEFAULT(x, default_value)                       \
  std::string x;                                                               \
  ___nh_param_reader.param<std::string>(PARAM_HELPER_TOSTRING(x), x,           \
                                        default_value);                        \
  ROS_INFO(PARAM_HELPER_TOSTRING(x) ": %s", x.c_str());

#define READ_XML_PARAM_WITH_DEFAULT(x, default_value)                          \
  XmlRpc::XmlRpcValue x;                                                       \
  ___nh_param_reader.param<XmlRpc::XmlRpcValue>(PARAM_HELPER_TOSTRING(x), x,   \
                                                default_value);

#define READ_INT_PARAM_WITH_DEFAULT(x, default_value)                          \
  int x;                                                                       \
  ___nh_param_reader.param<int>(PARAM_HELPER_TOSTRING(x), x, default_value);   \
  ROS_INFO(PARAM_HELPER_TOSTRING(x) ": %d", x);

#define READ_GLOBAL_BOOL_PARAM_WITH_DEFAULT(x, default_value)                  \
  bool x;                                                                      \
  ___nh_param_reader.param<bool>(std::string("/") + PARAM_HELPER_TOSTRING(x),  \
                                 x, default_value);                            \
  ROS_INFO(PARAM_HELPER_TOSTRING(x) ": %d", x);
