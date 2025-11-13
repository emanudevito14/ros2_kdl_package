#ifndef ROS2_KDL_PACKAGE__VISIBILITY_CONTROL_H_
#define ROS2_KDL_PACKAGE__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROS2_KDL_PACKAGE_EXPORT __attribute__ ((dllexport))
    #define ROS2_KDL_PACKAGE_IMPORT __attribute__ ((dllimport))
  #else
    #define ROS2_KDL_PACKAGE_EXPORT __declspec(dllexport)
    #define ROS2_KDL_PACKAGE_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROS2_KDL_PACKAGE_BUILDING_DLL
    #define ROS2_KDL_PACKAGE_PUBLIC ROS2_KDL_PACKAGE_EXPORT
  #else
    #define ROS2_KDL_PACKAGE_PUBLIC ROS2_KDL_PACKAGE_IMPORT
  #endif
  #define ROS2_KDL_PACKAGE_PUBLIC_TYPE ROS2_KDL_PACKAGE_PUBLIC
  #define ROS2_KDL_PACKAGE_LOCAL
#else
  #define ROS2_KDL_PACKAGE_EXPORT __attribute__ ((visibility("default")))
  #define ROS2_KDL_PACKAGE_IMPORT
  #if __GNUC__ >= 4
    #define ROS2_KDL_PACKAGE_PUBLIC __attribute__ ((visibility("default")))
    #define ROS2_KDL_PACKAGE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROS2_KDL_PACKAGE_PUBLIC
    #define ROS2_KDL_PACKAGE_LOCAL
  #endif
  #define ROS2_KDL_PACKAGE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROS2_KDL_PACKAGE__VISIBILITY_CONTROL_H_