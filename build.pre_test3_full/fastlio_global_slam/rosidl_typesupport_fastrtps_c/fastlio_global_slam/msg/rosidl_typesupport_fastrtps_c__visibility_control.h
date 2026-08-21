// generated from
// rosidl_typesupport_fastrtps_c/resource/rosidl_typesupport_fastrtps_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef FASTLIO_GLOBAL_SLAM__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_
#define FASTLIO_GLOBAL_SLAM__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_

#if __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_fastlio_global_slam __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_fastlio_global_slam __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_fastlio_global_slam __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_fastlio_global_slam __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_FASTRTPS_C_BUILDING_DLL_fastlio_global_slam
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fastlio_global_slam ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_fastlio_global_slam
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fastlio_global_slam ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_fastlio_global_slam
  #endif
#else
  #define ROSIDL_TYPESUPPORT_FASTRTPS_C_EXPORT_fastlio_global_slam __attribute__ ((visibility("default")))
  #define ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_fastlio_global_slam
  #if __GNUC__ >= 4
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fastlio_global_slam __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_fastlio_global_slam
  #endif
#endif

#if __cplusplus
}
#endif

#endif  // FASTLIO_GLOBAL_SLAM__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_C__VISIBILITY_CONTROL_H_
