#ifndef PX4_TOPIC__VISIBILITY_CONTROL_H_
#define PX4_TOPIC__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PX4_TOPIC_EXPORT __attribute__ ((dllexport))
    #define PX4_TOPIC_IMPORT __attribute__ ((dllimport))
  #else
    #define PX4_TOPIC_EXPORT __declspec(dllexport)
    #define PX4_TOPIC_IMPORT __declspec(dllimport)
  #endif
  #ifdef PX4_TOPIC_BUILDING_LIBRARY
    #define PX4_TOPIC_PUBLIC PX4_TOPIC_EXPORT
  #else
    #define PX4_TOPIC_PUBLIC PX4_TOPIC_IMPORT
  #endif
  #define PX4_TOPIC_PUBLIC_TYPE PX4_TOPIC_PUBLIC
  #define PX4_TOPIC_LOCAL
#else
  #define PX4_TOPIC_EXPORT __attribute__ ((visibility("default")))
  #define PX4_TOPIC_IMPORT
  #if __GNUC__ >= 4
    #define PX4_TOPIC_PUBLIC __attribute__ ((visibility("default")))
    #define PX4_TOPIC_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PX4_TOPIC_PUBLIC
    #define PX4_TOPIC_LOCAL
  #endif
  #define PX4_TOPIC_PUBLIC_TYPE
#endif

#endif  // PX4_TOPIC__VISIBILITY_CONTROL_H_
