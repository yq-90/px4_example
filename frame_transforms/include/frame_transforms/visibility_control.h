#ifndef FRAME_TRANSFORMS__VISIBILITY_CONTROL_H_
#define FRAME_TRANSFORMS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FRAME_TRANSFORMS_EXPORT __attribute__ ((dllexport))
    #define FRAME_TRANSFORMS_IMPORT __attribute__ ((dllimport))
  #else
    #define FRAME_TRANSFORMS_EXPORT __declspec(dllexport)
    #define FRAME_TRANSFORMS_IMPORT __declspec(dllimport)
  #endif
  #ifdef FRAME_TRANSFORMS_BUILDING_LIBRARY
    #define FRAME_TRANSFORMS_PUBLIC FRAME_TRANSFORMS_EXPORT
  #else
    #define FRAME_TRANSFORMS_PUBLIC FRAME_TRANSFORMS_IMPORT
  #endif
  #define FRAME_TRANSFORMS_PUBLIC_TYPE FRAME_TRANSFORMS_PUBLIC
  #define FRAME_TRANSFORMS_LOCAL
#else
  #define FRAME_TRANSFORMS_EXPORT __attribute__ ((visibility("default")))
  #define FRAME_TRANSFORMS_IMPORT
  #if __GNUC__ >= 4
    #define FRAME_TRANSFORMS_PUBLIC __attribute__ ((visibility("default")))
    #define FRAME_TRANSFORMS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FRAME_TRANSFORMS_PUBLIC
    #define FRAME_TRANSFORMS_LOCAL
  #endif
  #define FRAME_TRANSFORMS_PUBLIC_TYPE
#endif

#endif  // FRAME_TRANSFORMS__VISIBILITY_CONTROL_H_
