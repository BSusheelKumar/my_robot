

#ifndef MY_ROBOT__VISIBILITY_CONTROL_H_
#define MY_ROBOT__VISIBILITY_CONTROL_H_



#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MY_ROBOT_EXPORT __attribute__((dllexport))
#define MY_ROBOT_IMPORT __attribute__((dllimport))
#else
#define MY_ROBOT_EXPORT __declspec(dllexport)
#define MY_ROBOT_IMPORT __declspec(dllimport)
#endif
#ifdef MY_ROBOT_BUILDING_DLL
#define MY_ROBOT_PUBLIC MY_ROBOT_EXPORT
#else
#define MY_ROBOT_PUBLIC MY_ROBOT_IMPORT
#endif
#define MY_ROBOT_PUBLIC_TYPE MY_ROBOT_PUBLIC
#define MY_ROBOT_LOCAL
#else
#define MY_ROBOT_EXPORT __attribute__((visibility("default")))
#define MY_ROBOT_IMPORT
#if __GNUC__ >= 4
#define MY_ROBOT_PUBLIC __attribute__((visibility("default")))
#define MY_ROBOT_LOCAL __attribute__((visibility("hidden")))
#else
#define MY_ROBOT_PUBLIC
#define MY_ROBOT_LOCAL
#endif
#define MY_ROBOT_PUBLIC_TYPE
#endif

#endif  // MY_ROBOT__VISIBILITY_CONTROL_H_
