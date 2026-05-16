#ifndef IANVS__VISIBILITY_CONTROL_H_
#define IANVS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define IANVS_EXPORT __attribute__((dllexport))
#define IANVS_IMPORT __attribute__((dllimport))
#else
#define IANVS_EXPORT __declspec(dllexport)
#define IANVS_IMPORT __declspec(dllimport)
#endif
#ifdef IANVS_BUILDING_LIBRARY
#define IANVS_PUBLIC IANVS_EXPORT
#else
#define IANVS_PUBLIC IANVS_IMPORT
#endif
#define IANVS_PUBLIC_TYPE IANVS_PUBLIC
#define IANVS_LOCAL
#else
#define IANVS_EXPORT __attribute__((visibility("default")))
#define IANVS_IMPORT
#if __GNUC__ >= 4
#define IANVS_PUBLIC __attribute__((visibility("default")))
#define IANVS_LOCAL __attribute__((visibility("hidden")))
#else
#define IANVS_PUBLIC
#define IANVS_LOCAL
#endif
#define IANVS_PUBLIC_TYPE
#endif

#endif  // IANVS__VISIBILITY_CONTROL_H_
