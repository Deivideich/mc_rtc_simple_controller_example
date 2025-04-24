#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define SimpleHandFollowerController_DLLIMPORT __declspec(dllimport)
#  define SimpleHandFollowerController_DLLEXPORT __declspec(dllexport)
#  define SimpleHandFollowerController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define SimpleHandFollowerController_DLLIMPORT __attribute__((visibility("default")))
#    define SimpleHandFollowerController_DLLEXPORT __attribute__((visibility("default")))
#    define SimpleHandFollowerController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define SimpleHandFollowerController_DLLIMPORT
#    define SimpleHandFollowerController_DLLEXPORT
#    define SimpleHandFollowerController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef SimpleHandFollowerController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define SimpleHandFollowerController_DLLAPI
#  define SimpleHandFollowerController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef SimpleHandFollowerController_EXPORTS
#    define SimpleHandFollowerController_DLLAPI SimpleHandFollowerController_DLLEXPORT
#  else
#    define SimpleHandFollowerController_DLLAPI SimpleHandFollowerController_DLLIMPORT
#  endif // SimpleHandFollowerController_EXPORTS
#  define SimpleHandFollowerController_LOCAL SimpleHandFollowerController_DLLLOCAL
#endif // SimpleHandFollowerController_STATIC