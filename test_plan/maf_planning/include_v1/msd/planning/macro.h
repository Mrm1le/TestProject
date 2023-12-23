/* -*- mode:c -*- */
#pragma once

#ifdef __cplusplus
#define MSD_API_BEGIN extern "C" {
#define MSD_API_END }
#else
#define MSD_API_BEGIN
#define MSD_API_END
#endif

#if MSD_SHARED_LIBRARY
#if defined(_WIN32)
#if MSD_EXPORT
#define MSD_API __declspec(dllexport)
#else
#define MSD_API __declspec(dllimport)
#endif
#else
#if MSD_EXPORT
#define MSD_API __attribute__((visibility("default")))
#else
#define MSD_API
#endif
#endif
#else
#define MSD_API
#endif

#ifdef __GNUC__
#define MSD_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define MSD_DEPRECATED __declspec(deprecated)
#else
#define MSD_DEPRECATED
#endif

// vim: syntax=cpp.doxygen foldmethod=marker foldmarker=f{{{,f}}}
