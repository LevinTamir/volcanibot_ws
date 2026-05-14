// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef VOLCANIBOT_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define VOLCANIBOT_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VOLCANIBOT_HARDWARE_INTERFACE_EXPORT __attribute__((dllexport))
    #define VOLCANIBOT_HARDWARE_INTERFACE_IMPORT __attribute__((dllimport))
  #else
    #define VOLCANIBOT_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
    #define VOLCANIBOT_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef VOLCANIBOT_HARDWARE_INTERFACE_BUILDING_DLL
    #define VOLCANIBOT_HARDWARE_INTERFACE_PUBLIC VOLCANIBOT_HARDWARE_INTERFACE_EXPORT
  #else
    #define VOLCANIBOT_HARDWARE_INTERFACE_PUBLIC VOLCANIBOT_HARDWARE_INTERFACE_IMPORT
  #endif
  #define VOLCANIBOT_HARDWARE_INTERFACE_PUBLIC_TYPE VOLCANIBOT_HARDWARE_INTERFACE_PUBLIC
  #define VOLCANIBOT_HARDWARE_INTERFACE_LOCAL
#else
  #define VOLCANIBOT_HARDWARE_INTERFACE_EXPORT __attribute__((visibility("default")))
  #define VOLCANIBOT_HARDWARE_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define VOLCANIBOT_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))
    #define VOLCANIBOT_HARDWARE_INTERFACE_LOCAL __attribute__((visibility("hidden")))
  #else
    #define VOLCANIBOT_HARDWARE_INTERFACE_PUBLIC
    #define VOLCANIBOT_HARDWARE_INTERFACE_LOCAL
  #endif
  #define VOLCANIBOT_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // VOLCANIBOT_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
