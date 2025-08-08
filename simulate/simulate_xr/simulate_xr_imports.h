// Copyright 2024 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SIMULATE_XR_IMPORTS_H_
#define SIMULATE_XR_IMPORTS_H_

// the code only works with Windows, because support for XR on Linux
// and Mac is limited or nonexistent
#ifdef _WIN32

// Windows is needed but without minmax
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#include <unknwn.h>

// TODO future setup for Android build on devices
// for proper init of openxr
#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_OPENGL

#endif // WIN32

// openxr with render by opengl
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

// to link to windows, default classes, warnings
#include <mujoco/mujoco.h>

#endif  // SIMULATE_XR_IMPORTS_H_
