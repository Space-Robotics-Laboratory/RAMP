// Copyright (c) 2026 Masazumi Imai
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

#ifndef MOMENTUM_DISTRIBUTION__VISIBILITY_CONTROL_HPP_
#define MOMENTUM_DISTRIBUTION__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MOMENTUM_DISTRIBUTION_EXPORT __attribute__((dllexport))
#define MOMENTUM_DISTRIBUTION_IMPORT __attribute__((dllimport))
#else
#define MOMENTUM_DISTRIBUTION_EXPORT __declspec(dllexport)
#define MOMENTUM_DISTRIBUTION_IMPORT __declspec(dllimport)
#endif
#ifdef MOMENTUM_DISTRIBUTION_BUILDING_LIBRARY
#define MOMENTUM_DISTRIBUTION_PUBLIC MOMENTUM_DISTRIBUTION_EXPORT
#else
#define MOMENTUM_DISTRIBUTION_PUBLIC MOMENTUM_DISTRIBUTION_IMPORT
#endif
#define MOMENTUM_DISTRIBUTION_PUBLIC_TYPE MOMENTUM_DISTRIBUTION_PUBLIC
#define MOMENTUM_DISTRIBUTION_LOCAL
#else
#define MOMENTUM_DISTRIBUTION_EXPORT __attribute__((visibility("default")))
#define MOMENTUM_DISTRIBUTION_IMPORT
#if __GNUC__ >= 4
#define MOMENTUM_DISTRIBUTION_PUBLIC __attribute__((visibility("default")))
#define MOMENTUM_DISTRIBUTION_LOCAL __attribute__((visibility("hidden")))
#else
#define MOMENTUM_DISTRIBUTION_PUBLIC
#define MOMENTUM_DISTRIBUTION_LOCAL
#endif
#define MOMENTUM_DISTRIBUTION_PUBLIC_TYPE
#endif

#endif  // MOMENTUM_DISTRIBUTION__VISIBILITY_CONTROL_HPP_
