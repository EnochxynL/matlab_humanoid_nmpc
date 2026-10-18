#pragma once
#include <string>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#undef CONST
#undef interface
#undef near
#undef far
#undef FALSE
#undef TRUE
#else
#include <cstdlib>
#endif

namespace g1_platform {
// MATLAB and MinGW use different CRT environments. Query the process on Windows.
inline std::string environmentVariable(const char* name) {
#ifdef _WIN32
  DWORD count = GetEnvironmentVariableA(name,nullptr,0);
  if (!count) return {};
  std::string value(count,'\0');
  DWORD copied = GetEnvironmentVariableA(name,&value[0],count);
  if (!copied || copied >= count) return {};
  value.resize(copied);
  return value;
#else
  const char* value = std::getenv(name);
  return value ? value : "";
#endif
}
}
