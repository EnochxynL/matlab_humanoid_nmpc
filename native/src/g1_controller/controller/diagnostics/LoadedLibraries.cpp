#include "LoadedLibraries.h"
#include <sstream>
#include <fstream>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <tlhelp32.h>
#endif

namespace g1_controller {
std::string loadedLibraries() {
  std::ostringstream output;
#ifdef _WIN32

  HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE | TH32CS_SNAPMODULE32,GetCurrentProcessId());
  if (snapshot == INVALID_HANDLE_VALUE) {
    output << "Module enumeration failed: " << GetLastError() << '\n';
    return output.str();
  }
  MODULEENTRY32 module{};
  module.dwSize = sizeof(module);
  if (Module32First(snapshot,&module)) {
    do { output << static_cast<const void*>(module.modBaseAddr) << ' ' << module.szExePath << '\n'; }
    while (Module32Next(snapshot,&module));
  }
  CloseHandle(snapshot);
#else
  std::ifstream maps("/proc/self/maps");
  output << maps.rdbuf();
#endif
  return output.str();
}
}
