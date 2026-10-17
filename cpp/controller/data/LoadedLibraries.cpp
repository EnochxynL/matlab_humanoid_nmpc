#include "LoadedLibraries.h"
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
void writeLoadedLibraries(const std::string& filename) {
#ifdef _WIN32
  std::ofstream output(filename);
  HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE | TH32CS_SNAPMODULE32,GetCurrentProcessId());
  if (snapshot == INVALID_HANDLE_VALUE) {
    output << "Module enumeration failed: " << GetLastError() << '\n';
    return;
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
  std::ofstream(filename) << maps.rdbuf();
#endif
}
}
