#pragma once
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
#include <fstream>

// Native Windows implementation of the pinned CppADCodeGen system interface.
namespace CppAD { namespace cg { namespace system {
template<class T> const std::string SystemInfo<T>::DYNAMIC_LIB_EXTENSION = ".dll";
template<class T> const std::string SystemInfo<T>::STATIC_LIB_EXTENSION = ".a";
inline std::string getWorkingDirectory() {
  const DWORD size = GetCurrentDirectoryA(0,nullptr);
  std::string path(size,'\0');
  if (!size || !GetCurrentDirectoryA(size,&path[0])) throw CGException("Cannot read working directory");
  path.resize(size-1);
  std::replace(path.begin(),path.end(),'\\','/');
  return path;
}
inline bool isDirectory(const std::string& p) {
  DWORD attributes = GetFileAttributesA(p.c_str());
  return attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_DIRECTORY);
}
inline void createFolder(const std::string& p) {
  if (p.empty() || isDirectory(p)) return;
  const auto slash = p.find_last_of("/\\");
  if (slash != std::string::npos && slash > 0) createFolder(p.substr(0,slash));
  if (!CreateDirectoryA(p.c_str(),nullptr) && !isDirectory(p)) throw CGException("Cannot create folder: ",p);
}
inline std::string createPath(const std::string& p, const std::string& f) {
  return p.empty() ? f : p + (p.back() == '/' || p.back() == '\\' ? "" : "/") + f;
}
inline std::string filenameFromPath(const std::string& p) {
  const auto slash = p.find_last_of("/\\");
  return slash == std::string::npos ? p : p.substr(slash+1);
}
inline std::string directoryFromPath(const std::string& p) {
  const auto slash = p.find_last_of("/\\");
  return slash == std::string::npos ? "./" : p.substr(0,slash+1);
}
inline bool isAbsolutePath(const std::string& p) {
  return !p.empty() && (p[0] == '/' || p[0] == '\\' || (p.size() > 2 && p[1] == ':'));
}
inline bool isFile(const std::string& p) {
  DWORD attributes = GetFileAttributesA(p.c_str());
  return attributes != INVALID_FILE_ATTRIBUTES && !(attributes & FILE_ATTRIBUTE_DIRECTORY);
}
// Windows CRT argument quoting, including backslashes immediately before quotes.
inline std::string escapePath(const std::string& s) {
  std::string out = "\"";
  size_t slashes = 0;
  for (char c : s) {
    if (c == '\\') { ++slashes; continue; }
    out.append(c == '"' ? 2 * slashes + 1 : slashes, '\\');
    out += c;
    slashes = 0;
  }
  out.append(2 * slashes, '\\');
  return out + '"';
}
namespace windows_detail {
struct TemporaryFile {
  HANDLE handle = INVALID_HANDLE_VALUE;
  std::string path;
  TemporaryFile() {
    char directory[MAX_PATH], name[MAX_PATH];
    if (!GetTempPathA(MAX_PATH, directory) || !GetTempFileNameA(directory,"g1a",0,name))
      throw CGException("Cannot create AD compiler temporary file");
    path = name;
    SECURITY_ATTRIBUTES security{sizeof(SECURITY_ATTRIBUTES),nullptr,1};
    handle = CreateFileA(name,GENERIC_READ | GENERIC_WRITE,FILE_SHARE_READ | FILE_SHARE_DELETE,
                         &security,OPEN_EXISTING,FILE_ATTRIBUTE_TEMPORARY,nullptr);
    if (handle == INVALID_HANDLE_VALUE) { DeleteFileA(name); throw CGException("Cannot open AD temporary file"); }
  }
  ~TemporaryFile() { if (handle != INVALID_HANDLE_VALUE) CloseHandle(handle); DeleteFileA(path.c_str()); }
  TemporaryFile(const TemporaryFile&) = delete;
  TemporaryFile& operator=(const TemporaryFile&) = delete;
};
}
inline void callExecutable(const std::string& executable, const std::vector<std::string>& args,
                           std::string* output, const std::string* input) {
  // Disk-backed streams avoid pipe deadlocks when compiler input/output is large.
  windows_detail::TemporaryFile in, out;
  if (input) {
    size_t offset = 0;
    while (offset < input->size()) {
      DWORD written = 0;
      DWORD count = static_cast<DWORD>(std::min<size_t>(input->size()-offset,1024*1024));
      if (!WriteFile(in.handle,input->data()+offset,count,&written,nullptr) || !written)
        throw CGException("Cannot write AD compiler input");
      offset += written;
    }
  }
  LARGE_INTEGER zero{};
  SetFilePointerEx(in.handle,zero,nullptr,FILE_BEGIN);
  std::string command = escapePath(executable);
  for (const auto& arg : args) command += " " + escapePath(arg);
  STARTUPINFOA startup{};
  startup.cb = sizeof(startup);
  startup.dwFlags = STARTF_USESTDHANDLES | STARTF_USESHOWWINDOW;
  startup.wShowWindow = SW_HIDE;
  startup.hStdInput = in.handle;
  startup.hStdOutput = startup.hStdError = out.handle;
  PROCESS_INFORMATION process{};
  // A null application name lets Windows resolve a bare compiler name via PATH.
  // command already quotes the executable, including absolute paths with spaces.
  if (!CreateProcessA(nullptr,&command[0],nullptr,nullptr,1,CREATE_NO_WINDOW,
                      nullptr,nullptr,&startup,&process))
    throw CGException("Cannot launch AD compiler: ",executable," Windows error ",GetLastError());
  CloseHandle(process.hThread);
  const DWORD waited = WaitForSingleObject(process.hProcess,INFINITE);
  DWORD status = 1;
  const BOOL obtained = GetExitCodeProcess(process.hProcess,&status);
  CloseHandle(process.hProcess);
  SetFilePointerEx(out.handle,zero,nullptr,FILE_BEGIN);
  std::string message;
  char buffer[4096]; DWORD count = 0;
  while (ReadFile(out.handle,buffer,sizeof(buffer),&count,nullptr) && count) message.append(buffer,count);
  if (output) *output = message;
  if (waited != WAIT_OBJECT_0 || !obtained || status != 0)
    throw CGException("AD compiler failed (",status,"): ",message);
}
}}}
