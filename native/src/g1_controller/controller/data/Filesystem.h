#pragma once
#include <string>
// Platform file operations used only by application diagnostics.
#ifdef _WIN32
#include <boost/filesystem.hpp>
#include <boost/winapi/error_codes.hpp>
#include <chrono>
#include <thread>
namespace g1_filesystem = boost::filesystem;
#else
#include <filesystem>
namespace g1_filesystem = std::filesystem;
#endif

namespace g1_controller::data {
// Publish a complete diagnostic file. Windows can briefly deny replacement
// while the destination is open without delete sharing.
inline void replaceFile(const std::string& source, const std::string& destination) {
#ifdef _WIN32
  for (unsigned attempt = 0; ; ++attempt) {
    boost::system::error_code error;
    g1_filesystem::rename(source, destination, error);
    if (!error) return;
    const bool sharingConflict = error.category() == boost::system::system_category() &&
        (error.value() == boost::winapi::ERROR_ACCESS_DENIED_ ||
         error.value() == boost::winapi::ERROR_SHARING_VIOLATION_ ||
         error.value() == boost::winapi::ERROR_LOCK_VIOLATION_);
    // Eight attempts, at most 127 ms of requested sleep; permanent failures
    // and unrelated errors still propagate instead of losing diagnostics.
    if (!sharingConflict || attempt == 7)
      throw g1_filesystem::filesystem_error("replace diagnostic file", source, destination, error);
    std::this_thread::sleep_for(std::chrono::milliseconds(1u << attempt));
  }
#else
  g1_filesystem::rename(source, destination);
#endif
}
}
