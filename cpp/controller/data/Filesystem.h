#pragma once
// GCC 8.1's Windows std::filesystem is incomplete. Keep Linux's implementation.
#ifdef _WIN32
#include <boost/filesystem.hpp>
namespace g1_filesystem = boost::filesystem;
#else
#include <filesystem>
namespace g1_filesystem = std::filesystem;
#endif
