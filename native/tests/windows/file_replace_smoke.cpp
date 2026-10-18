#include "controller/data/Filesystem.h"
#include <windows.h>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
void write(const std::string& path, const std::string& text) {
  std::ofstream out(path);
  out << text;
  if (!out) throw std::runtime_error("Cannot write fixture");
}
std::string read(const std::string& path) {
  std::ifstream in(path);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}
HANDLE lock(const std::string& path) {
  HANDLE file = CreateFileA(path.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE,
                            nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) throw std::runtime_error("Cannot lock fixture");
  return file;
}
}

int main() {
  try {
    const auto folder = (g1_filesystem::current_path() / "file replacement smoke").string();
    g1_filesystem::create_directories(folder);
    const auto source = folder + "/snapshot.tmp", destination = folder + "/snapshot.bin";
    write(source, "new snapshot");
    write(destination, "old snapshot");
    HANDLE held = lock(destination);
    std::thread release([held] {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      CloseHandle(held);
    });
    try { g1_controller::data::replaceFile(source, destination); }
    catch (...) { release.join(); throw; }
    release.join();
    if (read(destination) != "new snapshot" || g1_filesystem::exists(source))
      throw std::runtime_error("Transient lock did not publish complete snapshot");

    write(source, "next snapshot");
    held = lock(destination);
    bool rejected = false;
    try { g1_controller::data::replaceFile(source, destination); }
    catch (const g1_filesystem::filesystem_error&) { rejected = true; }
    catch (...) { CloseHandle(held); throw; }
    CloseHandle(held);
    if (!rejected || read(destination) != "new snapshot" || read(source) != "next snapshot")
      throw std::runtime_error("Permanent lock was hidden or snapshot changed");

    g1_filesystem::remove(source);
    g1_filesystem::remove(destination);
    g1_filesystem::remove(folder);
    std::cout << "DIAGNOSTIC_FILE_REPLACE_PASS\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
