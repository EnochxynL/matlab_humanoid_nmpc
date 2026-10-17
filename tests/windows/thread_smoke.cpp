#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <atomic>
#include <iostream>
int main() {
  std::atomic<int> completed{0};
  std::thread worker([&] { completed.store(1); });
  ocs2::setThreadPriority(0,worker);
  worker.join();
  if (completed.load() != 1) return 1;
  std::cout << "OCS2 native thread handle PASS\n";
}
