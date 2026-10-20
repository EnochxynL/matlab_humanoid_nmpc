#pragma once
#include <string>
#include <vector>
#include <type_traits>

namespace g1_controller {
// MATLAB-independent diagnostic transport. Numbers remain numbers until the
// consumer chooses a storage format; no CSV or file paths belong here.
struct DiagnosticValue {
  double number = 0;
  std::string text;
  bool textual = false;
  template<class T, std::enable_if_t<std::is_arithmetic_v<T>, int> = 0>
  DiagnosticValue(T value) : number(static_cast<double>(value)) {}
  DiagnosticValue(const char* value) : text(value), textual(true) {}
  DiagnosticValue(const std::string& value) : text(value), textual(true) {}
};
struct DiagnosticTable {
  std::string name;
  std::vector<std::string> columns;
  std::vector<std::vector<DiagnosticValue>> rows;
};
struct DiagnosticBatch {
  std::vector<DiagnosticTable> tables;
  std::vector<double> snapshot;
  std::string configurationJson;
  std::string loadedLibraries;
};
} // namespace g1_controller
