#pragma once
#include "g1_controller/Diagnostics.h"
#include <map>
#include <stdexcept>
#include <sstream>

namespace g1_controller::diagnostics {
class DiagnosticBuffer {
 public:
  DiagnosticTable& define(const char* name, const char* columns) {
    auto& table = tables_[name]; table.name = name;
    std::istringstream input(columns); std::string column;
    while (std::getline(input,column,',')) table.columns.push_back(column);
    return table;
  }
  void row(const char* name, std::vector<DiagnosticValue> values) {
    auto& table = tables_.at(name);
    if (values.size()!=table.columns.size()) throw std::runtime_error("Diagnostic row/schema mismatch");
    // Direct native callers must drain too; fail explicitly rather than growing
    // without bound or silently dropping samples.
    if (rows_ >= 250000) throw std::runtime_error("Drain diagnostic batches before continuing");
    table.rows.push_back(std::move(values)); ++rows_;
  }
  DiagnosticBatch drain() {
    DiagnosticBatch result;
    for (auto& [name,table] : tables_) {
      result.tables.push_back({name,table.columns,{}});
      result.tables.back().rows.swap(table.rows);
    }
    result.snapshot.swap(snapshot);
    result.configurationJson.swap(configurationJson);
    result.loadedLibraries.swap(loadedLibraries);
    rows_=0;
    return result;
  }
  std::vector<double> snapshot;
  std::string configurationJson, loadedLibraries;
 private:
  std::map<std::string,DiagnosticTable> tables_;
  size_t rows_=0;
};
} // namespace g1_controller::diagnostics
