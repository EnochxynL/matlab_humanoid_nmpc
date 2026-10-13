#pragma once
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <memory>
namespace g1 {
inline std::mutex termMutex;
inline std::unordered_set<std::string> registeredTermNames;
template<class Collection, class Term>
void addTerm(Collection& c, std::string name, std::unique_ptr<Term> term) {
 {std::lock_guard<std::mutex> lock(termMutex);registeredTermNames.insert(name);}
 c.add(std::move(name), std::move(term));
}
template<class Collection>
std::unordered_map<std::string,size_t> termNames(const Collection& c) {
 std::lock_guard<std::mutex> lock(termMutex);
 std::unordered_map<std::string,size_t> result;
 for(const auto& name:registeredTermNames){size_t index;if(c.getTermIndex(name,index))result.emplace(name,index);}
 return result;
}
}
