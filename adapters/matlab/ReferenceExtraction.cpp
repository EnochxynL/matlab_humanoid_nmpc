#include "ReferenceExtraction.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace g1::matlab {
std::array<double,428> extractReference(const std::array<double,428>& a) {
  for(double v:a) if(!std::isfinite(v)) throw std::runtime_error("Nonfinite reference field");
  if(a[1]!=0 && a[1]!=1) throw std::runtime_error("Invalid reference publication flag");
  // Unpublished slots carry no target; preserve their complete transport identity.
  if(a[1]==0) return a;
  if(a[0]<0 || a[0]>9007199254740992. || a[0]!=std::floor(a[0]))
    throw std::runtime_error("Invalid reference tick");
  if((a[2]!=1 && a[2]!=3)||a[285]<1||a[285]>64||a[285]!=std::floor(a[285]))
    throw std::runtime_error("Reference shape rejected");
  const auto n=static_cast<size_t>(a[2]),m=static_cast<size_t>(a[285]);
  for(size_t k=1;k<n;++k) if(a[3+k]<=a[2+k]) throw std::runtime_error("Target time order rejected");
  if(!std::is_sorted(a.begin()+286,a.begin()+286+m)) throw std::runtime_error("Event order rejected");
  for(size_t k=0;k<=m;++k) if(a[350+k]<0||a[350+k]>3||a[350+k]!=std::floor(a[350+k]))
    throw std::runtime_error("Unknown contact mode");
  if(a[415]<=a[286+m-1]) throw std::runtime_error("Terminal marker must follow events");
  const std::array<double,8> swing={.05,-0.0,.08,-.001,.4,-.15,.3,.005};
  if(!std::equal(swing.begin(),swing.end(),a.begin()+416)) throw std::runtime_error("Swing parameters changed");
  // No floating arithmetic is applied to target values, event times or commands.
  return a;
}
}
