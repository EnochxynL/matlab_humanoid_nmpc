#pragma once
#include <cstdint>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
// Sole controller-facing physical time conversion for every backend.
// Epoch is the physical initial state at zero, not an MPC nominal deadline.
class CanonicalControlTime {
 std::ofstream out_;uint64_t next_=0,session_=0;bool started_=false;
 public:
 static constexpr double period=.002;
 static double time(uint64_t tick){
  if(tick>(uint64_t{1}<<53))throw std::runtime_error("Tick exceeds exact double integer range");
  return static_cast<double>(tick)*period;
 }
 static double validate(uint64_t tick,double raw){
  double canonical=time(tick);
  if(!std::isfinite(raw)||std::abs(raw-canonical)>1e-12)throw std::runtime_error("Raw backend time exceeds canonical physical tick tolerance");
  return canonical;
 }
 explicit CanonicalControlTime(const std::string& root):out_(root+"/time_identity.csv"){
  out_<<std::setprecision(17)<<"raw_backend_time,control_tick,canonical_control_time,raw_minus_canonical,session_id,state_sequence\n";
 }
 double accept(uint64_t tick,double raw,uint64_t session,uint64_t sequence){
  if(tick!=next_ || (started_&&session!=session_))throw std::runtime_error("Physical tick/session discontinuity");
  double canonical=validate(tick,raw);
  out_<<raw<<','<<tick<<','<<canonical<<','<<raw-canonical<<','<<session<<','<<sequence<<'\n'<<std::flush;
  session_=session;started_=true;++next_;return canonical;
 }
};
