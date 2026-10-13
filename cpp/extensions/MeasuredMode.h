#pragma once
#include <cstddef>
namespace g1 {
class MeasuredModeReceiver {
public:
 virtual ~MeasuredModeReceiver() = default;
 virtual void setMeasuredMode(std::size_t mode) = 0;
};
}
