#pragma once
#include <cstddef>
namespace humanoid::ocp_extensions {
class MeasuredModeReceiver {
public:
 virtual ~MeasuredModeReceiver() = default;
 virtual void setMeasuredMode(std::size_t mode) = 0;
};
}
