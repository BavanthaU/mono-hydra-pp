#pragma once
#include <ostream>

namespace kimera_rpgo {
enum class LossType { LS, HUBER, GM, TUKEY, TLS };

std::ostream& operator<<(std::ostream& os, const LossType& type);
}  // namespace kimera_rpgo
