#include "kimera_rpgo/utils/types.h"

namespace kimera_rpgo {
std::ostream& operator<<(std::ostream& os, const LossType& type) {
  switch (type) {
    case LossType::LS:
      os << "Quadratic-Loss";
      break;
    case LossType::HUBER:
      os << "Huber-Loss";
      break;
    case LossType::GM:
      os << "Geman-McClure Loss";
      break;
    case LossType::TUKEY:
      os << "Tukey's-Biweight-Loss";
      break;
    case LossType::TLS:
      os << "Truncated-Quadratic-Loss";
      break;
  }
  return os;
}

}  // namespace kimera_rpgo
