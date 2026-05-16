/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <vector>

#include <gtsam/base/Matrix.h>

#include "kimera-vio/common/vio_types.h"

namespace VIO {

struct SparseDepthMeasurement {
  LandmarkId lmk_id_;
  double depth_meas_;
  double u_;
  double v_;
};

using SparseDepthMeasurements = std::vector<SparseDepthMeasurement>;

}  // namespace VIO
