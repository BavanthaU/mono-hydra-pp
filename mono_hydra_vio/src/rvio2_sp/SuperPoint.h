/**
* SuperPoint front end for the RVIO2-SP variant.
*
* Runs a SuperPoint ONNX model that exposes the raw network heads
* ("semi": 1x65xHc xWc detector logits, "desc": 1xDxHcxWc descriptors) and
* provides NMS keypoints plus L2-normalised descriptors sampled at arbitrary
* image locations. ONNX Runtime is hidden behind a pimpl so that Tracker.h
* does not depend on its headers.
*/

#ifndef RVIO2SP_SUPERPOINT_H
#define RVIO2SP_SUPERPOINT_H

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>


namespace RVIO2SP
{

class SuperPoint
{
public:

    SuperPoint(const cv::FileStorage& fsSettings);

    ~SuperPoint();

    // Run the network on a grayscale 8-bit image of any size.
    void Compute(const cv::Mat& im);

    // NMS keypoints of the last image in image coordinates, sorted by
    // decreasing response, all with response >= SuperPoint.CandThreshold.
    const std::vector<cv::KeyPoint>& Keypoints() const;

    // Unit-norm descriptor at an image location of the last image.
    void Describe(const cv::Point2f& pt, float* desc) const;

    // Cosine similarity between a unit descriptor and the (normalised, bilinear)
    // descriptor field of the last image on a square grid around a centre:
    // offsets -radius..radius in steps of 'step' (row-major, x inner loop).
    // Exact equivalent of Describe() at every grid point, computed from cell
    // dot products.
    void SimilarityWindow(const float* desc, const cv::Point2f& center, int radius, int step,
                          std::vector<float>& sims) const;

    int DescriptorDim() const;

    bool UsingGpu() const;

    double LastInferenceMs() const;

private:

    struct Impl;
    std::unique_ptr<Impl> mpImpl;
};

// Squared L2 distance between two unit descriptors.
inline float DescDist2(const float* a, const float* b, const int dim)
{
    float dot = 0;
    for (int k=0; k<dim; ++k)
        dot += a[k]*b[k];
    return std::max(0.f, 2.f-2.f*dot);
}

} // namespace RVIO2SP

#endif
