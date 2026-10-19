/**
* SuperPoint front end for the RVIO2-SP variant.
*/

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include <onnxruntime_cxx_api.h>

#include <opencv2/opencv.hpp>

#include "SuperPoint.h"


namespace RVIO2SP
{

namespace
{

template <typename T>
T ReadOr(const cv::FileStorage& fs, const std::string& key, const T& def)
{
    const cv::FileNode node = fs[key];
    if (node.empty())
        return def;
    T value;
    node >> value;
    return value;
}

} // namespace


struct SuperPoint::Impl
{
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "rvio2_sp"};
    Ort::SessionOptions options;
    std::unique_ptr<Ort::Session> session;
    Ort::MemoryInfo memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    std::string inputName;
    std::string semiName;
    std::string descName;

    bool bUsingGpu = false;

    // Network input size (multiples of 8)
    int W = 0;
    int H = 0;
    const int cell = 8;

    float detThreshold = 0.005f;
    float candThreshold = 0.005f;
    int nmsRadius = 4;
    int border = 8;
    int maxKeypoints = 2000;

    // Last image
    int imCols = 0;
    int imRows = 0;
    std::vector<float> input;
    std::vector<cv::KeyPoint> keypoints;
    // Unit-norm coarse descriptors in HWC order
    std::vector<float> coarse;
    int Hc = 0;
    int Wc = 0;
    int D = 0;
    double lastMs = 0;

    // Dot products between neighbouring coarse cells (Hc x Wc each):
    // right (y,x)-(y,x+1), down (y,x)-(y+1,x), diag (y,x)-(y+1,x+1), anti (y,x+1)-(y+1,x)
    std::vector<float> dotRight, dotDown, dotDiag, dotAnti;
};


SuperPoint::SuperPoint(const cv::FileStorage& fsSettings)
    : mpImpl(new Impl)
{
    Impl& m = *mpImpl;

    const std::string strModelPath = ReadOr<std::string>(fsSettings, "SuperPoint.ModelPath", "");
    if (strModelPath.empty())
        throw std::runtime_error("SuperPoint.ModelPath is not set");

    const bool bUseGpu = ReadOr<int>(fsSettings, "SuperPoint.UseGpu", 1);
    const bool bRequireGpu = ReadOr<int>(fsSettings, "SuperPoint.RequireGpu", 0);
    const int nDeviceId = ReadOr<int>(fsSettings, "SuperPoint.DeviceId", 0);
    const int nThreads = ReadOr<int>(fsSettings, "SuperPoint.nThreads", 2);

    m.detThreshold = ReadOr<float>(fsSettings, "SuperPoint.DetThreshold", 0.005f);
    m.candThreshold = std::min(m.detThreshold, ReadOr<float>(fsSettings, "SuperPoint.CandThreshold", 0.005f));
    m.nmsRadius = ReadOr<int>(fsSettings, "SuperPoint.NmsRadius", 4);
    m.border = ReadOr<int>(fsSettings, "SuperPoint.Border", 8);
    m.maxKeypoints = ReadOr<int>(fsSettings, "SuperPoint.MaxKeypoints", 2000);

    m.options.SetIntraOpNumThreads(nThreads);
    m.options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    if (bUseGpu)
    {
        try
        {
            OrtCUDAProviderOptions cudaOptions;
            cudaOptions.device_id = nDeviceId;
            m.options.AppendExecutionProvider_CUDA(cudaOptions);
            m.bUsingGpu = true;
        }
        catch (const Ort::Exception& e)
        {
            if (bRequireGpu)
                throw std::runtime_error(std::string("SuperPoint: CUDA provider unavailable: ")+e.what());
            std::cerr << "SuperPoint: CUDA provider unavailable, running on CPU (" << e.what() << ")\n";
        }
    }

    m.session.reset(new Ort::Session(m.env, strModelPath.c_str(), m.options));

    Ort::AllocatorWithDefaultOptions allocator;
    m.inputName = m.session->GetInputNameAllocated(0, allocator).get();

    std::vector<int64_t> inShape = m.session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (inShape.size()!=4 || inShape[1]!=1)
        throw std::runtime_error("SuperPoint: expected a 1x1xHxW grayscale input");
    m.H = (int)inShape[2];
    m.W = (int)inShape[3];
    if (m.H<=0 || m.W<=0)
    {
        // Dynamic model: use the camera resolution rounded down to cells
        const int nCols = ReadOr<int>(fsSettings, "Camera.width", 0);
        const int nRows = ReadOr<int>(fsSettings, "Camera.height", 0);
        m.W = nCols/m.cell*m.cell;
        m.H = nRows/m.cell*m.cell;
    }
    if (m.H<=0 || m.W<=0 || m.H%m.cell || m.W%m.cell)
        throw std::runtime_error("SuperPoint: input size must be a positive multiple of 8");

    // Identify the detector head by its 65 channels
    const size_t nOutputs = m.session->GetOutputCount();
    if (nOutputs<2)
        throw std::runtime_error("SuperPoint: model must output detector logits and descriptors");
    for (size_t i=0; i<nOutputs; ++i)
    {
        std::string name = m.session->GetOutputNameAllocated(i, allocator).get();
        std::vector<int64_t> shape = m.session->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
        if (shape.size()==4 && shape[1]==65 && m.semiName.empty())
            m.semiName = name;
        else if (m.descName.empty())
            m.descName = name;
    }
    if (m.semiName.empty() || m.descName.empty())
        throw std::runtime_error("SuperPoint: could not identify 'semi' (65 channels) and descriptor outputs");

    m.input.assign(m.W*m.H, 0);

    std::cout << "SuperPoint: model=" << strModelPath << " input=" << m.W << "x" << m.H
              << " provider=" << (m.bUsingGpu ? "CUDA" : "CPU") << "\n";

    // Warm up so the first tracked frames are not delayed by provider setup
    Compute(cv::Mat(m.H, m.W, CV_8UC1, cv::Scalar(0)));
    Compute(cv::Mat(m.H, m.W, CV_8UC1, cv::Scalar(0)));
}


SuperPoint::~SuperPoint() = default;


void SuperPoint::Compute(const cv::Mat& im)
{
    Impl& m = *mpImpl;
    const auto tic = std::chrono::steady_clock::now();

    cv::Mat gray;
    if (im.channels()==3)
        cv::cvtColor(im, gray, cv::COLOR_BGR2GRAY);
    else if (im.channels()==4)
        cv::cvtColor(im, gray, cv::COLOR_BGRA2GRAY);
    else
        gray = im;

    m.imCols = gray.cols;
    m.imRows = gray.rows;

    cv::Mat resized;
    if (gray.cols!=m.W || gray.rows!=m.H)
        cv::resize(gray, resized, cv::Size(m.W,m.H), 0, 0, cv::INTER_AREA);
    else
        resized = gray;

    cv::Mat inputMat(m.H, m.W, CV_32F, m.input.data());
    resized.convertTo(inputMat, CV_32F, 1./255.);

    const std::vector<int64_t> inDims = {1, 1, m.H, m.W};
    Ort::Value inTensor = Ort::Value::CreateTensor<float>(m.memInfo, m.input.data(), m.input.size(), inDims.data(), inDims.size());

    const char* inNames[] = {m.inputName.c_str()};
    const char* outNames[] = {m.semiName.c_str(), m.descName.c_str()};
    std::vector<Ort::Value> outputs = m.session->Run(Ort::RunOptions{nullptr}, inNames, &inTensor, 1, outNames, 2);

    // Detector head: softmax over 65 channels, drop dustbin, depth-to-space
    const std::vector<int64_t> semiShape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    const int Hc = (int)semiShape[2];
    const int Wc = (int)semiShape[3];
    const float* semi = outputs[0].GetTensorData<float>();

    cv::Mat heat(Hc*m.cell, Wc*m.cell, CV_32F, cv::Scalar(0));
    const int plane = Hc*Wc;
    float logits[65];
    for (int cy=0; cy<Hc; ++cy)
    {
        for (int cx=0; cx<Wc; ++cx)
        {
            const int idx = cy*Wc+cx;
            float maxLogit = -1e30f;
            for (int c=0; c<65; ++c)
            {
                logits[c] = semi[c*plane+idx];
                maxLogit = std::max(maxLogit, logits[c]);
            }
            float sum = 0;
            for (int c=0; c<65; ++c)
            {
                logits[c] = std::exp(logits[c]-maxLogit);
                sum += logits[c];
            }
            for (int c=0; c<64; ++c)
                heat.at<float>(cy*m.cell+c/m.cell, cx*m.cell+c%m.cell) = logits[c]/sum;
        }
    }

    // Max-pool NMS
    cv::Mat dilated;
    const int k = 2*m.nmsRadius+1;
    cv::dilate(heat, dilated, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k,k)));

    const float sx = (float)m.imCols/m.W;
    const float sy = (float)m.imRows/m.H;

    m.keypoints.clear();
    for (int y=m.border; y<heat.rows-m.border; ++y)
    {
        const float* h = heat.ptr<float>(y);
        const float* d = dilated.ptr<float>(y);
        for (int x=m.border; x<heat.cols-m.border; ++x)
        {
            if (h[x]>=m.candThreshold && h[x]>=d[x])
                m.keypoints.emplace_back(cv::Point2f(x*sx, y*sy), 1.f, -1, h[x]);
        }
    }

    std::sort(m.keypoints.begin(), m.keypoints.end(),
              [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return a.response>b.response; });
    if ((int)m.keypoints.size()>m.maxKeypoints)
        m.keypoints.resize(m.maxKeypoints);

    // Descriptor head: normalise each cell, store in HWC order
    const std::vector<int64_t> descShape = outputs[1].GetTensorTypeAndShapeInfo().GetShape();
    m.D = (int)descShape[1];
    m.Hc = (int)descShape[2];
    m.Wc = (int)descShape[3];
    const float* desc = outputs[1].GetTensorData<float>();
    const int descPlane = m.Hc*m.Wc;

    m.coarse.resize(descPlane*m.D);
    for (int i=0; i<descPlane; ++i)
    {
        float* out = &m.coarse[i*m.D];
        float norm = 0;
        for (int c=0; c<m.D; ++c)
        {
            out[c] = desc[c*descPlane+i];
            norm += out[c]*out[c];
        }
        norm = std::sqrt(norm)+1e-12f;
        for (int c=0; c<m.D; ++c)
            out[c] /= norm;
    }

    const auto cellDot = [&](int y0, int x0, int y1, int x1) {
        const float* a = &m.coarse[(y0*m.Wc+x0)*m.D];
        const float* b = &m.coarse[(y1*m.Wc+x1)*m.D];
        float dot = 0;
        for (int c=0; c<m.D; ++c)
            dot += a[c]*b[c];
        return dot;
    };
    m.dotRight.assign(descPlane, 0);
    m.dotDown.assign(descPlane, 0);
    m.dotDiag.assign(descPlane, 0);
    m.dotAnti.assign(descPlane, 0);
    for (int y=0; y<m.Hc; ++y)
    {
        for (int x=0; x<m.Wc; ++x)
        {
            const int i = y*m.Wc+x;
            if (x+1<m.Wc)
                m.dotRight[i] = cellDot(y, x, y, x+1);
            if (y+1<m.Hc)
                m.dotDown[i] = cellDot(y, x, y+1, x);
            if (x+1<m.Wc && y+1<m.Hc)
            {
                m.dotDiag[i] = cellDot(y, x, y+1, x+1);
                m.dotAnti[i] = cellDot(y, x+1, y+1, x);
            }
        }
    }

    m.lastMs = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tic).count();
}


const std::vector<cv::KeyPoint>& SuperPoint::Keypoints() const
{
    return mpImpl->keypoints;
}


void SuperPoint::Describe(const cv::Point2f& pt, float* out) const
{
    const Impl& m = *mpImpl;

    // Image -> network pixel coordinates
    const float xm = pt.x*m.W/m.imCols;
    const float ym = pt.y*m.H/m.imRows;

    // Same convention as the reference SuperPoint descriptor sampler
    // (grid_sample with align_corners over cell centres)
    const float s = (float)m.cell;
    float gx = (xm-s/2+.5f)/(m.Wc*s-s/2-.5f)*(m.Wc-1);
    float gy = (ym-s/2+.5f)/(m.Hc*s-s/2-.5f)*(m.Hc-1);
    gx = std::min(std::max(gx, 0.f), (float)(m.Wc-1));
    gy = std::min(std::max(gy, 0.f), (float)(m.Hc-1));

    const int x0 = std::min((int)gx, m.Wc-2 < 0 ? 0 : m.Wc-2);
    const int y0 = std::min((int)gy, m.Hc-2 < 0 ? 0 : m.Hc-2);
    const int x1 = std::min(x0+1, m.Wc-1);
    const int y1 = std::min(y0+1, m.Hc-1);
    const float ax = gx-x0;
    const float ay = gy-y0;

    const float* d00 = &m.coarse[(y0*m.Wc+x0)*m.D];
    const float* d01 = &m.coarse[(y0*m.Wc+x1)*m.D];
    const float* d10 = &m.coarse[(y1*m.Wc+x0)*m.D];
    const float* d11 = &m.coarse[(y1*m.Wc+x1)*m.D];

    const float w00 = (1-ax)*(1-ay);
    const float w01 = ax*(1-ay);
    const float w10 = (1-ax)*ay;
    const float w11 = ax*ay;

    float norm = 0;
    for (int c=0; c<m.D; ++c)
    {
        out[c] = w00*d00[c]+w01*d01[c]+w10*d10[c]+w11*d11[c];
        norm += out[c]*out[c];
    }
    norm = std::sqrt(norm)+1e-12f;
    for (int c=0; c<m.D; ++c)
        out[c] /= norm;
}


void SuperPoint::SimilarityWindow(const float* desc, const cv::Point2f& center, int radius, int step,
                                  std::vector<float>& sims) const
{
    const Impl& m = *mpImpl;
    const float s = (float)m.cell;
    const int n = 2*(radius/step)+1;
    sims.assign(n*n, -1.f);

    // Dot products of the query with the cells covering the window
    const auto toGrid = [&](float px, float py, float& gx, float& gy) {
        const float xm = px*m.W/m.imCols;
        const float ym = py*m.H/m.imRows;
        gx = std::min(std::max((xm-s/2+.5f)/(m.Wc*s-s/2-.5f)*(m.Wc-1), 0.f), (float)(m.Wc-1));
        gy = std::min(std::max((ym-s/2+.5f)/(m.Hc*s-s/2-.5f)*(m.Hc-1), 0.f), (float)(m.Hc-1));
    };
    float gx0, gy0, gx1, gy1;
    toGrid(center.x-radius, center.y-radius, gx0, gy0);
    toGrid(center.x+radius, center.y+radius, gx1, gy1);
    const int cx0 = std::max((int)gx0, 0), cy0 = std::max((int)gy0, 0);
    const int cx1 = std::min((int)gx1+1, m.Wc-1), cy1 = std::min((int)gy1+1, m.Hc-1);
    const int cw = cx1-cx0+1;
    std::vector<float> cellSim((cy1-cy0+1)*cw);
    for (int y=cy0; y<=cy1; ++y)
    {
        for (int x=cx0; x<=cx1; ++x)
        {
            const float* c = &m.coarse[(y*m.Wc+x)*m.D];
            float dot = 0;
            for (int k=0; k<m.D; ++k)
                dot += desc[k]*c[k];
            cellSim[(y-cy0)*cw+(x-cx0)] = dot;
        }
    }

    int idx = 0;
    for (int oy=-radius; oy<=radius; oy+=step)
    {
        for (int ox=-radius; ox<=radius; ox+=step, ++idx)
        {
            float gx, gy;
            toGrid(center.x+ox, center.y+oy, gx, gy);
            const int x0 = std::min((int)gx, std::max(m.Wc-2, 0));
            const int y0 = std::min((int)gy, std::max(m.Hc-2, 0));
            const int x1 = std::min(x0+1, m.Wc-1);
            const int y1 = std::min(y0+1, m.Hc-1);
            if (x0<cx0 || y0<cy0 || x1>cx1 || y1>cy1)
                continue;
            const float ax = gx-x0, ay = gy-y0;
            const float w00 = (1-ax)*(1-ay), w01 = ax*(1-ay), w10 = (1-ax)*ay, w11 = ax*ay;

            const float num = w00*cellSim[(y0-cy0)*cw+(x0-cx0)]+w01*cellSim[(y0-cy0)*cw+(x1-cx0)]
                             +w10*cellSim[(y1-cy0)*cw+(x0-cx0)]+w11*cellSim[(y1-cy0)*cw+(x1-cx0)];
            const int i00 = y0*m.Wc+x0;
            float norm2 = w00*w00+w01*w01+w10*w10+w11*w11
                         +2*(w00*w01*m.dotRight[i00]+w00*w10*m.dotDown[i00]+w00*w11*m.dotDiag[i00]
                            +w01*w10*m.dotAnti[i00]+w01*w11*m.dotDown[y0*m.Wc+x1]+w10*w11*m.dotRight[y1*m.Wc+x0]);
            sims[idx] = num/std::sqrt(std::max(norm2, 1e-12f));
        }
    }
}


int SuperPoint::DescriptorDim() const
{
    return mpImpl->D;
}


bool SuperPoint::UsingGpu() const
{
    return mpImpl->bUsingGpu;
}


double SuperPoint::LastInferenceMs() const
{
    return mpImpl->lastMs;
}

} // namespace RVIO2SP
