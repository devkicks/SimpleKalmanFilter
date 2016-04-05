#ifndef SIMPLEKALMANFILTER_H
#define SIMPLEKALMANFILTER_H

#include "common.h"

class SimpleKalmanFilter
{
public:
    SimpleKalmanFilter();
    SimpleKalmanFilter(const cv::Mat& inX, const cv::Mat& inP, const cv::Mat& inA, const cv::Mat& inB, const cv::Mat& inH, const cv::Mat& inQ, const cv::Mat& inR);

    cv::Mat getX();

    void setX(const cv::Mat& inX);
    void setP(const cv::Mat& inP);
    void setA(const cv::Mat& inA);
    void setB(const cv::Mat& inB);
    void setU(const cv::Mat& inU);
    void setH(const cv::Mat& inH);
    void setQ(const cv::Mat& inQ);
    void setR(const cv::Mat& inR);

    // predict the next state of X
    void predict();

    // correct next state of X and covariance P based of input measurement
    void correct(const cv::Mat& inZ);

    virtual ~SimpleKalmanFilter();

protected:
private:
    cv::Mat x, z, u, P, H, A, B, Q, R;
};

#endif // SIMPLEKALMANFILTER_H
