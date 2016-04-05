#include "SimpleKalmanFilter.h"

SimpleKalmanFilter::SimpleKalmanFilter()
{
    //ctor
}

SimpleKalmanFilter::SimpleKalmanFilter(const cv::Mat& inX, const cv::Mat& inP, const cv::Mat& inA, const cv::Mat& inB, const cv::Mat& inH, const cv::Mat& inQ, const cv::Mat& inR)
{
    //ctor
    this->setX(inX);
    this->setP(inP);
    this->setA(inA);
    this->setB(inB);
    this->setH(inH);
    this->setQ(inQ);
    this->setR(inR);

}


// set the initial covariance Matrix
void SimpleKalmanFilter::setX(const cv::Mat& inX)
{
    x = inX;
}


// set the initial covariance Matrix
void SimpleKalmanFilter::setP(const cv::Mat& inP)
{
    P = inP;
}


// set the state Transition Matrix
void SimpleKalmanFilter::setA(const cv::Mat& inA)
{
    A = inA;
}

// set the input matrix
void SimpleKalmanFilter::setB(const cv::Mat& inB)
{
    B = inB;
}

void SimpleKalmanFilter::setU(const cv::Mat& inU)
{
    u = inU;
}

// set the measurement Matrix
void SimpleKalmanFilter::setH(const cv::Mat& inH)
{
    H = inH;
}

// set the Process noise Covariance
void SimpleKalmanFilter::setQ(const cv::Mat& inQ)
{
    Q = inQ;
}

// set the measurement Noise Covariance
void SimpleKalmanFilter::setR(const cv::Mat& inR)
{
    R = inR;
}

cv::Mat SimpleKalmanFilter::getX()
{
    return x;
}

// predict the next state of X
void SimpleKalmanFilter::predict()
{
    std::cout << x.cols << ", " << x.rows << std::endl;
    std::cout << A.cols << ", " << A.rows << std::endl;
    std::cout << B.cols << ", " << B.rows << std::endl;
    std::cout << u.cols << ", " << u.rows << std::endl;

    this->x = this->A*this->x + this->B*this->u;
    this->P = this->A*this->P*this->A.t() + this->Q;

}

// correct next state of X and covariance P based of input measurement
void SimpleKalmanFilter::correct(const cv::Mat& inZ)
{
    // calculate the kalman gain
    cv::Mat K = P*H*(H*P*H.t() + R).inv();
    x = x + K*(inZ - H*x);
    P = P - K*H*P;
}

SimpleKalmanFilter::~SimpleKalmanFilter()
{
    //dtor
}
