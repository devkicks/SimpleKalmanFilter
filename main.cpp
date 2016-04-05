#include <iostream>
#include "common.h"
#include "SimpleKalmanFilter.h"


int main()
{
    // initialize all matrices
    cv::Mat A, H, X, P, Q, R, u, B;
    SimpleKalmanFilter myTracker;

    A = cv::Mat::zeros(4, 4, CV_64FC1);
    H = cv::Mat::zeros(2, 4, CV_64FC1);
    X = cv::Mat::zeros(2, 4, CV_64FC1);
    P = cv::Mat::eye(4, 4, CV_64FC1);
    Q = cv::Mat::eye(4, 4, CV_64FC1);
    R = cv::Mat::eye(2, 2, CV_64FC1);

    P = P/10.0;
    Q = Q/5.0;
    R = R/5.0;

    // not using B and u - therefore setting these to zero
    u = cv::Mat::zeros(1, 1, CV_64FC1);
    B = cv::Mat::zeros(1, 1, CV_64FC1);

    // position
    A.at<double>(0, 0) = 1;
    A.at<double>(0, 1) = 1;
    A.at<double>(1, 2) = 1;
    A.at<double>(1, 3) = 1;

    // velocity
    A.at<double>(3, 1) = 1;
    A.at<double>(3, 3) = 1;

    // state measurement matrix
    H.at<double>(0, 0) = 1;
    H.at<double>(1, 1) = 1;

    X.at<double>(0, 0) = 1;
    X.at<double>(1, 2) = 1;

    myTracker.setX(X);
    myTracker.setP(P);
    myTracker.setA(A);
    myTracker.setB(B);
    myTracker.setU(u);
    myTracker.setH(H);
    myTracker.setQ(Q);
    myTracker.setR(R);

    double xI, xJ;
    xJ = 10.0;
    cv::Mat x;
    x = cv::Mat::zeros(2, 1, CV_64FC1);
    for (xI = 0; xI <15; xI++)
    {
        x.at<double>(0,0) = xJ;
        x.at<double>(1,0) = xI;

        myTracker.predict();
        myTracker.correct(x);

        // print output

    }

    return 0;
}
