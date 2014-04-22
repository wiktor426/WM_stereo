#ifndef PARAMETERSFORSGBM_H
#define PARAMETERSFORSGBM_H

class ParametersForSGBM
{


public:
    ParametersForSGBM();
    ~ParametersForSGBM();

private:


    int SADWindowSize;
    int numberOfDisparities;
    int preFilterCap;
    int minDisparity;
    int uniquenessRatio;
    int speckleWindowSize;
    int speckleRange;
    int disp12MaxDiff;
    int fullDP;
    int P1;
    int P2;
};

#endif // PARAMETERSFORSGBM_H
