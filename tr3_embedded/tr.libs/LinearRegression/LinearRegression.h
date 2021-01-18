#ifndef LinearRegression_h
#define LinearRegression_h

#include "Arduino.h"

class LinearRegression {
    public:
        LinearRegression();
        LinearRegression(double min, double max);
        void learn(double x, double y);
        double calculate(double x);
        double correlation();
        void getValues(double values[]);
        void reset();
    private:
        double meanX = 0;
        double meanX2 = 0; //mean x²
        double varX = 0;
        double meanY = 0;
        double meanY2 = 0; //mean y²
        double varY = 0;
        double meanXY = 0; //mean x*y
        double covarXY = 0;
        double n = 0;
        bool limits = false; 
        double minX = 0;
        double maxX = 0;
        // m*x + b = y;
        double m = 0;
        double b = 0;
};

#endif