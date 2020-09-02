#ifndef REGRESSION_H
#define REGRESSION_H

#include <BasicLinearAlgebra.h>
using namespace BLA;

class Regression {
  private:
    const static int len = 64;
    const static int coefficientLen = 3;
    
    float X[len];
    float Y[len];
    int counts[len];

    float minValue = 0;
    float maxValue = TAU;
    float groupSize = (maxValue - minValue) / len;

    float offset = 0.0;

    BLA::Matrix<len, coefficientLen> getA (float wf) {
      BLA::Matrix<len, coefficientLen> a;
      for (int i = 0; i < len; i++) {
        float x = X[i] / counts[i];
        
        if (isnan(x)) {
          x = (i * groupSize) - (groupSize / 2.0);
        }
        
        a(i, 0) = 1;
        a(i, 1) = f(x, wf);
        a(i, 2) = g(x, wf);
      }
      return a;
    }

    BLA::Matrix<len, 1> getY1() {
      BLA::Matrix<len, 1> y;
      for (int i = 0; i < len; i++) {
        float _y = Y[i] / counts[i];

        if (isnan(_y)) {
          _y = 0;
        }

        y(i, 0) = _y;
      }
      return y;
    }

    BLA::Matrix<len, 1> getY2() {
      BLA::Matrix<len, 1> y;
      for (int i = 0; i < len; i++) {
        float x = X[i] / counts[i];
        float _y = Y[i] / counts[i];
        
        if (isnan(x)) {
          x = (i * groupSize) - (groupSize / 2.0);
        }

        if (isnan(_y)) {
          y(i, 0) = 0;
        } else {
          y(i, 0) = _y - filter1(x);
        }
      }
      return y;
    }

    int getGroupIdx (float x) {
      return floor (x / groupSize);
    }

    BLA::Matrix<coefficientLen, 1> train1 () {
      //MULT(MULT(INVERSE(MULT(TRANSPOSE(A),A)),TRANSPOSE(A)),Y)
      BLA::Matrix<len, coefficientLen> a = getA(1.0);
      BLA::Matrix<len, 1> y = getY1();

      BLA::Matrix<coefficientLen, len> a_trans = ~a;
      
      BLA::Matrix<coefficientLen, coefficientLen> v2 = a_trans * a;
      BLA::Matrix<coefficientLen, coefficientLen> v3 = Invert(v2);
      
      BLA::Matrix<coefficientLen, len> v4 = v3 * a_trans;
      return v4 * y;
    }

    BLA::Matrix<coefficientLen, 1> train2 () {
      //MULT(MULT(INVERSE(MULT(TRANSPOSE(A),A)),TRANSPOSE(A)),Y)
      BLA::Matrix<len, coefficientLen> a = getA(10.0);
      BLA::Matrix<len, 1> y = getY2();

      BLA::Matrix<coefficientLen, len> a_trans = ~a;
      
      BLA::Matrix<coefficientLen, coefficientLen> v2 = a_trans * a;
      BLA::Matrix<coefficientLen, coefficientLen> v3 = Invert(v2);
      
      BLA::Matrix<coefficientLen, len> v4 = v3 * a_trans;
      return v4 * y;
    }

    float f (float x, float wf) {
      return cos(wf * x);
    }

    float g (float x, float wf) {
      return sin(wf * x);
    }

  public:
    float coefficients1[coefficientLen];
    float coefficients2[coefficientLen];
    
    Regression () {
    }
    
    void add (float x, float y) {
      int i = getGroupIdx(x);
      X[i] += x;
      Y[i] += y;
      counts[i]++;
    }

    void train() {
      offset = 0;
      
      BLA::Matrix<coefficientLen, 1> v1 = train1();
      for (int i = 0; i < coefficientLen; i++) {
        coefficients1[i] = v1(i, 0);
      }
      
      BLA::Matrix<coefficientLen, 1> v2 = train2();
      for (int i = 0; i < coefficientLen; i++) {
        coefficients2[i] = v2(i, 0);
      }
    }

    float filter1 (float x) {
      float _x = formatAngle(x + offset);
      return coefficients1[0] + coefficients1[1] * f(_x, 1.0) + coefficients1[2] * g(_x, 1.0);
    }

    float filter2 (float x) {
      float _x = formatAngle(x + offset);
      return coefficients2[0] + coefficients2[1] * f(_x, 10.0) + coefficients2[2] * g(_x, 10.0);
    }

    void reset () {
      coefficients1[0] = 0;
      coefficients1[1] = 0;
      coefficients1[2] = 0;
      coefficients2[0] = 0;
      coefficients2[1] = 0;
      coefficients2[2] = 0;
    }

    void addOffset (float o) {
      offset = formatAngle(offset + o);
    }

    void setOffset (float o) {
      offset = o;
    }

    float getOffset() {
      return offset;
    }

    float formatAngle (float x) {
      if (x < -TAU) {
        x += TAU;
        return formatAngle(x);
      } else if (x > TAU) {
        x -= TAU;
        return formatAngle(x);
      } else {
        return x;
      }
    }
    
    
};

#endif
