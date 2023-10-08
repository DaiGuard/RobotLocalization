#include <RobotLocalization/EKF.h>
#include <cmath>

#include <Arduino.h>

using namespace RobotLocalization;
using namespace std;


EKF::EKF(float* x, float* P, float* Q, float* R)
{
    mI.setIndentity();

    mX.set(x);
    mX_now.set(x);
    mX_pre.setZeros();

    mP.setDiagonal(P);
    mP_pre.setZeros();

    mQ.setDiagonal(Q);
    mR.setDiagonal(R);

    mf.setIndentity();
    mF.setIndentity();
    mU.setZeros();

    mY.setZeros();
    mZ.setZeros();    

    mh.setZeros();
    mh[0][3] = 1.0f;
    mh[1][4] = 1.0f;
    mh[2][5] = 1.0f;
    mH = mh;
    mS.setZeros();
    mK.setZeros();
}



void ::EKF::update(float* x, float* y, float dt, float* x_new)
{
    float v     = y[0];
    float arx   = y[1];
    float ary   = y[2];
    float w     = y[3];
    float th    = x[2];
    float cth   = cos(th);
    float sth   = sin(th);

    mf[0][3] = dt;
    mf[1][4] = dt;
    mf[2][5] = dt;

    mU[0][0] = dt * dt / 2.0f * (arx * cth - ary * sth);
    mU[1][0] = dt * dt / 2.0f * (arx * sth + ary * cth);
    mU[3][0] = dt * (arx * cth - ary * sth);
    mU[4][0] = dt * (arx * sth + ary * cth);

    mF[0][2] = - dt * dt / 2.0f * (arx * sth + ary * cth);
    mF[0][3] =   dt;
    mF[1][2] =   dt * dt / 2.0f * (arx * cth - ary * sth);
    mF[1][4] =   dt;
    mF[2][5] =   dt;
    mF[3][2] = - dt * (arx * sth + ary * cth);
    mF[4][2] =   dt * (arx * cth - ary * sth);

    mX_now.set(x);

    // 予測ステップ
    mX_pre = mf * mX_now + mU;
    mP_pre = mF * mP * mF.T() + mQ;


    // 更新ステップ
    mZ[0][0] = v * cth;
    mZ[1][0] = v * sth;
    mZ[2][0] = w;

    mY = mZ - mh * mX_pre;
    mS = mH * mP_pre * mH.T() + mR;
    mK = mP_pre * mH.T() * mS.invert();

    mX = mX_pre + mK * mY;
    mP = (mI - mK * mH) * mP_pre;

    // 値出力
    memcpy(x_new, &mX[0][0], 6*sizeof(float));
}
