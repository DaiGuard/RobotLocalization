#ifndef __ROBOT_LOCALIZATION_H__
#define __ROBOT_LOCALIZATION_H__


#include <EmbedMatrix/Matrix.h>


namespace RobotLocalization
{
class EKF
{
    public:
        /**
         * @brief EKFコンストラクタ
         * 
         * @param x: 初期状態 [x, y, th, x', y', th']
         * @param P: 状態の分散 
         * @param Q: 状態方程式の分散
         * @param R: 観測方程式の分散
         */
        EKF(float* x, float* P, float* Q, float* R);
        /**
         * @brief EKFデストラクタ
         * 
         */
        ~EKF(){}

        /**
         * @brief 自己位置更新
         * 
         * @param x: 現在状態 [x, y, th, x', y', th']
         * @param y: センサー情報 [v, arx, ary, w]
         */
        void update(float* x, float* y, float dt, float* x_new);

    private:
        Embed::Matrix<float, 6, 6> mI;

        Embed::Matrix<float, 6, 1> mX;
        Embed::Matrix<float, 6, 1> mX_now;
        Embed::Matrix<float, 6, 1> mX_pre;

        Embed::Matrix<float, 6, 6> mP;
        Embed::Matrix<float, 6, 6> mP_pre;
        Embed::Matrix<float, 6, 6> mQ;
        Embed::Matrix<float, 3, 3> mR;

        Embed::Matrix<float, 6, 6> mf;
        Embed::Matrix<float, 6, 6> mF;
        Embed::Matrix<float, 6, 1> mU;

        Embed::Matrix<float, 3, 1> mY;
        Embed::Matrix<float, 3, 1> mZ;
        Embed::Matrix<float, 3, 6> mh;
        Embed::Matrix<float, 3, 6> mH;
        Embed::Matrix<float, 3, 3> mS;
        Embed::Matrix<float, 6, 3> mK;
};

}


#endif
