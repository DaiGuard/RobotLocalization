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
             * @param x: [x, y, th, x', y', th']
             * @param P: 状態の分散 
             * @param Q: 状態方程式の分散
             * @param R: 観測方程式の分散
             */
            EKF(float* x, float* P, float* Q, float* R);
            ~EKF();

            /**
             * @brief 自己位置更新
             * 
             * @param x: [x, y, th, x', y', th']
             * @param y: [x', y', th']
             */
            void update(float* x, float* y);

        private:
            Embed::Matrix<float, 6, 1> _x;
            Embed::Matrix<float, 6, 6> _P;
            Embed::Matrix<float, 6, 6> _Q;
            Embed::Matrix<float, 6, 6> _R;
            Embed::Matrix<float, 3, 1> _y;
    };

};


#endif