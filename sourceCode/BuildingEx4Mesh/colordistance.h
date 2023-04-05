/********************************************
 * Class: ColorDistance
 * Author: Dayu
 * Site: WHU
 * Date: 2020321
 * email: dayuyu@whu.edu.cn
********************************************/
#ifndef COLORDISTANCE_H
#define COLORDISTANCE_H

#include <algorithm>

#include "codelibrary/visualization/color/rgb32_color.h"

#include "yutils/ColorUtils.h"

namespace cd  {

/**
 * @brief getWeightedEulerDistance4RGB
 * @param rgb1
 * @param rgb2
 * @return 线性RGB色彩空间内，两个颜色间的加权欧式距离值
 * One of the better low-cost approximations, 
 * sometimes called "redmean", combines the two cases smoothly
 */
inline float getWeightedEulerDistance4RGB(
    const unsigned char rgb1[3], const unsigned char rgb2[3])
{
    int rmean = ( (int)rgb1[0] + (int)rgb2[0] ) / 2;
    int r = (int)rgb1[0] - (int)rgb2[0];
    int g = (int)rgb1[1] - (int)rgb2[1];
    int b = (int)rgb1[2] - (int)rgb2[2];
    return (float)sqrt(
        (((512 + rmean) * r * r) >> 8) + 4 * g * g + (((767 - rmean) * b * b) >> 8)
    );
};

/**
 * @brief getWeightedEulerDistance4RGB
 * @param rgb1
 * @param rgb2
 * @return 线性RGB色彩空间内，两个颜色间的加权欧式距离值
 */
inline float getWeightedEulerDistance4RGB(
    const cl::RGB32Color &rgb1, const cl::RGB32Color &rgb2)
{
    unsigned char rgb11[3] = {rgb1.red(), rgb1.green(), rgb1.blue()};
    unsigned char rgb22[3] = {rgb2.red(), rgb2.green(), rgb2.blue()};

    return getWeightedEulerDistance4RGB(rgb11, rgb22);
};

/**
 * @brief getEulerDistance4RGB
 * @param rgb1
 * @param rgb2
 * @return 线性RGB色彩空间内，两个颜色间的欧式距离值
 */
inline float getEulerDistance4RGB(
    const cl::RGB32Color& rgb1, const cl::RGB32Color& rgb2)
{
    unsigned char rgb11[3] = { rgb1.red(), rgb1.green(), rgb1.blue() };
    unsigned char rgb22[3] = { rgb2.red(), rgb2.green(), rgb2.blue() };

    int r = (int)rgb11[0] - (int)rgb22[0];
    int g = (int)rgb11[1] - (int)rgb22[1];
    int b = (int)rgb11[2] - (int)rgb22[2];

    return sqrt(r*r+g*g+b*b);
};

/**
 * @brief getDelta-E_CIE76
 * @param rgb1
 * @param rgb2
 * @return 线性RGB色彩空间内，两个颜色间的欧式距离值
 */
inline float getDeltaE_CIE76(
    const cl::RGB32Color& rgb1, const cl::RGB32Color& rgb2)
{
    ColorUtils::rgbColor c1(static_cast<unsigned int>(rgb1.red()), rgb1.green(), rgb1.blue());  // unsigned int constructor
    ColorUtils::rgbColor c2(static_cast<unsigned int>(rgb2.red()), rgb2.green(), rgb2.blue());  // unsigned int constructor

    float deltaE = ColorUtils::getColorDeltaE( c1,  c2);

    return deltaE;
};

};

#endif // COLORDISTANCE_H
