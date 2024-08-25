/***
 *
 *
 *                                                    __----~~~~~~~~~~~------___
 *                                   .  .   ~~//====......          __--~ ~~
 *                   -.            \_|//     |||\\  ~~~~~~::::... /~
 *                ___-==_       _-~o~  \/    |||  \\            _/~~-
 *        __---~~~.==~||\=_    -_--~/_-~|-   |\\   \\        _/~
 *    _-~~     .=~    |  \\-_    '-~7  /-   /  ||    \      /
 *  .~       .~       |   \\ -_    /  /-   /   ||      \   /
 * /  ____  /         |     \\ ~-_/  /|- _/   .||       \ /
 * |~~    ~~|--~~~~--_ \     ~==-/   | \~--===~~        .\
 *          '         ~-|      /|    |-~\~~       __--~~
 *                      |-~~-_/ |    |   ~\_   _-~            /\
 *                           /  \     \__   \/~                \__
 *                       _--~ _/ | .-~~____--~-/                  ~~==.
 *                      ((->/~   '.|||' -_|    ~~-/ ,              . _||
 *                                 -_     ~\      ~~---l__i__i__i--~~_/
 *                                 _-~-__   ~)  \--______________--~~
 *                               //.-~~~-~_--~- |-------~~~~~~~~
 *                                      //.-~~~--\
 *                               神兽保佑
 *                              代码无BUG!
 */

#ifndef PLOT_H_
#define PLOT_H_

#include <vector>
#include <opencv2/core/core.hpp>

#define COLOR_BLUE          cv::Scalar(255,0,0)
#define COLOR_DEEPSKYBLUE   cv::Scalar(255,191,0)
#define COLOR_GREEN         cv::Scalar(0,255,0)
#define COLOR_RED           cv::Scalar(0,0,255)
#define COLOR_WRITE         cv::Scalar(255,255,255)
#define COLOR_BLACK         cv::Scalar(0,0,0)
#define COLOR_PINK          cv::Scalar(203, 192, 255)
#define COLOR_DEEPPINK      cv::Scalar(147, 20, 255)
#define COLOR_GOLD          cv::Scalar(0,215,255)

cv::Mat residual_plot(cv::InputOutputArray image,
                      cv::InputArrayOfArrays obs_points,
                      cv::InputArrayOfArrays reproj_points);

cv::Scalar random_color();

// draw arrow, like cv::line
void draw_arrow(cv::Mat& img, cv::Point start, cv::Point end, cv::Scalar color,
               int len = 16, int alpha = 30, int thickness = 1,
               int line_type = cv::LINE_8);

void draw_cross(cv::Mat& img, cv::Point point, cv::Scalar color, int size, int thickness = 1);

cv::Mat reprojection_error_plot(
    std::vector<std::vector<cv::Point_<float>>>& image_points,
    std::vector<std::vector<cv::Point_<float>>>& reproj_pts);

cv::Mat merge_image(int width, int height,
                    const std::vector<cv::Mat>& input_images,
                    bool resize_image = true);
#endif