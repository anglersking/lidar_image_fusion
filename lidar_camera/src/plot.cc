#include "plot.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

#ifndef DEG2RAD
#define DEG2RAD(deg) (deg * 0.017453292519943295)
#endif

#ifndef RAD2DEG
#define RAD2DEG(rad) (rad * 57.29577951308232087721)
#endif

template<typename _Tp>
std::vector<_Tp> convertMat2Vector(const cv::Mat &mat)
{
	return (std::vector<_Tp>)(mat.reshape(1, 1));//通道数不变，按行转为一行
}

void drawPoints(cv::Mat pt1, cv::Mat pt2, cv::Mat& src) {
  char msg[512];
  int point_num = (int)std::max(pt1.checkVector(2, CV_32F), pt1.checkVector(2, CV_64F));
  CV_Assert(pt1.total() == (int)pt2.total());

  if (pt1.depth() == CV_32F) {
    for (int p = 0; p < point_num; p++) {
      cv::line(src, pt1.at<cv::Point2f>(p), pt2.at<cv::Point2f>(p), COLOR_BLUE, 2);
      cv::circle(src, pt1.at<cv::Point2f>(p), 1, COLOR_GREEN, 2);
      cv::circle(src, pt2.at<cv::Point2f>(p), 1, COLOR_RED, 2);
      float dis = cv::norm(pt1.at<cv::Point2f>(p) - pt2.at<cv::Point2f>(p));
      if (dis > 2.0) {
        sprintf(msg, "%0.1f", dis);
        cv::putText(src, msg, pt1.at<cv::Point2f>(p) + cv::Point2f(10, 0), 2, 0.3, COLOR_BLUE);
      }
    }
  } else if (pt1.depth() == CV_64F) {
    for (int p = 0; p < point_num; p++) {
      cv::line(src, pt1.at<cv::Point2d>(p), pt2.at<cv::Point2d>(p), COLOR_BLUE, 2);
      cv::circle(src, pt1.at<cv::Point2d>(p), 1, COLOR_GREEN, 2);
      cv::circle(src, pt2.at<cv::Point2d>(p), 1, COLOR_RED, 2);
      double dis = cv::norm(pt1.at<cv::Point2d>(p) - pt2.at<cv::Point2d>(p));
      if (dis > 2.0) {
        sprintf(msg, "%0.1f", dis);
        cv::putText(src, msg, pt1.at<cv::Point2d>(p) + cv::Point2d(10, 0), 2, 0.3, COLOR_BLUE);
      }
    }
  }
}

cv::Mat residual_plot(cv::InputOutputArray _image,
                      cv::InputArrayOfArrays _obs_points,
                      cv::InputArrayOfArrays _reproj_points) {
  cv::Mat image = _image.getMat();
  int nimages = (int)_obs_points.total();
  CV_Assert(nimages > 0 && nimages == (int)_reproj_points.total());

  std::vector<std::vector<cv::Point2f>> real_points;
  std::vector<std::vector<cv::Point2f>> res_points;
  cv::Mat opoints, rpoints;
  if (_reproj_points.isVector() && _obs_points.isVector()) {
    opoints = _obs_points.getMat(), rpoints = _reproj_points.getMat();
    drawPoints(opoints, rpoints, image);

  } else {
    for(int i = 0; i < nimages; i++ ){
      opoints = _obs_points.getMat(i), rpoints = _reproj_points.getMat(i);
      drawPoints(opoints, rpoints, image);
    }
  }

  return image;
}


cv::Scalar random_color() {
  static cv::RNG rng(0xFFFFFFFF);
  int icolor = (unsigned)rng;
  return cv::Scalar(icolor & 0xFF, (icolor >> 8) & 0xFF, (icolor >> 16) & 0xFF, 0.6);
}

// draw arrow, like cv::line
void draw_arrow(cv::Mat& img, cv::Point start, cv::Point end, cv::Scalar color,
                int len, int alpha, int thickness, int line_type) {
  double angle = atan2(start.y - end.y, start.x - end.x);
  cv::line(img, start, end, color, thickness, line_type);
  cv::Point2d arrow;
  arrow.x = end.x + len * cos(angle + DEG2RAD(alpha));
  arrow.y = end.y + len * sin(angle + DEG2RAD(alpha));
  cv::line(img, end, arrow, color, thickness, line_type);
  arrow.x = end.x + len * cos(angle - DEG2RAD(alpha));
  arrow.y = end.y + len * sin(angle - DEG2RAD(alpha));
  cv::line(img, end, arrow, color, thickness, line_type);
}

void draw_cross(cv::Mat& img, cv::Point point, cv::Scalar color, int size,
                int thickness) {
  //绘制横线
  cv::line(img, cvPoint(point.x - size / 2, point.y),
           cv::Point(point.x + size / 2, point.y), color, thickness, 8, 0);
  //绘制竖线
  cv::line(img, cvPoint(point.x, point.y - size / 2),
           cv::Point(point.x, point.y + size / 2), color, thickness, 8, 0);
}

#include <opencv2/opencv.hpp>
cv::Mat reprojection_error_plot(
    std::vector<std::vector<cv::Point_<float>>>& image_points,
    std::vector<std::vector<cv::Point_<float>>>& reproj_pts) {
  double total_rms = 0;
  int outlier_num = 0;
  int total_num = 0;
  
  // 求每个角点的残差
  std::vector<std::vector<cv::Point_<float>>> residual_pts;
  std::vector<float> residual_x;
  std::vector<float> residual_y;
  for (int i = 0; i < image_points.size(); i++) {
    std::vector<cv::Point_<float>> delta_pts;
    for (int j = 0; j < reproj_pts[i].size(); j++) {
      auto delta_pt = image_points[i][j] - reproj_pts[i][j];
      delta_pts.push_back(delta_pt);
      residual_x.push_back(delta_pt.x);
      residual_y.push_back(delta_pt.y);
      total_num++;
    }
    residual_pts.push_back(delta_pts);
  }

  // 均值,标准差
  cv::Mat mean;
  cv::Mat stddev;
  cv::meanStdDev(residual_x, mean, stddev);
  double stddev_x = stddev.at<double>(0);
  cv::meanStdDev(residual_y, mean, stddev);
  double stddev_y = stddev.at<double>(0);

  // 生成图像
  const int img_size = 2000;
  const int half_size = img_size / 2;
  const double max_value = 4.0 * std::max(stddev_x, stddev_y);
  const double scale = half_size / max_value;
  cv::Mat reproj_err_plot = cv::Mat(img_size, img_size, CV_8UC3, COLOR_WRITE);

  // 绘制散点图， 并统计直方图数据
  const int hist_cols = 40;
  std::vector<int> hist_x(2 * hist_cols, 0);
  std::vector<int> hist_y(2 * hist_cols, 0);

  const float offset = (float)half_size;
  for (int i = 0; i < residual_pts.size(); i++) {
    for (int j = 0; j < residual_pts[i].size(); j++) {
      cv::Point2f draw_pt(residual_pts[i][j].x * scale + offset,
                          -residual_pts[i][j].y * scale + offset);
      draw_cross(reproj_err_plot, draw_pt, random_color(), 20, 3);

      int col_x = (residual_pts[i][j].x / max_value) * hist_cols + hist_cols;
      int col_y = (residual_pts[i][j].y / max_value) * hist_cols + hist_cols;
      if (col_x < 0 || col_x >= 2 * hist_cols ||
          col_y < 0 || col_y >= 2 * hist_cols) {
        outlier_num++;
      } else {
        hist_x[col_x]++;
        hist_y[col_y]++;
      }

      total_rms += cv::norm(residual_pts[i][j]);
    }
  }

  if (total_num != 0) {
    total_rms /= total_num;

    // 直方图
    const int hist_width = img_size / hist_cols / 2;
    for (int i = 0; i < 2 * hist_cols; i++) {
      int density_x = hist_x[i] * (img_size * 2.5 / total_num);
      cv::rectangle(reproj_err_plot, 
                    cv::Point(i * hist_width, img_size),
                    cv::Point((i + 1) * hist_width, img_size - density_x),
                    random_color(), CV_FILLED);

      int density_y = hist_y[i] * (img_size * 2.5 / total_num); // 后项为缩放比例
      cv::rectangle(reproj_err_plot, 
                    cv::Point(0, i * hist_width),
                    cv::Point(density_y, (i + 1) * hist_width), 
                    random_color(), CV_FILLED);
    }
    cv::circle(reproj_err_plot, {half_size, half_size}, total_rms * scale, COLOR_DEEPPINK, 4);
  }

  // 一些信息
  cv::putText(reproj_err_plot, cv::format("total_num:%d", total_num), {20, 30}, 0, 1.2, COLOR_BLACK, 2);
  cv::putText(reproj_err_plot, cv::format("outlier:    %d", outlier_num), {20, 70}, 0, 1.2, COLOR_BLACK, 2);
  cv::putText(reproj_err_plot, cv::format("mean:    %.2f", total_rms), {20, 110}, 0, 1.2, COLOR_BLACK, 2);
  cv::putText(reproj_err_plot, cv::format("stddev_x: %.2f", stddev_x), {20, 150}, 0, 1.2, COLOR_BLACK, 2);
  cv::putText(reproj_err_plot, cv::format("stddev_y: %.2f", stddev_y), {20, 190}, 0, 1.2, COLOR_BLACK, 2);

  //建立坐标轴
  cv::putText(reproj_err_plot, cv::format("%.2f",max_value), {(int)(0.95 * img_size), half_size - 25}, 0, 1.2, COLOR_BLACK, 2);
  draw_arrow(reproj_err_plot, {10, half_size}, {img_size, half_size}, COLOR_BLACK);
  draw_arrow(reproj_err_plot, {half_size, img_size}, {half_size, 10}, COLOR_BLACK);

  return reproj_err_plot;
}

cv::Mat merge_image(int width, int height,
                    const std::vector<cv::Mat>& input_images,
                    bool resize_image) {

  int row = ceil(sqrt(input_images.size()));
  int col = ceil(input_images.size() / (float)row);

  int sub_win_width = width / col;    //每个子窗口的宽度
  int sub_win_height = height / row;  //每个子窗口的高度

  //检查图像的通道数，将单通道图像转化成三通道用来显示
  for (int i = 0; i < input_images.size(); i++) {
    if (input_images[i].channels() == 1) {
      cv::cvtColor(input_images[i], input_images[i], CV_GRAY2BGR);
    }
  }

  //是否缩放图像
  std::vector<cv::Mat> images;
  if (resize_image) {
    for (int i = 0; i < input_images.size(); i++) {
      float cw = (float)sub_win_width;        //显示区的宽
      float ch = (float)sub_win_height;       //显示区的高
      float pw = (float)input_images[i].cols;  //载入图像的宽
      float ph = (float)input_images[i].rows;  //载入图像的高
      float cs = cw / ch;                    //显示区的宽高比
      float ps = pw / ph;                    //载入图像的宽高比
      float scale = (cs > ps) ? (ch / ph) : (cw / pw);  //缩放比例因子
      int rw = (int)pw * scale;                         //缩放后图片的宽
      int rh = (int)ph * scale;                         //缩放后图片的高
      
      cv::Mat disp = cv::Mat(cv::Size(cw, ch), CV_8UC3, cv::Scalar(0,0,0));
      cv::Rect roi_of_roi = cv::Rect((cw - rw) / 2, (ch - rh) / 2, rw, rh);
      
      cv::Mat temp;
      cv::resize(input_images[i], temp, cv::Size(rw, rh));
      temp.copyTo(disp(roi_of_roi));
      images.push_back(disp);
    }
  } else {
    for (int i = 0; i < input_images.size(); i++) {
      images.push_back(input_images[i]);
    }
  }

  //显示窗口中显示的图像
  cv::Mat mer_img = cv::Mat(height, width, CV_8UC3, cv::Scalar(0,0,0));
  for (int i = 0; i < row; i++) {
    for (int j = 0; j < col; j++) {
      cv::Rect roi = cv::Rect(0 + j * sub_win_width, 0 + i * sub_win_height, sub_win_width, sub_win_height);
      if (i * col + j < images.size()) {
        cv::Mat temp;
        cv::resize(images[i * col + j], temp, cv::Size(sub_win_width, sub_win_height));
        temp.copyTo(mer_img(roi));
      }
    }
  }
  return mer_img;
}

// void density_plot(cv::Mat& src, std::vector<cv::Point2f>& points, int point_hold)
// {
//     if(src.rows==0 || src.cols==0 || points.empty() || point_hold<2)
//         return;

//     int half = point_hold/2;
//     for(auto p:points)
//     {
//         cv::Rect rect = cv::Rect(p.x-half, p.y-half, point_hold, point_hold);
//         rect.x = slamp(rect.x, 0, src.cols-2);
//         rect.y = slamp(rect.y, 0, src.rows-2);
//         rect.width = slamp(rect.width, 1, src.cols-rect.x);
//         rect.height = slamp(rect.height, 1, src.rows-rect.y);

//         src(rect).setTo(255);
//     }
// }