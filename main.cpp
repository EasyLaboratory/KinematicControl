#include <iostream>
#include <opencv2/opencv.hpp>

#include "include/control.hpp"

using namespace cv;
using namespace std;

int main() {
  // 创建一个黑色背景的图像
  int width = 500, height = 600;
  Mat image = Mat::zeros(width, height, CV_8UC3);
  namedWindow("Demo Image", WINDOW_AUTOSIZE);

  Planning::Control controller_;

  // 设置自机的起点状态
  Position start_pos(250, 300, 0);
  Vel start_vel(0, 0, 0);
  State start_state(start_pos, start_vel, 0, 0);
  controller_.setStartState(start_state);

  // 目标物体的信息
  std::vector<State> target_states;
  // 直线
  //  for (int i = 0; i < 100; ++i) {
  //    State target_state;
  //    Position target_pos(30 + i, 30 + 0.5 * i, 0);
  //    target_state.setPos(target_pos);
  //    target_states.emplace_back(target_state);
  //  }

  // 正弦曲线
  // for (int i = 0; i < 300; ++i) {
  //   State target_state;
  //   Position target_pos(30 + i, 30 + 10 * sin(0.1 * i),
  //                       0);  // y = 30 + 10 * sin(0.1 * i)
  //   target_state.setPos(target_pos);
  //   target_states.emplace_back(target_state);
  // }

  // 李萨如曲线参数
  double A = 200;    // 振幅X
  double B = 50;     // 振幅Y
  double a = 1;      // 频率X
  double b = 2;      // 频率Y
  double delta = 0;  // 相位差

  // 生成8字形轨迹
  for (double t = 0; t < 2 * CV_PI; t += 0.05) {
    // 通过李萨如曲线公式生成X和Y坐标
    State target_state;
    double x = width / 2 + A * std::sin(a * t + delta);
    double y = height / 2 + B * std::sin(b * t);

    Position target_pos(x, y, 0);
    target_state.setPos(target_pos);
    target_states.emplace_back(target_state);
  }

  // 打印8字形轨迹的起点位置
  std::cout << "8 start postion:"
            << "x = " << target_states[0].pos().x
            << ", y = " << target_states[0].pos().y << std::endl;

  // 开始循环刷新图像
  for (int i = 0; i < target_states.size(); ++i) {
    // 每次刷新时清空图像
    image = Mat::zeros(width, height, CV_8UC3);
    controller_.setTargetState(target_states.at(i));

    // 绘制当前目标点的轨迹曲线
    for (int j = i; j < target_states.size() - 1; ++j) {
      cv::Point p1(target_states.at(j).pos().x, target_states.at(j).pos().y);
      cv::Point p2(target_states.at(j + 1).pos().x,
                   target_states.at(j + 1).pos().y);
      cv::line(image, p1, p2, cv::Scalar(0, 255, 0), 1);  // 绘制绿色的曲线
    }

    // 绘制当前的目标物体
    circle(image,
           Point(target_states.at(i).pos().x, target_states.at(i).pos().y), 2,
           Scalar(255, 0, 0), -1);
    // 绘制自机的位置
    circle(
        image,
        Point(controller_.egoState().pos().x, controller_.egoState().pos().y),
        1, Scalar(0, 0, 255), -1);

    controller_.KinematicControl();
    std::cout << "-------------------------------------------------------------"
              << std::endl;

    // 绘制起点
    // circle(image, Point(start_state.pos().x, start_state.pos().y), 1,
    //        Scalar(0, 255, 0), -1);

    Mat enlargedImage;
    resize(image, enlargedImage, Size(), 1.0, 1.0, INTER_LINEAR);

    // 显示图像
    imshow("Demo Image", enlargedImage);

    // 等待按键，每次按下按键时刷新一次图像
    int key = waitKey(0);
    if (key == 27) {  // 按下ESC键退出
      break;
    }
    controller_.setStartState(controller_.egoState());
  }

  return 0;
}