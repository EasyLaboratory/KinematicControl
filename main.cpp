#include <iostream>
#include <opencv2/opencv.hpp>

#include "include/control.hpp"

using namespace cv;
using namespace std;

int main() {
  Planning::Control controller_;

  // 设置自机的起点状态
  Position start_pos(10, 10, 0);
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

  // 正选曲线
  // for (int i = 0; i < 300; ++i) {
  //   State target_state;
  //   Position target_pos(30 + i, 30 + 10 * sin(0.1 * i),
  //                       0);  // y = 30 + 10 * sin(0.1 * i)
  //   target_state.setPos(target_pos);
  //   target_states.emplace_back(target_state);
  // }

  // 变加速度初始条件
  float a = 0.2;   // 加速度
  float v0_x = 0;  // 初始速度x
  float v0_y = 1;  // 初始速度y
  float x0 = 30;   // 初始位置x
  float y0 = 30;   // 初始位置y

  // 生成变加速度轨迹
  for (int t = 0; t < 100; ++t) {
    State target_state;
    // x = 1/2 * a * t^2 + v0 * t + x0
    float x = 0.5 * a * t * t + v0_x * t + x0;
    float y = 0.5 * a * t * t + v0_y * t + y0;  // y方向同样加速变化

    Position target_pos(x, y, 0);
    target_state.setPos(target_pos);
    target_states.emplace_back(target_state);
  }

  // 创建一个黑色背景的图像
  Mat image = Mat::zeros(400, 400, CV_8UC3);
  namedWindow("Demo Image", WINDOW_AUTOSIZE);

  // 开始循环刷新图像
  for (int i = 0; i < target_states.size(); ++i) {
    // 每次刷新时清空图像
    image = Mat::zeros(400, 400, CV_8UC3);
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
    resize(image, enlargedImage, Size(), 3.0, 3.0, INTER_LINEAR);

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