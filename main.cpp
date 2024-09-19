#include <iostream>
#include <opencv2/opencv.hpp>

#include "include/control.hpp"

using namespace cv;
using namespace std;

int main() {
  Planning::Control controller_;

  // 设置自机的起点状态
  Postion start_pos(10, 10, 0);
  Vel start_vel(0, 0, 0);
  State start_state(start_pos, start_vel, 0, 0);
  controller_.setStartState(start_state);

  // 目标物体的信息
  std::vector<State> target_states;
  for (int i = 0; i < 100; ++i) {
    State target_state;
    Postion target_pos(30 + i, 30 + 0.5 * i, 0);
    target_state.setPos(target_pos);
    target_states.emplace_back(target_state);
  }
  // double tmp = controller_.calDistance(start_pos,
  // target_states.front().pos()); std::cout << "distance = " << tmp <<
  // std::endl;

  // 创建一个黑色背景的图像
  Mat image = Mat::zeros(200, 200, CV_8UC3);
  namedWindow("Demo Image", WINDOW_AUTOSIZE);

  // 开始循环刷新图像
  for (int i = 0; i < target_states.size(); ++i) {
    // 每次刷新时清空图像
    image = Mat::zeros(200, 200, CV_8UC3);

    // 只绘制当前的目标物体
    circle(image,
           Point(target_states.at(i).pos().x, target_states.at(i).pos().y), 2,
           Scalar(255, 0, 0), -1);
    controller_.KinematicControl();
    std::cout
        << "--------------------------------------------------------------"
        << std::endl;

    // 绘制自机的位置
    circle(
        image,
        Point(controller_.egoState().pos().x, controller_.egoState().pos().y),
        1, Scalar(0, 0, 255), -1);

    // 绘制起点
    circle(image, Point(start_state.pos().x, start_state.pos().y), 1,
           Scalar(0, 255, 0), -1);

    Mat enlargedImage;
    resize(image, enlargedImage, Size(), 3.0, 3.0, INTER_LINEAR);

    // 显示图像
    imshow("Demo Image", enlargedImage);

    // 等待按键，每次按下按键时刷新一次图像
    int key = waitKey(0);
    if (key == 27) {  // 按下ESC键退出
      break;
    }
  }

  return 0;
}