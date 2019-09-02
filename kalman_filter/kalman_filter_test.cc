#include <opencv2/opencv.hpp>
#include <iostream>
#include "kalman_filter.h"
//#include <stdio.h>
using namespace std;
using namespace cv;
Mat img(500, 500, CV_8UC3);
//计算相对窗口的坐标值，因为坐标原点在左上角，所以sin前有个负号
static inline Point calcPoint(Point2f center, double R, double angle)
{
  return center + Point2f((float)cos(angle), (float)-sin(angle))*(float)R;
}
void drawCross(Point center, Scalar color, int d)
{
  line(img, Point(center.x - d, center.y - d),
       Point(center.x + d, center.y + d), color, 1, CV_AA, 0);
  line(img, Point(center.x + d, center.y - d),
       Point(center.x - d, center.y + d), color, 1, CV_AA, 0);
}
static void help()
{
  printf("\nExamle of c calls to OpenCV's Kalman filter.\n"
         "   Tracking of rotating point.\n"
         "   Rotation speed is constant.\n"
         "   Both state and measurements vectors are 1D (a point angle),\n"
         "   Measurement is the real point angle + gaussian noise.\n"
         "   The real and the estimated points are connected with yellow line segment,\n"
         "   the real and the measured points are connected with red line segment.\n"
         "   (if Kalman filter works correctly,\n"
         "    the yellow segment should be shorter than the red one).\n"
         "\n"
         "   Pressing any key (except ESC) will reset the tracking with a different speed.\n"
         "   Pressing ESC will stop the program.\n"
  );
}
//打印一个Mat矩阵
void PrintMat(Mat A)
{
  for(int i=0;i<A.rows;i++)
  {
    for(int j=0;j<A.cols;j++)
      cout<<A.at<float>(i,j)<<' ';
    cout<<endl;
  }
  cout<<endl;
}

int main(int, char**)
{
  help();

  KalmanFilter KF;                                     //创建卡尔曼滤波器对象KF
  char code = (char)-1;
  Scalar color;
  int d=5;

  for (;;)
  {
    //1.初始化
    KF.Init(Eigen::VectorXd(),Eigen::MatrixXd());

    for (;;)
    {
      Point2f center(img.cols*0.5f, img.rows*0.5f);                 //center图像中心点
      float R = img.cols / 3.f;                                     //半径
      double stateAngle = state.at<float>(0);                       //跟踪点角度
      std::cout << "stateAngle:" << state << std::endl;
      Point statePt = calcPoint(center, R, stateAngle);             //跟踪点坐标statePt

      //2. 预测
      KF.predict();                                           //计算预测值，返回x'
      double predictAngle = KF.getGlobal_states()[0];            //预测点的角度
      Point predictPt = calcPoint(center, R, predictAngle);   //预测点坐标predictPt


      // generate measurement
      randn(measurementNoise, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));     //给measurement赋值N(0,R)的随机值
      measurement = KF.measurementMatrix*state + measurementNoise;  //z = H*x + v;
      std::cout << "measurementNoise" << measurementNoise << std::endl;
      std::cout << "measurementMatrix" << KF.measurementMatrix << std::endl;
      std::cout << "state" << state << std::endl;
      std::cout << "measurement" << measurement << std::endl;

      double measAngle = measurement.at<float>(0);
      Point measPt = calcPoint(center, R, measAngle);

      // plot points
      img = Scalar::all(0);
      drawCross(statePt, Scalar(255, 255, 255), 3);
      drawCross(measPt, Scalar(0, 0, 255), 3);
      drawCross(predictPt, Scalar(0, 255, 0), 3);
      line(img, statePt, measPt, Scalar(0, 0, 255), 3, CV_AA, 0);
      line(img, statePt, predictPt, Scalar(0, 255, 255), 3, CV_AA, 0);


      //3.更新
      //调用kalman这个类的correct方法得到加入观察值校正后的状态变量值矩阵
      if (theRNG().uniform(0, 4) != 0)
       //todo 计算出测试的noise
        KF.correct(Eigen::VectorXd(measurement.at<float>(0),(measurement.at<float>(1)),Eigen::MatrixXd());

      //更新实际值：不加噪声的话就是匀速圆周运动，加了点噪声类似匀速圆周运动，因为噪声的原因，运动方向可能会改变
      randn(processNoise, Scalar::all(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));   //vk
      std::cout << "processNoiseCov:" << KF.processNoiseCov << std::endl;
      std::cout << "processNoise:" << processNoise << std::endl;

      imshow("Kalman", img);
      code = (char)waitKey(100);

      if (code > 0)
        break;
    }
    if (code == 27 || code == 'q' || code == 'Q')
      break;
  }

  return 0;
}