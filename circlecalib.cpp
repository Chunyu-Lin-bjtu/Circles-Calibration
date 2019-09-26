#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <string>


using namespace std;
using namespace cv;


cv::Mat colorImg,grayImg;//用于存取原图，以及存取转化后的灰度图
int boardWidth=4;//标定板上的行数
int boardHeight=11;//标定板上的列数
cv::vector < cv::Point2f > circleGridCenters;//用于记录检测到的圆的中心点（一张图片）
cv::vector < cv::Point2f > circleGridCenter1;//记录第一张图上中心点的相机坐标
cv::vector < cv::Point3f > obj1;//记录第一张图片上中心点的世界坐标
float circleDistance=0.025;//圆点之间的距离????????????不准确

cv::vector < cv::vector <cv::Point3f> > objectPoints;//圆点中心的世界坐标,每张图片上的点构成内层vector，所有图片构成外层vector
cv::vector < cv::vector <cv::Point2f> > imagePoints;//圆点中心的图像坐标,,每张图片上的点构成内层vector，所有图片构成外层vector
//测试图片testcircle2.jpg width=rows=1080,height=cols=2277
//std::string imgFile="/home/yaoyishen/stereo-calibration/circle-calibration/testcircle2.jpg";//测试路径
int main()
{
    //设置圆点检测器SimpleBlobDector
    cv::SimpleBlobDetector::Params params;
    params.minThreshold=10;
    params.maxThreshold=200;

    params.filterByArea=true;
    params.filterByArea=1500;

    params.filterByCircularity=true;
    params.minCircularity=0.1;

    params.filterByConvexity=true;
    params.minConvexity=0.87;

    params.filterByInertia=true;
    params.minInertiaRatio=0.01;
    cv::Ptr<cv::SimpleBlobDetector> detector = new SimpleBlobDetector(params);



    for(int i=0;i<=20;i++){
        char imgFile[100];
        sprintf(imgFile, "%s%d%s","/home/yaoyishen/stereo-calibration/calib_imgs/circleCalib/left/",i,".jpg");
        colorImg=cv::imread(imgFile,CV_LOAD_IMAGE_COLOR);
        cv::resize(colorImg,colorImg,cv::Size(1000,1000),0,0,1);//产生误差
        cv::cvtColor(colorImg,grayImg,CV_BGR2GRAY);
        
        bool foundCircles=false;
        cv::Size boardSize=cv::Size(boardWidth,boardHeight);
        foundCircles=cv::findCirclesGrid(grayImg,boardSize,circleGridCenters,
                                        cv::CALIB_CB_ASYMMETRIC_GRID,detector);
        if(foundCircles){
            //提取亚像素坐标
            cv::cornerSubPix(grayImg,circleGridCenters,cv::Size(5,5),cv::Size(-1,-1),
                            TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            //标记检测到的点
            //cv::drawChessboardCorners(grayImg,boardSize,circleGridCenters,foundCircles);
            cv::drawChessboardCorners(colorImg,boardSize,circleGridCenters,foundCircles);
        }
        cv::imshow("检测到的点",colorImg);
        cv::waitKey(0);
        
        cv::vector <cv::Point3f> objs;//一张图片上的世界坐标
        for(int m=0;m<boardHeight;m++){
            for(int n=0;n<boardWidth;n++){
                //共m竖列，奇数列和偶数列圆点排列不同
                if(m%2==0){
                    objs.push_back(Point3f((float)n*circleDistance,(float)m*0.5*circleDistance,0));
                }
                if(m%2!=0){
                    objs.push_back(Point3f((float)(2*n+1)*0.5*circleDistance,(float)m*0.5*circleDistance,0));
                }   
            }
        }
        
        if(foundCircles){
            //记录第一张图片的相机坐标和世界坐标用来反推出旋转平移矩阵
            if(i==1){
                circleGridCenter1=circleGridCenters;
                obj1=objs;
            }
            //记录所有图像的坐标用于求内参
            cout <<"检测到图像"<<i<<".jpg的圆点中心"<<endl;
            imagePoints.push_back(circleGridCenters);
            objectPoints.push_back(objs);
        }
        //std::cout << "二维图像坐标" << imagePoints[0] << std::endl;
        //std::cout << "三维世界坐标" << objectPoints[0] << std::endl;
    }


    
    cv::Mat cameraMatrix;//相机内参矩阵（最后输出用）
    cv::Mat distortMatrix;//相机畸变矩阵（最后输出用）
    cv::Mat rotationMatrix;//标定板到相机的旋转矩阵（最后输出用）
    cv::Mat translationMatrix;//标定板到相机的平移矩阵(最后输出用)

    cv::vector <cv::Mat> camRVec;//每幅图像到相机的旋转矩阵
    cv::vector <cv::Mat> camTVec;//每幅图像到相机的平移矩阵
    //求内参
    cv::calibrateCamera(objectPoints,imagePoints,colorImg.size(),cameraMatrix,distortMatrix,camRVec,camTVec,0,cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS),
                         30, DBL_EPSILON));//相机内参
    std::cout << "calibrateCamera已通过" << std::endl;
    
    //求外参
    cv::Mat rVec;//临时旋转向量；
    cv::solvePnP(obj1,circleGridCenter1,cameraMatrix,distortMatrix,rVec,translationMatrix);//使用某一张图片上的点和内参来求出外参
    std::cout << "solvePnP已通过" << std::endl;
    cv::Rodrigues(rVec,rotationMatrix);
    std::cout << "Rodrigues已通过" << std::endl;

    
    std::string calibFile="/home/yaoyishen/stereo-calibration/circle-calibration/build/calibration.xml";
    cv::FileStorage fs(calibFile, cv::FileStorage::WRITE);

    fs << "Circle_Calibration" <<"{:";

    fs << "Camera_Matrix" << cameraMatrix << "Distrotion_Matrix" << distortMatrix
       << "Rotation_Matrix" << rotationMatrix << "Translation_Matrix" << translationMatrix;
    
    fs << "}";
    fs.release();
    std::cout << "标定文件已生成" << std::endl;
    return 0;
}
