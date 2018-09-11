
#include <iostream>
#include <stdio.h>
#include <string.h>
//#include <sys/time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/gpu/device/utility.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include "car_lib.h"

#define PI 3.1415926
#define WIDTH 320
#define HEIGHT 180

using namespace std;
using namespace cv;

extern "C" {

/**
  * @brief  To load image file to the buffer.
  * @param  file: pointer for load image file in local path
             outBuf: destination buffer pointer to load
             nw : width value of the destination buffer
             nh : height value of the destination buffer
  * @retval none
  */
int lkas(int upP_x, int leftP_x, int rightP_x, int flag)
{
    if ((leftP_x > -70) && flag != 2)
    {
        upP_x += 2 * (leftP_x + 70);
    }
    if (rightP_x < 390 && flag != 1)
    {
        upP_x += 2 * (rightP_x - 390);
    }

    return upP_x;
}

void OpenCV_load_file(char* file, unsigned char* outBuf, int nw, int nh)
{
    Mat srcRGB;
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

    srcRGB = imread(file, CV_LOAD_IMAGE_COLOR); // rgb
    //cvtColor(srcRGB, srcRGB, CV_RGB2BGR);

    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

/**
  * @brief  To convert format from BGR to RGB.
  * @param  inBuf: buffer pointer of BGR image
             w: width value of the buffers
             h : height value of the buffers
             outBuf : buffer pointer of RGB image
  * @retval none
  */
void OpenCV_Bgr2RgbConvert(unsigned char* inBuf, int w, int h, unsigned char* outBuf)
{
    Mat srcRGB(h, w, CV_8UC3, inBuf);
    Mat dstRGB(h, w, CV_8UC3, outBuf);

    cvtColor(srcRGB, dstRGB, CV_BGR2RGB);
}

/**
  * @brief  Detect faces on loaded image and draw circles on the faces of the loaded image.
  * @param  file: pointer for load image file in local path
             outBuf: buffer pointer to draw circles on the detected faces
             nw : width value of the destination buffer
             nh : height value of the destination buffer
  * @retval none
  */
void OpenCV_face_detection(char* file, unsigned char* outBuf, int nw, int nh)
{
    Mat srcRGB = imread(file, CV_LOAD_IMAGE_COLOR);
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);
    
    // Load Face cascade (.xml file)
    CascadeClassifier face_cascade;
    face_cascade.load( "haarcascade_frontalface_alt.xml" );
 
    // Detect faces
    std::vector<Rect> faces;
    face_cascade.detectMultiScale( srcRGB, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
    
    // Draw circles on the detected faces
    for( int i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
        ellipse( srcRGB, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
    }
 
    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

/**
  * @brief  To bind two images on destination buffer.
  * @param  file1: file path of first image to bind
             file2: file path of second image to bind
             outBuf : destination buffer pointer to bind
             nw : width value of the destination buffer
             nh : height value of the destination buffer
  * @retval none
  */
void OpenCV_binding_image(char* file1, char* file2, unsigned char* outBuf, int nw, int nh)
{
    Mat srcRGB = imread(file1, CV_LOAD_IMAGE_COLOR);
    Mat srcRGB2 = imread(file2, CV_LOAD_IMAGE_COLOR);
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

    cv::resize(srcRGB2, srcRGB2, cv::Size(srcRGB2.cols/1.5, srcRGB2.rows/1.5));
    cv::Point location = cv::Point(280, 220);
    for (int y = std::max(location.y, 0); y < srcRGB.rows; ++y)
    {
        int fY = y - location.y;
        if (fY >= srcRGB2.rows)
            break;
        
        for (int x = std::max(location.x, 0); x < srcRGB.cols; ++x)
        {
            int fX = x - location.x;
            if (fX >= srcRGB2.cols)
            break;
            
            double opacity = ((double)srcRGB2.data[fY * srcRGB2.step + fX * srcRGB2.channels() + 3]) / 255.;
            for (int c = 0; opacity > 0 && c < srcRGB.channels(); ++c)
            {
                unsigned char overlayPx = srcRGB2.data[fY * srcRGB2.step + fX * srcRGB2.channels() + c];
                unsigned char srcPx = srcRGB.data[y * srcRGB.step + x * srcRGB.channels() + c];
                srcRGB.data[y * srcRGB.step + srcRGB.channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
            }
        }
    }
 
    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

/**
  * @brief  Apply canny edge algorithm and draw it on destination buffer.
  * @param  file: pointer for load image file in local path
             outBuf: destination buffer pointer to apply canny edge
             nw : width value of destination buffer
             nh : height value of destination buffer
  * @retval none
  */
void OpenCV_canny_edge_image(char* file, unsigned char* outBuf, int nw, int nh)
{
    Mat srcRGB = imread(file, CV_LOAD_IMAGE_COLOR);
    Mat srcGRAY;
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

    cvtColor(srcRGB, srcGRAY, CV_BGR2GRAY);
     // �ɴ� �˰��� ����
    cv::Mat contours;
    cv::Canny(srcGRAY, // �׷��̷��� ����
        contours, // ��� �ܰ���
        125,  // ���� ��谪
        350);  // ���� ��谪

    // ������ ȭ�ҷ� �ܰ����� ǥ���ϹǷ� ��� ���� ����
    //cv::Mat contoursInv; // ���� ����
    //cv::threshold(contours, contoursInv, 128, 255, cv::THRESH_BINARY_INV);
    // ��� ���� 128���� ������ 255�� �ǵ��� ����
 
    cvtColor(contours, contours, CV_GRAY2BGR);
    
    cv::resize(contours, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

/**
  * @brief  Detect the hough and draw hough on destination buffer.
  * @param  srcBuf: source pointer to hough transform
             iw: width value of source buffer
             ih : height value of source buffer
             outBuf : destination pointer to hough transform
             nw : width value of destination buffer
             nh : height value of destination buffer
  * @retval none
  */
int OpenCV_hough_transform(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, float be_angle, int threshold, int Low_T, int HIGH_T, int roi_th, int ignoreL)
{
    int left_cnt = 0, right_cnt = 0, stop_cnt = 0;
    float rho1 = 0, rho2 = 0, theta1 = 0, theta2 = 0;
    float angle = 0;
    int length = 800;
    int offset = 20;
    float gradientL, gradientR;
    float interceptL, interceptR;

    Point pt1, pt2, pt3, pt4;
    Point banishP, leftP1, leftP2, rightP1, rightP2, upPt, downPt, tempPt;

    upPt.y = -11;
    downPt.x = WIDTH/2;
    downPt.y = HEIGHT - 10;

    Scalar lineColor = cv::Scalar(255,255,255);
    Scalar lineColor1 = cv::Scalar(255,0,0);
    Scalar lineColor2 = cv::Scalar(0,0,255);
    Scalar lineColor3 = cv::Scalar(0,0,0);
    
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);
    Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    Mat resRGB(ih, iw, CV_8UC3);
    //cvtColor(srcRGB, srcRGB, CV_BGR2BGRA);

    // ĳ�� �˰��� ����
    cv::Mat contours, srcGRAY;
    
/*
    if (ignoreL != 0)
    {
        cvtColor(srcRGB, srcGRAY, CV_BGR2GRAY);

        for (int ii = 0; ii<180; ii++)
        {   
            for (int jj = 0; jj<320; jj++)
            {
                if (ii >= 0 && ii < roi_th)
                {
                    srcGRAY.at<uchar>(ii,jj) = 0;
                }
                else
                {
                    if (srcGRAY.at<uchar>(ii,jj) > 100)
                    {
                        srcGRAY.at<uchar>(ii,jj) = 255;
                    }
                    else srcGRAY.at<uchar>(ii,jj) = 0;
                }
            }
        }
        //cv::Canny(srcGRAY, contours, Low_T, HIGH_T);     //125, 300

        cv::Canny(srcGRAY, // �׷��̷��� ����
            contours, // ��� �ܰ���
            125,  // ���� ��谪
            350);  // ���� ��谪
    }
*/

    cv::Canny(srcRGB, contours, Low_T, HIGH_T);     //125, 300

    for (int ii = 0; ii<roi_th; ii++)       //70
    {
        for (int jj = 0; jj < 320; jj++)
        {
            contours.at<Vec3b>(ii,jj)[0] = 0;
            contours.at<Vec3b>(ii,jj)[1] = 0;
            contours.at<Vec3b>(ii,jj)[2] = 0;
        }
    }

    // �� ���� ���� ���� ��ȯ
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(contours, lines, 1, PI/180, // �ܰ躰 ũ�� (1�� ��/180���� �ܰ躰�� ������ ��� ������ �������� ���� ã��)
        threshold);  // ��ǥ(vote) �ִ� ����

    // �� �׸���
    cv::Mat result(contours.rows, contours.cols, CV_8UC3, lineColor);
    //printf("Lines detected: %d\n", lines.size());

    // �� ���͸� �ݺ��� �� �׸���
    std::vector<cv::Vec2f>::const_iterator it= lines.begin();
    while (it!=lines.end()) 
    {
        float rho = (*it)[0];   // ù ��° ��Ҵ� rho �Ÿ�
        float theta = (*it)[1]; // �� ��° ��Ҵ� ��Ÿ ����
        
        if (theta < 1.41 && theta >= 0 && ignoreL != 1) // ���� ��
//        if (theta < 1.41 && theta >= 0) // ���� ��
        {
            //cv::Point pt1(rho/cos(theta), 0); // ù �࿡�� �ش� ���� ������   
            //cv::Point pt2((rho-result.rows*sin(theta))/cos(theta), result.rows);
            // ������ �࿡�� �ش� ���� ������

            rho1 += rho;
            theta1 += theta;

            left_cnt++;
            //cv::line(srcRGB, pt1, pt2, lineColor1, 1); // �Ͼ� ������ �׸���

        } 
        else if (theta<3.14 && theta>=1.77 && ignoreL != 2)
//        else if (theta<3.14 && theta>=1.77)
        {
            rho2 += rho;
            theta2 += theta;
            
            right_cnt++;
            //cv::Point pt3(rho/cos(theta), 0); // ù �࿡�� �ش� ���� ������   
            //cv::Point pt4((rho-result.rows*sin(theta))/cos(theta), result.rows);
            //cv::line(srcRGB, pt3, pt4, lineColor2, 1); // �Ͼ� ������ �׸���
        }
        //printf("line: rho=%f, theta=%f\n", rho, theta);
/*
        if (left_cnt == 0 && ignoreL == 2)
        {
            stop_cnt = 1;
            angle = 1;
            //left turn
        }
        else if (right_cnt == 0 && ignoreL == 1)
        {
            stop_cnt = 1;
            angle = 1;
            //right turn
        }
*/
        ++it;
    }

    if (left_cnt >= 3)
    {
        rho1 = rho1 / left_cnt;
        theta1 = theta1 / left_cnt;

        double a1 = cos(theta1), b1 = sin(theta1);
        double x1 = a1 * rho1, y1 = b1 * rho1;

        pt1.x = cvRound(x1 - length * (-b1));
        pt1.y = cvRound(y1 - length * (a1));
        pt2.x = cvRound(x1 + length * (-b1));
        pt2.y = cvRound(y1 + length * (a1));
    }
    if (right_cnt >= 3)
    {
        rho2 = rho2 / right_cnt;
        theta2 = theta2 / right_cnt;

        double a2 = cos(theta2), b2 = sin(theta2);
        double x2 = a2 * rho2, y2 = b2 * rho2;

        pt3.x = cvRound(x2 - length * (-b2));
        pt3.y = cvRound(y2 - length * (a2));
        pt4.x = cvRound(x2 + length * (-b2));
        pt4.y = cvRound(y2 + length * (a2));
    }

    if (pt1.x != 0 && pt3.x != 0)
    {
        // banish Point detection  

        // leftLine : first linear equation
        gradientL = (float)(pt2.y - pt1.y) / (float)(pt2.x - pt1.x);		// gradient 
        interceptL = pt2.y - gradientL * pt2.x;							// y-intercept

        // rightLine : first linear equation
        gradientR = (float)(pt4.y - pt3.y) / (float)(pt4.x - pt3.x);		// gradient
        interceptR = pt4.y - gradientR * pt4.x;							// y-intercept

        // banishPoint : nodePoint of two equation
        banishP.x = (int)((interceptR - interceptL) / (gradientL - gradientR));
        banishP.y = (int)(gradientL * banishP.x + interceptL);

        if (banishP.y > 80)
        {
            if (be_angle >= 0)
            {
                pt3.x = 0;
                pt3.y = 0;
                pt4.x = 0;
                pt4.y = 0;
            }
            else if (be_angle < 0)
            {
                pt1.x = 0;
                pt1.y = 0;
                pt2.x = 0;
                pt2.y = 0;
            }
        }
    }

    if (pt1.x != 0 && pt3.x != 0)
    {
    /*
        // banish Point detection 

        // leftLine : first linear equation
        float gradientL = (float)(pt2.y - pt1.y) / (float)(pt2.x - pt1.x);		// gradient 
        float interceptL = pt2.y - gradientL * pt2.x;							// y-intercept

        // rightLine : first linear equation
        float gradientR = (float)(pt4.y - pt3.y) / (float)(pt4.x - pt3.x);		// gradient
        float interceptR = pt4.y - gradientR * pt4.x;							// y-intercept

        // banishPoint : nodePoint of two equation
        banishP.x = (int)((interceptR - interceptL) / (gradientL - gradientR));
        banishP.y = (int)(gradientL * banishP.x + interceptL);
    */

        float c1 = pt2.y - gradientL * pt2.x;
        leftP2.y = HEIGHT;
        leftP2.x = (leftP2.y - c1) / gradientL;

        float c2 = pt3.y - gradientR * pt3.x;
        rightP2.y = HEIGHT;
        rightP2.x = (rightP2.y - c2) / gradientR;

        banishP.x = lkas(banishP.x, leftP2.x, rightP2.x, 3);

		tempPt.y = (int)(HEIGHT / 2 + offset);
		tempPt.x = (int)(((float)(banishP.x-downPt.x)/(float)(banishP.y - downPt.y))*(float)(tempPt.y - downPt.y)+(float)downPt.x);

        if (stop_cnt != 1)
        {
    		angle = atan((float)(tempPt.x - downPt.x)/(float)(downPt.y - tempPt.y)) * 57.3;
        }
/*
        printf("banishP.x : %d\t banishP.y : %d\n", banishP.x, banishP.y);
        printf("tempPt.x : %d\t tempPt.y : %d\n", tempPt.x, tempPt.y);
        printf("downPt.x : %d\t downPt.y : %d\n", downPt.x, downPt.y);
        printf("leftP2.x : %d\t leftP2.y : %d\n", leftP2.x, leftP2.y);
        printf("rightP2.x : %d\t rightP2.y : %d\n", rightP2.x, rightP2.y);
*/
        cv::line(srcRGB, pt1, pt2, lineColor1, 1); // �Ͼ� ������ �׸���
        cv::line(srcRGB, pt3, pt4, lineColor2, 1); // �Ͼ� ������ �׸���

        cv::line(srcRGB, downPt, tempPt, lineColor3, 2); // �Ͼ� ������ �׸���
    }
    else if (pt1.x != 0 && pt3.x == 0)
    { 
        // left Point detection 

        // leftLine : first linear equation
        gradientL = (float)(pt2.y - pt1.y) / (float)(pt2.x - pt1.x);		// gradient
        interceptL = pt2.y - gradientL * pt2.x;					// y-intercept

        // leftPoint : nodePoint of two equation
        leftP1.x = (int)(interceptL / -gradientL);
        leftP1.y = (int)(gradientL * leftP1.x + interceptL);

        float c1 = leftP1.y - gradientL * leftP1.x;
        leftP2.y = HEIGHT;
        leftP2.x = (leftP2.y - c1) / gradientL;

        float gradient = (float)(pt2.y - pt1.y) / (float)(pt2.x - pt1.x);
		float intercept = pt2.y - gradient * pt2.x;

		upPt.x = (int)(intercept / -gradient);
		upPt.y = (int)(gradient * upPt.x + intercept);

        upPt.x = lkas(upPt.x, leftP2.x, rightP2.x, 1);

		tempPt.y = (int)(HEIGHT / 2 + offset);
		tempPt.x = (int)(((float)(upPt.x-downPt.x)/(float)(upPt.y - downPt.y))*(float)(tempPt.y - downPt.y)+(float)downPt.x);

        if (stop_cnt != 1)
        {
            angle = atan((float)(tempPt.x - downPt.x)/(float)(downPt.y - tempPt.y)) * 57.3;
        }
        //angle /= 2.0;
        //if (angle > 20)
        //{
        //    angle -= 20;
        //}

        cv::line(srcRGB, pt1, pt2, lineColor1, 1); // �Ͼ� ������ 
        
        cv::line(srcRGB, downPt, tempPt, lineColor3, 2);
    }
    else if (pt1.x == 0 && pt3.x != 0)
    { 
        // right Point detection 

        // rightLine : first linear equation
        gradientR = (float)(pt4.y - pt3.y) / (float)(pt4.x - pt3.x);		// gradient
        interceptR = pt4.y - gradientR * pt4.x;							// y-intercept

        // rightPoint : nodePoint of two equation
        rightP1.x = (int)(interceptR / -gradientR);
        rightP1.y = (int)(gradientR * rightP1.x + interceptR);

        float c2 = rightP1.y - gradientR * rightP1.x;
        rightP2.y = HEIGHT;
        rightP2.x = (rightP2.y - c2) / gradientR;

        float gradient = (float)(pt4.y - pt3.y) / (float)(pt4.x - pt3.x);
		float intercept = pt4.y - gradient * pt4.x;

		upPt.x = (int)(intercept / -gradient);
		upPt.y = (int)(gradient * upPt.x + intercept);

        upPt.x = lkas(upPt.x, leftP2.x, rightP2.x, 2);

		tempPt.y = (int)(HEIGHT / 2 + offset);
		tempPt.x = (int)(((float)(upPt.x-downPt.x)/(float)(upPt.y - downPt.y))*(float)(tempPt.y - downPt.y)+(float)downPt.x);

        if (stop_cnt != 1)
        {
            angle = atan((float)(tempPt.x - downPt.x)/(float)(downPt.y - tempPt.y)) * 57.3;
        }
        
        //angle /= 2.0;
        //if (angle < -20)
        //{
        //   angle += 20;
        //}

        cv::line(srcRGB, pt3, pt4, lineColor2, 1); // �Ͼ� ������ �׸���

        cv::line(srcRGB, downPt, tempPt, lineColor3, 2); // �Ͼ� ������ �׸���
    }

    //steering(angle);

    //printf("angle : %f\n", angle);  // -60 ~ 60 (max -80 ~ 80)

    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);

    //printf("stop_cnt : %d \t angle : %f\n", stop_cnt, angle);

    return angle;

/*
    if (ignoreL == 0)
    {
        return angle;
    }
    else if (ignoreL == 1)
    {
        return angle - 10.0;
    }
    else if (ignoreL == 2)
    {
        return angle + 10.0;
    }
*/
}

int opencv_obstacle(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, int step)
{
    int idx1 = 0;
    int area1 = 3;
    int area1_num = -1;
	int LowH1 = 170, HighH1 = 179;			//red
	int LowS = 50, HighS = 255;
	int LowV = 30, HighV = 255;

    int detect_step = 0;
    detect_step = step;

    Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    Mat hsvimg(ih, iw, CV_8UC3);
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

	Mat img_binary1;

    int element_shape = MORPH_RECT;
	Mat element = getStructuringElement(element_shape, Size(5,5));

    cvtColor(srcRGB, hsvimg, CV_BGR2HSV);		//transform hsv color space

	inRange(hsvimg, Scalar(LowH1,LowS,LowV), Scalar(HighH1, HighS, HighV), img_binary1);
	morphologyEx(img_binary1, img_binary1, MORPH_CLOSE, element);
    vector<vector<Point> > contour1;
    vector<Vec4i> hierarchy1;

    findContours(img_binary1, contour1, hierarchy1, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    if(!contour1.empty() && !hierarchy1.empty())
    {
        // iterate through all the top-level contours,
        // draw each connected component with its own random color
        for( ; idx1 >= 0; idx1 = hierarchy1[idx1][0] )
        {
            if (contour1[idx1].size() > area1)
            {
                area1 = contour1[idx1].size();
                area1_num = idx1;
            }
        }
        cout << contour1[area1_num].size() << endl;
        Scalar color( 0, 0, 255 );

        if (area1_num != -1 && area1 >= 30)
        {
            drawContours( srcRGB, contour1, area1_num, color, CV_FILLED, 8, hierarchy1 );
            putText(srcRGB, "obstacle : STOP!", Point(WIDTH/2-40, HEIGHT - 10), 1, 1, Scalar(255,255,255));
            detect_step++;
        }
        else detect_step = 0;
    }

    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
    return detect_step;
}

int colordetect(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, int step)
{
    int idx1 = 0;
    //int idx2 = 0;
    int idx3 = 0;

    int area1 = 3;
    //int area2 = 3
    int area3 = 3;

    int area1_num = -1;
    //int area2_num = -1;
    int area3_num = -1;

	int LowH1 = 170, HighH1 = 179;			//red
	//int LowH2 = 24, HighH2 = 33;			//yellow
	int LowH3 = 65, HighH3 = 85;			//green
	int LowS = 50, HighS = 255;
	int LowV = 30, HighV = 255;

    int color_step = 0;
    color_step = step;

    Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    Mat hsvimg(ih, iw, CV_8UC3);
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

	Mat img_binary1, img_binary2, img_binary3;

	int element_shape = MORPH_RECT;
	Mat element = getStructuringElement(element_shape, Size(5,5));

    cvtColor(srcRGB, hsvimg, CV_BGR2HSV);		//transform hsv color space

	inRange(hsvimg, Scalar(LowH1,LowS,LowV), Scalar(HighH1, HighS, HighV), img_binary1);
	//inRange(hsvimg, Scalar(LowH2,LowS,LowV), Scalar(HighH2, HighS, HighV), img_binary2);
	inRange(hsvimg, Scalar(LowH3,LowS+50,LowV), Scalar(HighH3, HighS, HighV), img_binary3);

	morphologyEx(img_binary1, img_binary1, MORPH_CLOSE, element);
	//morphologyEx(img_binary2, img_binary2, MORPH_CLOSE, element);
	morphologyEx(img_binary3, img_binary3, MORPH_CLOSE, element);
    
    vector<vector<Point> > contour1;
    //vector<vector<Point> > contour2;
    vector<vector<Point> > contour3;
    vector<Vec4i> hierarchy1;
    //vector<Vec4i> hierarchy2;
    vector<Vec4i> hierarchy3;


    if (color_step == 0)
    {
        findContours(img_binary1, contour1, hierarchy1, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

        if(!contour1.empty() && !hierarchy1.empty())
        {
            // iterate through all the top-level contours,
            // draw each connected component with its own random color
            for( ; idx1 >= 0; idx1 = hierarchy1[idx1][0] )
            {
                if (contour1[idx1].size() > area1)
                {
                    area1 = contour1[idx1].size();
                    area1_num = idx1;
                }
            }
            cout << contour1[area1_num].size() << endl;
            Scalar color( 0, 0, 255 );

            if (area1_num != -1 && area1 >= 20)
            {
                drawContours( srcRGB, contour1, area1_num, color, CV_FILLED, 8, hierarchy1 );
                putText(srcRGB, "Red : STOP!", Point(WIDTH/2-40, HEIGHT - 10), 1, 1, Scalar(255,255,255));
                color_step = 1;
            }
        }
    }
    else if (color_step >= 1)
    {
        if (color_step == 1)
        {
            putText(srcRGB, "Red : STOP!", Point(WIDTH/2-40, HEIGHT - 10), 1, 1, Scalar(255,255,255));
        }
        //findContours(img_binary2, contour2, hierarchy2, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
        findContours(img_binary3, contour3, hierarchy3, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    /*
        if(!contour2.empty() && !hierarchy2.empty())
        {
            // iterate through all the top-level contours,
            // draw each connected component with its own random color
            for( ; idx2 >= 0; idx2 = hierarchy2[idx2][0] )
            {
                if (contour2[idx2].size() > area2)
                {
                    area2= contour2[idx2].size();
                    area2_num = idx2;
                }
            }
            cout << contour2[area2_num].size() << endl;
            Scalar color( 0, 0, 255 );
            drawContours( srcRGB, contour2, area2_num, color, CV_FILLED, 8, hierarchy2 );
            putText(srcRGB, "yello : WAIT!", Point(WIDTH/2-40, HEIGHT - 10), 1, 1, Scalar(255,255,255));
            color_step = 1;
        }
    */

        if(!contour3.empty() && !hierarchy3.empty())
        {
            // iterate through all the top-level contours,
            // draw each connected component with its own random color
            for( ; idx3 >= 0; idx3 = hierarchy3[idx3][0] )
            {
                if (contour3[idx3].size() > area3)
                {
                    area3= contour3[idx3].size();
                    area3_num = idx3;
                }
            }
            //cout << contour3[area3_num].size() << endl;
            Scalar color( 0, 0, 255 );
            drawContours( srcRGB, contour3, area3_num, color, CV_FILLED, 8, hierarchy3 );

            if (area3 >= 8 && area3 <= 25 && (color_step % 2 == 1))
            {
                color_step = color_step + 2;
            }
            else if (area3 >= 8 && area3 <= 25 && color_step == 2)
            {
                color_step = 1;
            }
            else if ((color_step % 2 == 1))
            {
                color_step = 1;
            }

            if (area3 > 25 && (color_step % 2 == 1))
            {
                color_step = 2;
            }
            else if (area3 > 25)
            {
                color_step = color_step + 2;
            }
            else if ((color_step % 2 == 0))
            {
                color_step = 2;
            }

            if (color_step == 31)
            {
                putText(srcRGB, "green : Left Go!", Point(WIDTH/2-40, HEIGHT - 10), 1, 1, Scalar(255,255,255));
            }
            else if (color_step == 32)
            {
                putText(srcRGB, "green : Right Go!", Point(WIDTH/2-40, HEIGHT - 10), 1, 1, Scalar(255,255,255));
            }
        }
    }

    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
    return color_step;
}

void OpenCV_merge_image(unsigned char* src1, unsigned char* src2, unsigned char* dst, int w, int h)
{
    Mat src1AR32(h, w, CV_8UC4, src1);
    Mat src2AR32(h, w, CV_8UC4, src2);
    Mat dstAR32(h, w, CV_8UC4, dst);

    cvtColor(src2AR32, src2AR32, CV_BGRA2RGBA);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            double opacity = ((double)(src2AR32.data[y * src2AR32.step + x * src2AR32.channels() + 3])) / 255.;
            for (int c = 0; opacity > 0 && c < src1AR32.channels(); ++c) {
                unsigned char overlayPx = src2AR32.data[y * src2AR32.step + x * src2AR32.channels() + c];
                unsigned char srcPx = src1AR32.data[y * src1AR32.step + x * src1AR32.channels() + c];
                src1AR32.data[y * src1AR32.step + src1AR32.channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
            }
        }
    }

    memcpy(dst, src1AR32.data, w*h*4);
}

}

