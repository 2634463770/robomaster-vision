#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include "DetectArmor.h"

using namespace std;
using namespace cv;

int Detect::num_detect(Mat &num_roi)
{
	char load[50];
	char num[10];

	for (int i = 1; i < 6; i++)
	{
		strcpy(load, "/home/u0/Pictures/num/test_num/");
		sprintf(num, "%d", int(i));
		strcat(load, num);
		strcat(load, ".jpg");

		Mat img, templ, result;

		num_roi.copyTo(img);

		cvtColor(img, img, CV_BGR2GRAY);
		threshold(img, img, 20, 255, THRESH_BINARY);

		templ = imread(load);
		resize(templ, templ, Size(64, 80));
		cvtColor(templ, templ, CV_BGR2GRAY);
		threshold(templ, templ, 100, 255, THRESH_BINARY);
		imshow("templ", templ);

		int result_cols = img.cols - templ.cols + 1;
		int result_rows = img.rows - templ.rows + 1;
		result.create(result_cols, result_rows, img.type()); // CV_32FC1

		matchTemplate(img, templ, result, CV_TM_CCOEFF);
		normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

		double minVal = -1;
		double maxVal;
		Point minLoc;
		Point maxLoc;
		Point matchLoc;
		cout << "匹配度：" << minVal << endl;
		minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

		cout << i << "匹配度：" << minVal << endl;

		matchLoc = minLoc;

		rectangle(img, matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), Scalar(0, 255, 0), 2, 8, 0);

		imshow("img", img);
		waitKey(0);
	}

	return 0;
}
