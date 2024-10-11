
#include <iostream>
#include <opencv2/core/bufferpool.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/ml/ml.hpp>
#include <stdio.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/opencv.hpp>
#include "Base.h"
using namespace std;
using namespace cv;
using namespace cv::ml;

int svm_num_detect(Ptr<SVM> svm, Mat img)
{
	Mat test;

	img.copyTo(test);

	cout << "Load test image done..." << endl;
	resize(test, test, Size(28, 28));

	HOGDescriptor hog(Size(28, 28), Size(14, 14), Size(7, 7), Size(7, 7), 9);

	vector<float> descriptors;

	hog.compute(test, descriptors, Size(1, 1));
	Mat query;
	int r = svm->predict(descriptors);
	svm->predict(descriptors, query, 4);
	cout << query << endl;
	cout << "The number is " << r << endl;

	return r;
}

int Get_num_hog(Ptr<SVM> &svm, Mat num_roi)
{

	clock_t op, ed;
	op = clock();

	if (!svm)
	{
		cout << "Load file failed..." << endl;
	}

	cout << "Load test image done..." << endl;
	resize(num_roi, num_roi, Size(28, 28));


	HOGDescriptor hog(Size(28, 28), Size(14, 14), Size(7, 7), Size(7, 7), 9);

	vector<float> descriptors;

	hog.compute(num_roi, descriptors, Size(1, 1));

	int r = svm->predict(descriptors);
	ed = clock();

	printf("time1:%.3fs\n", (double)(ed - op) / CLOCKS_PER_SEC);
	return r;
}