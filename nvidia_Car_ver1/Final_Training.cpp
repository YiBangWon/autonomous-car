#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include "opencv2/opencv.hpp"
#include <cmath>
#include "opencv/cv.hpp"
#include "opencv/highgui.h"
#include <stdio.h>
#include <queue>
#include "cv.hpp"
#include <stdio.h>
//#include <opencv2\core\core.hpp>
//#include <opencv2\ml\ml.hpp>
//#include <opencv\ml.h>

using namespace cv;
using namespace std;
using namespace cv::ml;

#define _CRT_SECURE_NO_WARNINGS

//
//#ifdef _DEBUG        
//#pragma comment(lib, "opencv_core2413d.lib")         
//#pragma comment(lib, "opencv_imgproc2413d.lib")     
//#pragma comment(lib, "opencv_objdetect2413d.lib")
//#pragma comment(lib, "opencv_highgui2413d.lib")        
//#pragma comment(lib, "opencv_ml2413d.lib")      
//
//#else        
//#pragma comment(lib, "opencv_core2413.lib")        
//#pragma comment(lib, "opencv_imgproc2413.lib")        
//#pragma comment(lib, "opencv_objdetect2413.lib")        
//#pragma comment(lib, "opencv_highgui2413.lib")        
//#pragma comment(lib, "opencv_ml2413.lib")          
//#endif 

//class MySvm : public SVM {
//public:
//   int get_alpha_count() {
//      return this->sv_total;
//   }
//
//   int get_sv_dim() {
//      return this->var_all;
//   }
//
//   int get_sv_count() {
//      return this->decision_func->sv_count;
//   }
//
//   double* get_alpha() {
//      return this->decision_func->alpha;
//   }
//
//   float** get_sv() {
//      return this->sv;
//   }
//
//   float get_rho() {
//      return this->decision_func->rho;
//   }
//};
//

void main() {
	//variables
	char FullFileName[100];
	char pos_FirstFileName[100] = "C:\\Users\\kang_\\Desktop\\공학프로젝트기획_smartcar\\samples\\data\\pos_1373";
	//char SaveHogDesFileName[100] = "Positive.xml";
	char neg_FirstFileName[100] = "C:\\Users\\kang_\\Desktop\\공학프로젝트기획_smartcar\\samples\\data\\neg_2753";
	//char SaveHogDesFileName[100] = "Negative.xml";
	int pos_FileNum = 1373;
	int neg_FileNum = 2753;

	vector< vector < float> > pos_v_descriptorsValues;
	vector< vector < Point> > pos_v_locations;
	vector <String> pos_filenames;

	vector< vector < float> > neg_v_descriptorsValues;
	vector< vector < Point> > neg_v_locations;
	vector <String> neg_filenames;

	glob(pos_FirstFileName, pos_filenames);
	glob(neg_FirstFileName, neg_filenames);

	//for POS
	for (int i = 0; i< pos_FileNum; ++i)
	{
		//read image file
		Mat pos_img, pos_img_gray;
		pos_img = imread(pos_filenames[i]);
		cout << pos_filenames[i] << endl;

		//resizing
		resize(pos_img, pos_img, Size(64, 64)); //Size(64,48) ); //Size(32*2,16*2)); //Size(80,72) ); 
												//gray
		cvtColor(pos_img, pos_img_gray, CV_RGB2GRAY);

		//extract feature
		//HOGDescriptor d(Size(32, 32), Size(8, 8), Size(4, 4), Size(4, 4), 9);
		HOGDescriptor d(Size(64, 64), Size(32, 32), Size(16, 16), Size(16, 16), 9);
		//HOGDescriptor d(Size(64, 64), Size(32, 32), Size(8, 8), Size(8, 8), 9);

		vector< float> pos_descriptorsValues;
		vector< Point> pos_locations;
		d.compute(pos_img_gray, pos_descriptorsValues, Size(0, 0), Size(0, 0), pos_locations);

		//printf("descriptor number =%d\n", descriptorsValues.size() );
		pos_v_descriptorsValues.push_back(pos_descriptorsValues);
		pos_v_locations.push_back(pos_locations);
		//show image
		imshow("pos_origin", pos_img);
		pos_img.release();
		pos_img_gray.release();
		waitKey(5);
	}
	destroyWindow("pos_origin");

	//save to xml
	//FileStorage hogXml(SaveHogDesFileName, FileStorage::WRITE); //FileStorage::READ
	//2d vector to Mat
	int pos_row = pos_v_descriptorsValues.size(), pos_col = pos_v_descriptorsValues[0].size();
	printf("col=%d, row=%d\n", pos_row, pos_col);
	Mat pMat(pos_row, pos_col, CV_32F);
	//save Mat to XML
	for (int i = 0; i< pos_row; ++i)
		memcpy(&(pMat.data[pos_col * i * sizeof(float)]), pos_v_descriptorsValues[i].data(), pos_col * sizeof(float));

	//write xml
	//write(hogXml, "Descriptor_of_images", M);

	//write(hogXml, "Descriptor", v_descriptorsValues );
	//write(hogXml, "locations", v_locations );
	//hogXml.release();
	cout << "pos_Complete!!!!!!!!!!!!" << endl;

	//for NEG
	for (int i = 0; i< neg_FileNum; ++i)
	{
		//read image file
		Mat neg_img, neg_img_gray;
		neg_img = imread(neg_filenames[i]);
		cout << neg_filenames[i] << endl;

		//resizing
		resize(neg_img, neg_img, Size(64, 64)); //Size(64,48) ); //Size(32*2,16*2)); //Size(80,72) ); 
												//gray
		cvtColor(neg_img, neg_img_gray, CV_RGB2GRAY);

		//extract feature
		//HOGDescriptor d(Size(32, 32), Size(8, 8), Size(4, 4), Size(4, 4), 9);
		HOGDescriptor d(Size(64, 64), Size(32, 32), Size(16, 16), Size(16, 16), 9);
		//HOGDescriptor d(Size(64, 64), Size(32, 32), Size(8, 8), Size(8, 8), 9);

		vector< float> neg_descriptorsValues;
		vector< Point> neg_locations;
		d.compute(neg_img_gray, neg_descriptorsValues, Size(0, 0), Size(0, 0), neg_locations);

		//printf("descriptor number =%d\n", descriptorsValues.size() );
		neg_v_descriptorsValues.push_back(neg_descriptorsValues);
		neg_v_locations.push_back(neg_locations);
		//show image
		imshow("neg_origin", neg_img);
		neg_img.release();
		neg_img_gray.release();
		waitKey(5);
	}

	destroyWindow("neg_origin");
	//save to xml
	//FileStorage hogXml(SaveHogDesFileName, FileStorage::WRITE); //FileStorage::READ
	//2d vector to Mat
	int neg_row = neg_v_descriptorsValues.size(), neg_col = pos_v_descriptorsValues[0].size();
	printf("col=%d, row=%d\n", neg_row, neg_col);
	Mat nMat(neg_row, neg_col, CV_32F);
	//save Mat to XML
	for (int i = 0; i< neg_row; ++i)
		memcpy(&(nMat.data[neg_col * i * sizeof(float)]), neg_v_descriptorsValues[i].data(), neg_col * sizeof(float));

	//write xml
	//write(hogXml, "Descriptor_of_images", M);

	//write(hogXml, "Descriptor", v_descriptorsValues );
	//write(hogXml, "locations", v_locations );
	//hogXml.release();
	cout << "neg_Complete!!!!!!!!!!!!" << endl;

	const bool AUTO_TRAIN_ENABLED = false;
	//create xml to read
	//cout << "Input Files Load " << endl;
	////FileStorage read_PositiveXml("test2_pos_2.xml", FileStorage::READ);
	////FileStorage read_NegativeXml("test2_neg.xml", FileStorage::READ);
	//FileStorage read_PositiveXml("Positive.xml", FileStorage::READ);
	//FileStorage read_NegativeXml("Negative.xml", FileStorage::READ);
	char SVMSaveFile[100] = "0706_1373_2753_RBF_1e4_1e-5.xml";

	////Positive Mat, Read Row, Cols
	//Mat pMat;
	//read_PositiveXml["Descriptor_of_images"] >> pMat;
	int pRow, pCol;
	pRow = pMat.rows; pCol = pMat.cols;
	//read_PositiveXml.release();

	////Negative Mat, Read Row, Cols
	//Mat nMat;
	//read_NegativeXml["Descriptor_of_images"] >> nMat;
	int nRow, nCol;
	nRow = nMat.rows; nCol = nMat.cols;
	//read_NegativeXml.release();

	//Make training data for SVM
	cout << "Make Training data for SVM  " << endl;
	Mat PN_Descriptor_mtx(pRow + nRow, pCol, CV_32FC1); // pCol and nCol must be the same
	memcpy(PN_Descriptor_mtx.data, pMat.data, sizeof(float)* pMat.cols * pMat.rows);
	int startP = sizeof(float)* pMat.cols * pMat.rows;
	memcpy(&(PN_Descriptor_mtx.data[startP]), nMat.data, sizeof(float)* nMat.cols * nMat.rows);

	//data labeling
	cout << "Data Labeling  " << endl;
	Mat labels(pRow + nRow, 1, CV_32S, Scalar(-1));
	labels.rowRange(0, pRow) = Scalar(1); // pMat.data

	//Set svm parameters
	cout << "Set SVM parameters " << endl;
	/*
	CvSVM svm;
	CvSVMParams params;
	params.svm_type = CvSVM::C_SVC;
	params.kernel_type = CvSVM::RBF;
	params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100000, 1e-6);
	*/
	Ptr<SVM> svm = SVM::create();
	svm->setType(SVM::C_SVC);
	svm->setKernel(SVM::RBF);

	//Training
	cout << "Training" << endl;
	if (AUTO_TRAIN_ENABLED) {
		//Ptr<TrainData> td_auto = TrainData::create(PN_Descriptor_mtx, ROW_SAMPLE, labels);
		svm->trainAuto(PN_Descriptor_mtx, ROW_SAMPLE, labels);
	}
	else {
		/*svm->setC(1.25e1);
		svm->setGamma(5.0625e-1);*/
		svm->setC(1.25e2);
		svm->setGamma(6.0625e-1);
		svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, (int)1e4, 1e-5));
		svm->train(PN_Descriptor_mtx, ROW_SAMPLE, labels);
	}
	//svm.train_auto(PN_Descriptor_mtx, labels, Mat(), Mat(), params);
	//params = svm.get_params();

	//Trained data save
	cout << "Save File " << endl;
	svm->save(SVMSaveFile);

	//svm->load(SVMSaveFile);
}
