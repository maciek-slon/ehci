#ifndef EHCI_H_
#define EHCI_H_


#include "cv.h"
#include "highgui.h"

#define EHCIMODELSCALE 50
#define EHCIFOCUS  602

///ehci loop modes
///EHCI2DFACEDETECT - only makes 2d facedetect
///EHCI6DFACEDETECT - makes 2d and 6dof facedetection, so it is 0x00000001 | 0x00000010
///EHCI2DHANDDETECT - 2d hand detect
///EHCI6DFACEDETECT - makes 2d and 6dof handdetections, so it is 0x00000100 | 0x00001000
enum detectionType{ EHCI2DFACEDETECT = 1, EHCI6DFACEDETECT =3 , EHCI2DHANDDETECT=4, EHCI6DHANDDETECT=12};

void updateGlPositMatrix(CvMatr32f rotation_matrix,CvVect32f translation_vector);
void setInitialRTMatrix(CvMatr32f rotation_matrix,CvVect32f translation_vector);



int getObjectPosition(IplImage* frame,int mode, CvPoint* upperHeadCorner,int* headWidth,int* headHeight);

int detect_and_draw( IplImage* img,CvPoint* upperHeadCorner,int* headWidth,int* headHeight,CvHaarClassifierCascade* cascade, 
		CvMemStorage* storage);

void getPositMatrix(IplImage* myImage,int initialGuess, CvMatr32f rotation_matrix, CvVect32f translation_vector,
		int numOfTrackingPoints,int focus,CvPoint2D32f* points, CvPoint upperHeadCorner, 
		int headWidth, int headHeight, float modelScale);

int insertNewPoints(IplImage* grey, int headX,int headY,int width, int height,
		CvPoint2D32f* points);

void setGLProjectionMatrix(double projectionMatrix[16]);

void ehciInit();
int ehciLoop(int mode, int initialGuess);

void getHeadBounds(int* headRefX,int* headRefY,int* aLastHeadW,int* aLastHeadH);

void getReferenceHeadBounds(int* headRefX,int* headRefY,int* aLastHeadW,int* aLastHeadH);


void getGlPositMatrix(double myGlPositMatrix[16]);


IplImage* getCurrentFrame();


void ehciExit();






#endif /*EHCI_H_*/
