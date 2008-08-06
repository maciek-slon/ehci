#ifndef EHCI_H_
#define EHCI_H_


#include "cv.h"
#include "highgui.h"

#define EHCIMODELSCALE 50
#define EHCIFOCUS  602

//ehci loop modes
//EHCI2DFACEDETECT - only makes 2d facedetect
//EHCI6DFACEDETECT - makes 2d and 6dof facedetection, so it is 0x00000001 | 0x00000010
int EHCI2DFACEDETECT = 1, EHCI6DFACEDETECT =3 ;

void updateGlPositMatrix(CvMatr32f rotation_matrix,CvVect32f translation_vector);
void setInitialRTMatrix(CvMatr32f rotation_matrix,CvVect32f translation_vector);



int getHeadPosition(IplImage* frame, CvPoint* upperHeadCorner,int* headWidth,int* headHeight );


void getPositMatrix(IplImage* myImage,int initialGuess, CvMatr32f rotation_matrix, CvVect32f translation_vector,
		int numOfTrackingPoints,int focus,CvPoint2D32f* points, CvPoint upperHeadCorner, 
		int headWidth, int headHeight, float modelScale);

int insertNewPoints(IplImage* grey, int headX,int headY,int width, int height,
		CvPoint2D32f* points);

void setGLProjectionMatrix(double projectionMatrix[16]);

int cvLoop(int mode, int initialGuess,int* refX,int * refY,int* myLastHeadW, int* myLastHeadH);


void getGlPositMatrix(double myGlPositMatrix[16]);


IplImage* getCurrentFrame();


void ehciExit();






#endif /*EHCI_H_*/
