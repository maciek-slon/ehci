#ifndef EHCI_H_
#define EHCI_H_


#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <vector>
#include <list>
#include <string>


//these values are overiden in case data/config.ini
//is loaded with these values
int USE_RANSAC=1;
int RANSAC_SAMPLES=8;
double RANSAC_DISTANCE_THRESHOLD = 30.0;
int RANSAC_ITERATIONS = 20;

float EHCIMODELSCALE = 1.0;//50;
int EHCIFOCUS  = 602;

int chosenCamera = 0;
char* movieFile=0;

///ehci loop modes
///EHCI2DFACEDETECT - only makes 2d facedetect
///EHCI6DFACEDETECT - makes 2d and 6dof facedetection, so it is 0x00000001 | 0x00000010
///EHCI2DHANDDETECT - 2d hand detect
///EHCI6DFACEDETECT - makes 2d and 6dof handdetections, so it is 0x00000100 | 0x00001000
///EHCI6DFACEDETECTKEYFRAME - keyframe 2d facedetect 'or' 0x00000001 | 0x00010000
enum detectionType{ EHCI2DFACEDETECT = 1, EHCI6DFACEDETECT =3 , EHCI2DHANDDETECT=4, EHCI6DHANDDETECT=12, EHCI6DFACEDETECTKEYFRAME=17};

//image structure for opengl texture
struct GLImage {
    unsigned long sizeX;
    unsigned long sizeY;
    uchar *data;
};
typedef struct GLImage GLImage;
GLImage glTexture;
int textureCreated = 0;


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
int ehciLoop(int mode,int initialGuess,IplImage* createdImage);

void getHeadBounds(int* headRefX,int* headRefY,int* aLastHeadW,int* aLastHeadH);

void getReferenceHeadBounds(int* headRefX,int* headRefY,int* aLastHeadW,int* aLastHeadH);

void getReferenceCoordinate(float* x, float* y, float* z);


void getGlPositMatrix(double myGlPositMatrix[16]);


IplImage* getCurrentFrame();


void ehciExit();






#endif /*EHCI_H_*/
