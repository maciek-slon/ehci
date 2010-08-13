#ifndef EHCI_H_
#define EHCI_H_

#if _MSC_VER // this is defined when compiling with Visual Studio
#define EXPORT_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this
#else
#define EXPORT_API // XCode does not need annotating exported functions, so define is empty
#endif

//Make sure everything goes ok when building .dlls
#if defined (_WIN32) 
  #if defined(Ehci_EXPORTS)
    #define  DllExport __declspec(dllexport)
  #else
    #define  DllExport
  #endif /* Ehci_EXPORTS */
#else /* defined (_WIN32) */
 #define DllExport
#endif


#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <vector>
#include <list>
#include <string>


///ehci loop modes
///EHCI2DFACEDETECT - only makes 2d facedetect
///EHCI6DFACEDETECT - makes 2d and 6dof facedetection, so it is 0x00000001 | 0x00000010
///EHCI2DHANDDETECT - 2d hand detect
///EHCI6DFACEDETECT - makes 2d and 6dof handdetections, so it is 0x00000100 | 0x00001000
///EHCI6DFACEDETECTKEYFRAME - keyframe 2d facedetect 'or' 0x00000001 | 0x00010000
extern "C"{
enum detectionType{ EHCI2DFACEDETECT = 1, EHCI6DFACEDETECT =3 , EHCI2DHANDDETECT=4, EHCI6DHANDDETECT=12, EHCI6DFACEDETECTKEYFRAME=17};

//image structure for opengl texture
struct GLImage {
    unsigned long sizeX;
    unsigned long sizeY;
    uchar *data;
};
}

DllExport int isEhciTextureCreated();
DllExport float getEhciModelScale();
DllExport GLImage getEhciGlTexture();
DllExport IplImage* getGeneratedImage();
DllExport void setGeneratedImage(IplImage* image);
DllExport void updateGlPositMatrix(CvMatr32f rotation_matrix,CvVect32f translation_vector);
DllExport void setInitialRTMatrix(CvMatr32f rotation_matrix,CvVect32f translation_vector);

DllExport int getObjectPosition(IplImage* frame,int mode, CvPoint* upperHeadCorner,int* headWidth,int* headHeight);
DllExport int detect_and_draw( IplImage* img,CvPoint* upperHeadCorner,int* headWidth,int* headHeight,CvHaarClassifierCascade* cascade,
		CvMemStorage* storage);

DllExport void getPositMatrix(IplImage* myImage,int initialGuess, CvMatr32f rotation_matrix, CvVect32f translation_vector,
		int numOfTrackingPoints,int focus,CvPoint2D32f* points, CvPoint upperHeadCorner,
		int headWidth, int headHeight, float modelScale);

DllExport int insertNewPoints(IplImage* grey, int headX,int headY,int width, int height,
		CvPoint2D32f* points);

DllExport void setGLProjectionMatrix(double projectionMatrix[16]);

DllExport void ehciInit();
extern "C"{
EXPORT_API int ehciLoop(int mode, int initialGuess,int debug=0);
EXPORT_API void ehciExit();
EXPORT_API void getHeadBounds(int* headRefX,int* headRefY,int* aLastHeadW,int* aLastHeadH);
EXPORT_API void getGlPositMatrix(double myGlPositMatrix[16]);
EXPORT_API int updateTexture(void* colors, int width, int height);

}

DllExport int ehciLoop(int mode,int initialGuess,IplImage* createdImage);


DllExport void getReferenceHeadBounds(int* headRefX,int* headRefY,int* aLastHeadW,int* aLastHeadH);

DllExport void getReferenceCoordinate(float* x, float* y, float* z);
DllExport IplImage* getCurrentFrame();


#endif /*EHCI_H_*/
