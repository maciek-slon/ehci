#ifndef EHCI_H_
#define EHCI_H_


#include "cv.h"
#include "highgui.h"

void updateGlPositMatrix(CvMatr32f rotation_matrix,CvVect32f translation_vector,double glPositMatrix[16]);
void setInitialRTMatrix(CvMatr32f rotation_matrix,CvVect32f translation_vector);


/*uses ViolaJones to find head position
 *returns upperHeadCorner, headWidth, and headHeight
 */
void getHeadPosition(IplImage* frame, CvPoint* upperHeadCorner,int* headWidth,int* headHeight );




#endif /*EHCI_H_*/
