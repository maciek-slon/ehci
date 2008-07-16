#include "ehci.h"
#include <stdio.h>



//static CvHaarClassifierCascade* cascade = 0;
//static CvMemStorage* storage = 0;

//loads cascade xml
void loadCascade(CvHaarClassifierCascade** cascade){	

	*cascade = (CvHaarClassifierCascade*)cvLoad( "haarcascade_frontalface_default.xml", 0, 0, 0 );
	if( !cascade ){
		printf( "ERROR: Could not load classifier cascade\n" );
	}	


}



void detect_and_draw( IplImage* img,CvPoint* upperHeadCorner,int* headWidth,int* headHeight,CvHaarClassifierCascade* cascade, 
		CvMemStorage* storage)
{
	
	double scale = 2.0;
		IplImage *gray, *small_img;
		int i, j;
	
	

	gray = cvCreateImage( cvSize(img->width,img->height), 8, 1 );
	small_img = cvCreateImage( cvSize( cvRound (img->width/scale),
			cvRound (img->height/scale)), 8, 1 );

	cvCvtColor( img, gray, CV_BGR2GRAY );
	cvResize( gray, small_img, CV_INTER_LINEAR );
	cvEqualizeHist( small_img, small_img );
	cvClearMemStorage( storage );
	
	printf("Dentro do detect\n");
	if( cascade )
	{
		printf("Entrou no cascade");
		double t = (double)cvGetTickCount();
		CvSeq* faces = cvHaarDetectObjects( small_img, cascade, storage,
				1.2, 2, 0
				|CV_HAAR_FIND_BIGGEST_OBJECT
				//|CV_HAAR_DO_ROUGH_SEARCH
				//|CV_HAAR_DO_CANNY_PRUNING
				//|CV_HAAR_SCALE_IMAGE
				,
				cvSize(40, 40) );
		t = (double)cvGetTickCount() - t;
		//        printf( "detection time = %gms\n", t/((double)cvGetTickFrequency()*1000.) );
		for( i = 0; i < (faces ? faces->total : 0); i++ )
		{
			CvRect* r = (CvRect*)cvGetSeqElem( faces, i );
			*upperHeadCorner= cvPoint(r->x*scale,r->y*scale);			
			*headWidth=r->width*scale;
			*headHeight=r->height*scale;					  	 
		}
	}

	//    cvShowImage( "result", img );
	cvReleaseImage( &gray );
	cvReleaseImage( &small_img );
}




void updateGlPositMatrix(CvMatr32f rotation_matrix,CvVect32f translation_vector,double glPositMatrix[16]){

	glPositMatrix[0] = rotation_matrix[0];
	glPositMatrix[1] = rotation_matrix[3];
	glPositMatrix[2] = rotation_matrix[6];
	glPositMatrix[3] = 0.0;

	glPositMatrix[4] = rotation_matrix[1];
	glPositMatrix[5] = rotation_matrix[4];
	glPositMatrix[6] = rotation_matrix[7];
	glPositMatrix[7] = 0.0;     

	glPositMatrix[8] =  rotation_matrix[2];
	glPositMatrix[9] =  rotation_matrix[5];
	glPositMatrix[10] = rotation_matrix[8];
	glPositMatrix[11] = 0.0;
	
	glPositMatrix[12] =  translation_vector[0];
	glPositMatrix[13] =  translation_vector[1]; 
	glPositMatrix[14] =  translation_vector[2]; //negative
	glPositMatrix[15] = 1.0; //homogeneous

}
 
void setInitialRTMatrix(CvMatr32f rotation_matrix,CvVect32f translation_vector){


	rotation_matrix[0]=1.0; rotation_matrix[1]=  0; rotation_matrix[2]=  0;
	rotation_matrix[3]=  0; rotation_matrix[4]=1.0; rotation_matrix[5]=  0;	
	rotation_matrix[6]=  0; rotation_matrix[7]=  0; rotation_matrix[8]=1.0;

	translation_vector[0]=   -90;
	translation_vector[1]=    80;
	translation_vector[2]=+300.0;

}
CvHaarClassifierCascade* cascade=0;
CvMemStorage* storage;
void getHeadPosition(IplImage* frame, CvPoint* upperHeadCorner,int* headWidth,int* headHeight ){
			
	if(!cascade){
		loadCascade(&cascade);
		storage = cvCreateMemStorage(0);
	}
	
	
	IplImage *frame_copy = 0;
	frame_copy = cvCreateImage( cvSize(frame->width,frame->height), IPL_DEPTH_8U, frame->nChannels );
	cvCopy( frame, frame_copy, 0 );
		
	
	detect_and_draw(frame_copy,upperHeadCorner,headWidth,headHeight,cascade,storage);

	cvRectangle(frame, *upperHeadCorner, cvPoint(upperHeadCorner->x + *headWidth,upperHeadCorner->y + *headHeight), cvScalar(0,0,255), 1);

	cvReleaseImage( &frame_copy );

}
