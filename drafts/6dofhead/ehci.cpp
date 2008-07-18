#include "ehci.h"
#include <stdio.h>

#include <vector>



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

void getHeadPosition(IplImage* frame, CvPoint* upperHeadCorner,int* headWidth,int* headHeight ){
	
	static CvHaarClassifierCascade* cascade=0;
	static CvMemStorage* storage;
			
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

/*
 * This function was used to plot the model using openCv matrix multiplications
 * during development phase. There's no use for it in a while.
 */

/*
  
float vertices[4719];

float points3d[NUMPTS][4]={ {0,0,0,1},{100*scale,0,0,1},{75*scale,-100*scale,0,1},{25*scale,-100*scale,0,1},
		{40*scale,-40*scale,+100*scale,1},{60*scale,-40*scale,+100*scale,1},
		{60*scale,-60*scale,+100*scale,1},{50*scale,-50*scale,+140*scale,1}
};
int model=0;
float gama = 0;//3.1415;
float dScale = 10.0;
float myRoty[] = { cos(theta),   0,  sin(theta),   0,
		0, 	1,  	0,  	  0,
		-sin(theta),   0,   cos(theta),  0,
		0,  0,  0,  1 };
float myRotz[] = { cos(gama),  -sin(gama),    0,  1.874*dScale,
		sin(gama),   cos(gama),    0,  -1.999*dScale,
		0, 	0,  	1,  	    -2.643*dScale,
		0,  0,  0,  1 };
		
void loadVertices(){
	int pos = 0;
	FILE* in = fopen("vertices.txt","r");
	while(fscanf(in,"%f",&vertices[pos++])!=EOF);
	fclose(in);
}

void plot2dModel(CvMatr32f rotation_matrix,CvVect32f translation_vector){

	float a[] = {  rotation_matrix[0],  rotation_matrix[1],  rotation_matrix[2], translation_vector[0],
			rotation_matrix[3],  rotation_matrix[4],  rotation_matrix[5], translation_vector[1],
			rotation_matrix[6],  rotation_matrix[7],  rotation_matrix[8], translation_vector[2],
			0,  0,  1,  0 };


	float pos[] = {0,0,0};

	CvMat Ma = cvMat(4, 4, CV_32FC1, a);

	int w;
	CvMat Mpoints[NUMPTS];

	for(w=0;w<NUMPTS;w++){
		Mpoints[w] = cvMat( 4, 1, CV_32FC1,&points3d[w]);
	}

	CvMat* Mr1 =  cvCreateMat(4,1,CV_32FC1);

	//plotting model
	if(model){
		for(int i=0;i<4719/3;i++){
			float myPoints[4];
			float scale = 30;

			CvMat Mry = cvMat(4, 4, CV_32FC1, myRoty);
			CvMat Mrz = cvMat(4, 4, CV_32FC1, myRotz);

			myPoints[0]=scale*vertices[3*i+0];
			myPoints[1]=scale*vertices[3*i+1];
			myPoints[2]=scale*vertices[3*i+2];
			myPoints[3]=1;
			CvMat temp = cvMat(4,1,CV_32FC1,&myPoints);

			cvMatMul(&Mry,&temp,&temp);
			cvMatMul(&Mrz,&temp,&temp);
			cvMatMul(&Ma,&temp,Mr1);

			cvCircle( image, cvPoint(320+1000*cvmGet(Mr1,0,0)/cvmGet(Mr1,2,0),240+1000*cvmGet(Mr1,1,0)/cvmGet(Mr1,2,0)), 3, CV_RGB(0,0,200), -1, 8,0);
		}		

	}



	for(w=0;w<NUMPTS;w++){

		cvMatMul(&Ma,&Mpoints[w],Mr1);
		//		printf("coord %f %f ",cvmGet(&Mpoints[w],0,0),cvmGet(&Mpoints[w],1,0));
		//		printf("coord %f %f\n",cvmGet(Mr1,0,0),cvmGet(Mr1,1,0));
		cvCircle( image, cvPoint(320+1000*cvmGet(Mr1,0,0)/cvmGet(Mr1,2,0),240+1000*cvmGet(Mr1,1,0)/cvmGet(Mr1,2,0)), w==0?8:3,w%2==0?CV_RGB(128,0,0): (w==7? CV_RGB(0,0,200) : CV_RGB(200,0,0)), -1, 8,0);

	}

}
*/

/*
 * This function will retrieve the rotation and translation matrixes 
 * using the POSIT algorithm
 * In case the initialGuess parameter is set to 1, the algorithm will
 * map points to the sinoidal head, else it will only track to the original
 * ones. In a future version, this function should also map new points back
 * to the current head position 
 * 
 */

std::vector<CvPoint3D32f> modelPoints;

void getPositMatrix(IplImage* myImage,int initialGuess, CvMatr32f rotation_matrix, CvVect32f translation_vector,
		int numOfTrackingPoints,int focus,CvPoint2D32f* points, CvPoint upperHeadCorner, 
		int headWidth, int headHeight, int* refX, int* refY){

	float cubeSize = 100.0;
	float alturaNariz = 20.0;

	int i;



	std::vector<CvPoint2D32f> imagePoints;

	if(initialGuess) modelPoints.clear();

	//setInitialRTMatrix();

	for(int i=0;i<numOfTrackingPoints;i++){
		float myPixel[4];

		int px = cvPointFrom32f(points[i]).x - upperHeadCorner.x ;
		int py = cvPointFrom32f(points[i]).y - upperHeadCorner.y ;
		int vertIndex = cvRound(3036.0*myPixel[0]);
		//glReadPixels(px,480-py,1,1,GL_RGBA,GL_FLOAT,&myPixel);
		
	
		if(initialGuess){
			if(i==0){
				
				*refX = cvPointFrom32f(points[i]).x-upperHeadCorner.x;//(int)(px/(1.0*headWidth)*100);
				*refY = cvPointFrom32f(points[i]).y-upperHeadCorner.y;//-(int)(py/(1.0*headHeight)*100);
				//refZ = 0;//(int)(40*sin(px*3.141593/headWidth));
			}
			float cScale = 100;
			
			float fx = (1.6667 * px/(1.0*headWidth)) - 0.332;
			float fy = (1.6667 * py/(1.0*headHeight)) - 0.332;
			float fz = sin(fx*3.141593);//cos((px-0.5*headWidth)/headWidth * 1.2 *3.141593);
			printf("px %d py %d hw %d hh %d fx %f ifx %d fy %f fz %f\n",px,py,headWidth,headHeight,fx,int(fx*cScale),fy,fz);
			
			modelPoints.push_back(   cvPoint3D32f( 	(int)(fx * cScale),
												   -(int)(fy * cScale),	
													(int)(fz * cScale)));
			
			/*modelPoints.push_back(   cvPoint3D32f( 	(int)(px/(1.0*headWidth)*cScale),
					-(int)(py/(1.0*headHeight)*cScale),	
					(int)(1.1*cScale*sin(px*3.141593/headWidth)) ));*/
			//(px<headWidth/2.0)?(int)((headWidth/4.0)*(px/(headWidth/2.0))):(int)((headWidth/4.0)*((headWidth -px)/(1.0*headWidth)))

			CvPoint2D32f point2D;
			point2D.x = cvPointFrom32f(points[i]).x-320;
			point2D.y = cvPointFrom32f(points[i]).y-240;				
			imagePoints.push_back( point2D );
			printf("Ip %f %f\n", point2D.x, point2D.y);

		}
		else if(numOfTrackingPoints==modelPoints.size()){
			CvPoint2D32f point2D;
			point2D.x = cvPointFrom32f(points[i]).x-320;
			point2D.y = cvPointFrom32f(points[i]).y-240;
			imagePoints.push_back( point2D );
			printf("Ip %f %f\n", point2D.x, point2D.y);

		}



		//}

	}

	
	if(modelPoints.size()==numOfTrackingPoints){
		printf("Creating posit with %d points\n",modelPoints.size());
		CvPOSITObject *positObject = cvCreatePOSITObject( &modelPoints[0], static_cast<int>(modelPoints.size()) );


		//set posit termination criteria: 1000 max iterations, convergence epsilon 1.0e-5
		CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 1000, 1.0e-5 );
		
		cvPOSIT( positObject, &imagePoints[0], focus, criteria, rotation_matrix, translation_vector ); 
		cvReleasePOSITObject (&positObject);

	}
	printf("Matrix data\n");
	for(i=0;i<9;i++){
		printf("%.5f ",rotation_matrix[i]);
	}
	for(i=0;i<3;i++){
		printf("%.5f ",translation_vector[i]);
	}
	printf("\n");



}