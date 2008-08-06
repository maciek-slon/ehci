#include "ehci.h"
#include <stdio.h>

#include <vector>


/**
 * Loads cascade xml
 */
void loadCascade(CvHaarClassifierCascade** cascade){	

	*cascade = (CvHaarClassifierCascade*)cvLoad( "data/haarcascade_frontalface_default.xml", 0, 0, 0 );
	
	if( !(*cascade) ){
		printf( "ERROR: Could not load classifier cascade. The required file 'haarcascade_frontalface_default.xml' should be in a 'data' subdirectory under the directory in which the application is being run.\n" );
		exit(1);
	}	


}

/**
 * Optimized function to detect head position and draw a rectangle around it.
 * This function uses OpenCV implemented Viola-Jones algorithm with 
 * Haar-like features and trained cascades. Refer to OpenCV documentation.
 */
int detect_and_draw( IplImage* img,CvPoint* upperHeadCorner,int* headWidth,int* headHeight,CvHaarClassifierCascade* cascade, 
		CvMemStorage* storage)
{
	
	double scale = 2.0;
		IplImage *gray, *small_img;
		int i, j,detected = 0;
	
	

	gray = cvCreateImage( cvSize(img->width,img->height), 8, 1 );
	small_img = cvCreateImage( cvSize( cvRound (img->width/scale),
			cvRound (img->height/scale)), 8, 1 );

	cvCvtColor( img, gray, CV_BGR2GRAY );
	cvResize( gray, small_img, CV_INTER_LINEAR );
	cvEqualizeHist( small_img, small_img );
	cvClearMemStorage( storage );
	
	if( cascade )
	{		
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
			detected++;//used to return function value
		}
	}

	cvReleaseImage( &gray );
	cvReleaseImage( &small_img );
	
	return detected;
}


/**
 * internal ehci function to get 2d head upper left corner, width and height
 * the width and height are proportional to Viola-Jones trained cascade
 * which are a little bit smaller than the real ones
 * 
 * returns 0 if no head was found 
 */
int getHeadPosition(IplImage* frame, CvPoint* upperHeadCorner,int* headWidth,int* headHeight ){
	
	static CvHaarClassifierCascade* cascade=0;
	static CvMemStorage* storage;
	int detected;
			
	if(!cascade){
		loadCascade(&cascade);
		storage = cvCreateMemStorage(0);
	}
	
	
	IplImage *frame_copy = 0;
	frame_copy = cvCreateImage( cvSize(frame->width,frame->height), IPL_DEPTH_8U, frame->nChannels );
	
	cvCopy( frame, frame_copy, 0 );
	
		
	
	detected = detect_and_draw(frame_copy,upperHeadCorner,headWidth,headHeight,cascade,storage);

	cvRectangle(frame, *upperHeadCorner, cvPoint(upperHeadCorner->x + *headWidth,upperHeadCorner->y + *headHeight), cvScalar(0,0,255), 1);

	cvReleaseImage( &frame_copy );
	
	return detected;

}



double glPositMatrix[16];
void updateGlPositMatrix(CvMatr32f rotation_matrix,CvVect32f translation_vector){

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
	glPositMatrix[14] =  translation_vector[2];
	glPositMatrix[15] = 1.0; //homogeneous

}

/**
 *Copies currently detected posit matrix (translation and rotation)
 *in the openGl format to the parameter array  
 */

void getGlPositMatrix(double myGlPositMatrix[16]){
	for(int i=0;i<16;i++)
		myGlPositMatrix[i] = glPositMatrix[i];
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



void printMatrixData(CvMatr32f rotation_matrix, CvVect32f translation_vector){
	printf("Matrix data\n");
	for(int i=0;i<9;i++){
		printf("%.5f ",rotation_matrix[i]);
	}
	for(int i=0;i<3;i++){
		printf("%.5f ",translation_vector[i]);
	}
	printf("\n");
}



std::vector<CvPoint3D32f> modelPoints;

/**
 * This function will retrieve the rotation and translation matrixes 
 * using the POSIT algorithm
 * In case the initialGuess parameter is set to 1, the algorithm will
 * map points to the sinoidal head, else it will only track to the original
 * ones. In a future version, this function should also map new points back
 * to the current head position 
 * 
 */

void getPositMatrix(IplImage* myImage,int initialGuess, CvMatr32f rotation_matrix, CvVect32f translation_vector,
		int numOfTrackingPoints,int focus,CvPoint2D32f* points, CvPoint upperHeadCorner, 
		int headWidth, int headHeight, float modelScale){

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
			
			float fx = (1.6667 * px/(1.0*headWidth)) - 0.332;
			float fy = (1.6667 * py/(1.0*headHeight)) - 0.332;
			float fz = sin(fx*3.141593);//cos((px-0.5*headWidth)/headWidth * 1.2 *3.141593);
			//printf("px %d py %d hw %d hh %d fx %f ifx %d fy %f fz %f\n",px,py,headWidth,headHeight,fx,int(fx*cScale),fy,fz);
			
			modelPoints.push_back(   cvPoint3D32f( 	(int)(fx * modelScale),
												   -(int)(fy * modelScale),	
													(int)(fz * modelScale)));
			
			/*modelPoints.push_back(   cvPoint3D32f( 	(int)(px/(1.0*headWidth)*cScale),
					-(int)(py/(1.0*headHeight)*cScale),	
					(int)(1.1*cScale*sin(px*3.141593/headWidth)) ));*/
			//(px<headWidth/2.0)?(int)((headWidth/4.0)*(px/(headWidth/2.0))):(int)((headWidth/4.0)*((headWidth -px)/(1.0*headWidth)))

			CvPoint2D32f point2D;
			point2D.x = cvPointFrom32f(points[i]).x-320;
			point2D.y = cvPointFrom32f(points[i]).y-240;				
			imagePoints.push_back( point2D );
			//printf("Ip %f %f\n", point2D.x, point2D.y);

		}
		else if(numOfTrackingPoints==modelPoints.size()){
			CvPoint2D32f point2D;
			point2D.x = cvPointFrom32f(points[i]).x-320;
			point2D.y = cvPointFrom32f(points[i]).y-240;
			imagePoints.push_back( point2D );
			//printf("Ip %f %f\n", point2D.x, point2D.y);

		}



		//}

	}

	
	if(modelPoints.size()==numOfTrackingPoints){		
		CvPOSITObject *positObject = cvCreatePOSITObject( &modelPoints[0], static_cast<int>(modelPoints.size()) );


		//set posit termination criteria: 1000 max iterations, convergence epsilon 1.0e-5
		CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 1000, 1.0e-5 );
		
		cvPOSIT( positObject, &imagePoints[0], focus, criteria, rotation_matrix, translation_vector ); 
		cvReleasePOSITObject (&positObject);

	}
	
	//printMatrixData(rotation_matrix,translation_vector);
	
}

/**
 * This function inserts feature points according to cvFindGoodFeatures to Track
 * in the roi given by headX,headY, width, and height
 * Returns the number of points it was able to insert 
 */


int insertNewPoints(IplImage* grey, int headX,int headY,int width, int height,
		CvPoint2D32f* points){
	
	IplImage *result;
	
	// set ROI, you may use following two funs:
	cvSetImageROI( grey, cvRect( headX, headY, width, height ));

	// sub-image
	result = cvCreateImage( cvSize(width, height), grey->depth, grey->nChannels );
	
	cvCopy(grey,result);
	
	cvResetImageROI(grey); // release image ROI


	IplImage* eig = cvCreateImage( cvGetSize(result), 32, 1 );
	IplImage* temp = cvCreateImage( cvGetSize(result), 32, 1 );


	double quality = 0.01;
	double min_distance = 5;

	int numPoints = 200;//MAX_COUNT;
	cvGoodFeaturesToTrack( result, eig, temp, points, &numPoints,
			quality, min_distance, 0, 3, 0, 0.04 );	    	
	//TODO: ENABLE SUBPIX AFTER TESTS  
	//TODO: clear cvOpticalFlowPyrLK flags after  inserting new points
	/*cvFindCornerSubPix( result, points[0], numPoints,
            cvSize(win_size,win_size), cvSize(-1,-1),
            cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));*/
	cvReleaseImage( &eig );
	cvReleaseImage( &temp );

	for(int i=0;i<numPoints;i++){
		CvPoint pt = cvPointFrom32f(points[i]);
		points[i] = cvPoint2D32f(pt.x+headX,pt.y+headY);		
	}	

	return numPoints;
}


void setGLProjectionMatrix(double projectionMatrix[16]){
	double farPlane=10000.0;
	double nearPlane=1.0;
	double width = 640;
	double height = 480;
	double focalLength = EHCIFOCUS;
	projectionMatrix[0] = 2*focalLength/width;
	projectionMatrix[1] = 0.0;
	projectionMatrix[2] = 0.0;
	projectionMatrix[3] = 0.0;

	projectionMatrix[4] = 0.0;
	projectionMatrix[5] = 2*focalLength/ height;
	projectionMatrix[6] = 0.0;
	projectionMatrix[7] = 0.0;

	projectionMatrix[8] = 0;
	projectionMatrix[9] = 0;	
	projectionMatrix[10] = - ( farPlane+nearPlane ) / ( farPlane - nearPlane );;
	projectionMatrix[11] = -1.0;

	projectionMatrix[12] = 0.0;
	projectionMatrix[13] = 0.0;
	projectionMatrix[14] = -2.0 * farPlane * nearPlane / ( farPlane - nearPlane );		
	projectionMatrix[15] = 0.0;

}

#define READFROMIMAGEFILE 0
#define NUMPTS 8

IplImage *image = 0, *grey = 0, *prev_grey = 0, *pyramid = 0, *prev_pyramid = 0, *swap_temp;
const int MAX_COUNT = 500;
CvPoint2D32f* points[2] = {0,0}, *swap_points;
char* status = 0;
CvPoint upperHeadCorner = cvPoint(0,0);


int lastHeadW, lastHeadH;
int win_size = 10;

IplImage* frame;

/**
 * Returns last captured frame.
 */
IplImage* getCurrentFrame(){
	return frame;
}


CvCapture* capture = 0;
//TODO: in case different types of input are required, change this variable
int initializeCapture(){
	if(!capture){
		capture = cvCaptureFromCAM(0);	
	}
	if( !capture )
	{
		fprintf(stderr,"Could not initialize capturing...\n");
		return 0;
	}
	return 1;
}

/**
 * Updates 6 degrees of freedom head tracking.
 */

void update6dof(int headHeight, int headWidth,int initialGuess,int numberOfTrackingPoints){
	static int flags = 0;	
	int i,k;
	
	if( numberOfTrackingPoints > 0 )
	{
		cvCalcOpticalFlowPyrLK( prev_grey, grey, prev_pyramid, pyramid,
				points[0], points[1], numberOfTrackingPoints, cvSize(win_size,win_size), 3, status, 0,
				cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), flags );
		flags |= CV_LKFLOW_PYR_A_READY;
		for( i = k = 0; i < numberOfTrackingPoints; i++ )
		{

			if( !status[i] )
				continue;

			points[1][k++] = points[1][i];
			//draws points on image for debugging
			cvCircle( image, cvPointFrom32f(points[1][i]), 4, CV_RGB(0,0,0), 0, 8,0);
			cvCircle( image, cvPointFrom32f(points[1][i]), 3, CV_RGB(200,0,0), 0, 8,0);
			
		}
		numberOfTrackingPoints = k;
	}
	




	CvMatr32f rotation_matrix = new float[9];
	CvVect32f translation_vector = new float[3];


	if(numberOfTrackingPoints >=NUMPTS){
		//getPositMatrix uses points[1] obtained from cvCalcOpticalFlowPyrLK
		getPositMatrix(image,initialGuess, rotation_matrix,translation_vector,
				numberOfTrackingPoints,EHCIFOCUS,points[1],upperHeadCorner,
				headWidth,headHeight,EHCIMODELSCALE);
		updateGlPositMatrix(rotation_matrix,translation_vector);	

	}

	CV_SWAP( prev_grey, grey, swap_temp );
	CV_SWAP( prev_pyramid, pyramid, swap_temp );
	CV_SWAP( points[0], points[1], swap_points );
	
}

/**
 * Updates internal parameters so that getHeadParameters works accordingly.
 */
int myUpperHeadX,myUpperHeadY,myHeadWidth,myHeadHeight;
void updateInternalHeadPosition(int upperHeadX, int upperHeadY,int headWidth,int headHeight){
	myUpperHeadX = upperHeadX;
	myUpperHeadY = upperHeadY;
	myHeadWidth  = headWidth;
	myHeadHeight = headHeight;
}

int referenceUpperHeadX,referenceUpperHeadY,referenceHeadWidth,referenceHeadHeight;
void updateReferenceInternalHeadPosition(int upperHeadX, int upperHeadY,int headWidth,int headHeight){
	referenceUpperHeadX=upperHeadX;
	referenceUpperHeadY=upperHeadY;
	referenceHeadWidth = headWidth;
	referenceHeadHeight = headHeight;
}

/**
 * Returns reference head position in pixel dimensions.
 * Upper left is x=0, y = 0 . 
 * Head width and height are also given in pixels.
 */
void getReferenceHeadBounds(int* headRefX,int* headRefY,int* aLastHeadW,int* aLastHeadH){
	*headRefX = referenceUpperHeadX;
	*headRefY = referenceUpperHeadY;
	*aLastHeadW = referenceHeadWidth;
	*aLastHeadH = referenceHeadHeight;
}


/**
 * Returns last captured head position in pixel dimensions.
 * Upper left is x=0, y = 0 .
 * Head width and height are also given in pixels.
 *  
 */

void getHeadBounds(int* headRefX,int* headRefY,int* aLastHeadW,int* aLastHeadH){
	*headRefX = myUpperHeadX;
	*headRefY = myUpperHeadY;
	*aLastHeadW = myHeadWidth;
	*aLastHeadH = myHeadHeight;
}

/**
 * Deals with library initialization and creating debug windows.
 */

void ehciInit(){
	cvNamedWindow( "EHCI Window", 1 );	
}

int ehciLoop(int mode,int initialGuess){	
	static int numberOfTrackingPoints=0;
	int i, k, c;	
	int headWidth, headHeight,detectedHead=0;
	
	
	

	frame = 0;
	
	if(!initializeCapture())
		return 0;
	
	
	if(READFROMIMAGEFILE){
		frame = cvLoadImage( "head.jpg", 1 );
	}
	else{
		frame = cvQueryFrame( capture );
		if( !frame ){
			
			return 0;
		}
	}
	cvFlip( frame, frame, 1 );

	if( !image )
	{
		/* allocate all the buffers */
		image = cvCreateImage( cvGetSize(frame), 8, 3 );
		image->origin = frame->origin;
		grey = cvCreateImage( cvGetSize(frame), 8, 1 );
		prev_grey = cvCreateImage( cvGetSize(frame), 8, 1 );
		pyramid = cvCreateImage( cvGetSize(frame), 8, 1 );
		prev_pyramid = cvCreateImage( cvGetSize(frame), 8, 1 );
		points[0] = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(points[0][0]));
		points[1] = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(points[0][0]));
		status = (char*)cvAlloc(MAX_COUNT);
	}

	cvCopy( frame, image, 0 );
	cvCvtColor( image, grey, CV_BGR2GRAY );

	if(mode & EHCI2DFACEDETECT)
		detectedHead = getHeadPosition(image, &upperHeadCorner,&headWidth,&headHeight );
	
	
	updateInternalHeadPosition(upperHeadCorner.x,upperHeadCorner.y,
			headWidth,headHeight);

	if(initialGuess){
		//automatic initialization won't work in case face was not detected
		if((headWidth <= 0) || (headHeight<=0)) return 0;				
		if(		(upperHeadCorner.x>=0)&&(upperHeadCorner.y>=0)&&
				(upperHeadCorner.x+headWidth< cvGetSize(grey).width) && 
				(upperHeadCorner.y+headHeight< cvGetSize(grey).height)){
			numberOfTrackingPoints = insertNewPoints(grey,upperHeadCorner.x+(int)(0.15*headWidth),upperHeadCorner.y+(int)(0.15*headHeight),
					(int)(headWidth*0.7),(int)(headHeight*0.7),points[0]);	
		}
		int refX = cvPointFrom32f(points[0][0]).x - upperHeadCorner.x;
		int refY = cvPointFrom32f(points[0][0]).y - upperHeadCorner.y;
		int myLastHeadW = headWidth;
		int myLastHeadH = headHeight;
		updateReferenceInternalHeadPosition(refX,refY,myLastHeadW,myLastHeadH);
	}
	


	if((mode & EHCI6DFACEDETECT)&&(detectedHead)){
		update6dof(headHeight, headWidth, initialGuess,numberOfTrackingPoints);
	}


	cvShowImage( "EHCI Window", image );

	//main cvLoop, used to process events
	cvWaitKey(5);

	if(numberOfTrackingPoints<NUMPTS)
		return 0;
	else
		return 1;
}


/**
 * ehci cleanup code
 */
void ehciExit(){
	
	cvDestroyWindow("EHCI Window");
	cvReleaseCapture( &capture );
}


/* This function is supposed to find the
 * translation vector to the origin, but does not
 * seem to be working
 */
/*
void getTranslationToOrigin(double glPositMatrix[16]){
	//find inverse operation so that displacement can be found
	//we know that image point at reference for posit, Pio
	//is related to the world point (Pwo) through:
	//Pio = KRT Pwo
	//now, the model origin image point, during initialization is
	//Pim = KRT Pwm
	//but we don't know Pwm. In order to find it:
	//Pwm = KRT^-1 Pim
	//and then, we can find the translation:
	//Trans = Pwm - Pwo

	//OpenGl Matrixes are column major
	float rt[] = { 	glPositMatrix [0], glPositMatrix[4],glPositMatrix[8] ,glPositMatrix[12],
			glPositMatrix [1], glPositMatrix[5],glPositMatrix[9] ,glPositMatrix[13],
			glPositMatrix [2], glPositMatrix[6],glPositMatrix[10],glPositMatrix[14],
			glPositMatrix [3], glPositMatrix[7],glPositMatrix[11],glPositMatrix[15]
	};
	CvMat RT=cvMat(4,4,CV_32FC1, rt);

	float k[] = { 	projectionMatrix [0], projectionMatrix[4],projectionMatrix[8] ,projectionMatrix[12],
			projectionMatrix [1], projectionMatrix[5],projectionMatrix[9] ,projectionMatrix[13],
			projectionMatrix [2], projectionMatrix[6],projectionMatrix[10],projectionMatrix[14],
			projectionMatrix [3], projectionMatrix[7],projectionMatrix[11],projectionMatrix[15]
	};

	CvMat K = cvMat(4,4,CV_32FC1, k);

	float pio[] = { 	lastHeadW+refX,
			lastHeadH+refY,
			1,
			1
	};
	CvMat Pio = cvMat(4,1,CV_32FC1,pio);


	float pim[] = {	 (refX/320.0)*400,//(lastHeadW-320)/320.0,
			(refY/240.0)*400,//(lastHeadH-240)/240.0,
			400,
			400
	};
	printf("pim %f %f %f %f\n",pim[0],pim[1],pim[2],pim[3]);

	CvMat Pim = cvMat(4,1,CV_32FC1,pim);

	float ptest[] = {	0,
			0,
			0,
			1
	};

	CvMat Ptest = cvMat(4,1,CV_32FC1,ptest);
	CvMat* Pres =  cvCreateMat(4,1,CV_32FC1);


	CvMat* Res  = cvCreateMat(4,4,CV_32FC1);
	CvMat* invKRT  = cvCreateMat(4,4,CV_32FC1);
	cvMatMul(&K,&RT,Res);

	cvMatMul(Res,&Ptest,Pres);
	printf("Test x %f (x/w) %f y %f z %f w %f\n",cvmGet(Pres,0,0),cvmGet(Pres,0,0)/cvmGet(Pres,3,0),cvmGet(Pres,1,0),cvmGet(Pres,2,0),cvmGet(Pres,3,0));

	cvInvert(Res,invKRT);

	printf("Inverse\n");
	printf("%f %f %f %f\n",cvmGet(invKRT,0,0),cvmGet(invKRT,0,1),cvmGet(invKRT,0,2),cvmGet(invKRT,0,3));
	CvMat* Pwm = cvCreateMat(4,1,CV_32FC1);
	cvMatMul(Res,&Ptest,&Pim);
	cvMatMul(invKRT,&Pim,Pwm);
	//note that Pwo is 0,0,0, so Trans = Pwm

	float deltaX = cvmGet(Pwm,0,0);
	float deltaY = cvmGet(Pwm,1,0);
	float deltaZ = cvmGet(Pwm,2,0);
	float deltaW = cvmGet(Pwm,3,0);

	//cvReleaseMat(&RT);
	//cvReleaseMat(&K);
	//cvReleaseMat(&Pio);
	//cvReleaseMat(&Pim);
	cvReleaseMat(&Res);
	cvReleaseMat(&invKRT);
	cvReleaseMat(&Pwm);
	glBegin(GL_LINES);
	glColor3f(1.0f,1.0f,1.0);
	glVertex3f(0.0f, 0.0f,0.0f);
	glVertex3f(deltaX, deltaY,deltaZ);

	glEnd();
	printf("Delta x %f y %f z %f w %f\n",deltaX,deltaY,deltaZ,deltaW);
}*/
