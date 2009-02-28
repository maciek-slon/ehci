#include "ehci.h"
using namespace std;

IplImage *image = 0;
vector<int> ransacDeleted;



/**
 * Loads cascade xml
 */
void loadCascade(CvHaarClassifierCascade** cascade, char* fileName){

	*cascade = (CvHaarClassifierCascade*)cvLoad( fileName, 0, 0, 0 );


	if( !(*cascade) ){
		printf( "ERROR: Could not load classifier cascade. The required file '%s' should be in a 'data' subdirectory under the directory in which the application is being run.\n"
				, fileName);
		exit(1);
	}


}

/**
 * Optimized function to detect head position and draw a rectangle around it.
 * This function uses OpenCV implemented Viola-Jones algorithm with
 * Haar-like features and trained cascades. Refer to OpenCV documentation.
 */
int detect_and_draw( IplImage* small_img, int mode,double scale, CvPoint* upperHeadCorner,int* headWidth,int* headHeight,CvHaarClassifierCascade* cascade,
		CvMemStorage* storage)
{
	int i, j,detected = 0;


	cvClearMemStorage( storage );

	//if(mode & EHCI2DFACEDETECT){

	if( cascade )
	{

		double t = (double)cvGetTickCount();
		CvSeq* faces = cvHaarDetectObjects( small_img, cascade, storage,
				1.2, 2, 0
#ifdef CV_HAAR_FIND_BIGGEST_OBJECT
				|CV_HAAR_FIND_BIGGEST_OBJECT
#endif
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
	//	}



	return detected;
}

/**
 * Stores keyframe reference points. Absolute x,y frow WINDOW reference (not head reference)
 */
CvPoint2D32f* referencePoints;
int numberOfReferencePoints;

void storeReferencePoints(CvPoint2D32f* points,int numberOfTrackingPoints){
	free(referencePoints);
	referencePoints = (CvPoint2D32f*) malloc(numberOfTrackingPoints*sizeof(CvPoint2D32f));
	for(int i=0;i<numberOfTrackingPoints;i++){
		referencePoints[i]=points[i];
	}
	numberOfReferencePoints = numberOfTrackingPoints;
}

int keyFrameUpperHeadCornerX,keyFrameUpperHeadCornerY,keyFrameHeadWidth,keyFrameHeadHeight;

void storeKeyFrameHeadPoints(int upperHeadCornerX,int upperHeadCornerY, int keyHeadWidth,int keyHeadHeight){
	keyFrameUpperHeadCornerX = upperHeadCornerX;
	keyFrameUpperHeadCornerY = upperHeadCornerY;
	keyFrameHeadWidth = keyHeadWidth;
	keyFrameHeadHeight = keyHeadHeight;
}

void getKeyFrameHeadPoints(int* upperHeadCornerX,int* upperHeadCornerY, int* keyHeadWidth,int* keyHeadHeight){
	*upperHeadCornerX = keyFrameUpperHeadCornerX;
	*upperHeadCornerY = keyFrameUpperHeadCornerY;
	*keyHeadWidth = keyFrameHeadWidth;
	*keyHeadHeight = keyFrameHeadHeight;

}



/**
 * internal ehci function to get 2d head upper left corner, width and height
 * the width and height are proportional to Viola-Jones trained cascade
 * which are a little bit smaller than the real ones
 *
 * returns 0 if no head was found
 */
CvHaarClassifierCascade* headCascade=0;
CvHaarClassifierCascade* handCascade=0;
int getObjectPosition(IplImage* frame,int mode, CvPoint* upperHeadCorner,int* headWidth,int* headHeight ){

	static CvMemStorage* storage;
	int detected;
	double scale = 2.0;

	if(!headCascade){
		loadCascade(&headCascade,"data/haarcascade_frontalface_default.xml");
		loadCascade(&handCascade,"data/aGest.xml");
		storage = cvCreateMemStorage(0);
	}


	IplImage *frame_copy = 0;
	frame_copy = cvCreateImage( cvSize(frame->width,frame->height), IPL_DEPTH_8U, frame->nChannels );

	cvCopy( frame, frame_copy, 0 );


	IplImage *gray, *small_img;




	gray = cvCreateImage( cvSize(frame_copy->width,frame_copy->height), 8, 1 );
	small_img = cvCreateImage( cvSize( cvRound (frame_copy->width/scale),
			cvRound (frame_copy->height/scale)), 8, 1 );

	cvCvtColor( frame_copy, gray, CV_BGR2GRAY );
	cvResize( gray, small_img, CV_INTER_LINEAR );
	cvEqualizeHist( small_img, small_img );



	if(mode&EHCI2DFACEDETECT){
		detected = detect_and_draw(small_img,mode,scale,upperHeadCorner,headWidth,headHeight,headCascade,storage);
		cvRectangle(frame, *upperHeadCorner, cvPoint(upperHeadCorner->x + *headWidth,upperHeadCorner->y + *headHeight), cvScalar(0,0,255), 1);
	}
	if(mode&EHCI2DHANDDETECT){
		detected = detect_and_draw(small_img,mode,scale,upperHeadCorner,headWidth,headHeight,handCascade,storage);
		cvRectangle(frame, *upperHeadCorner, cvPoint(upperHeadCorner->x + *headWidth,upperHeadCorner->y + *headHeight), cvScalar(255,0,0), 1);
	}

	cvReleaseImage( &frame_copy );
	cvReleaseImage( &gray );
	cvReleaseImage( &small_img );

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

float referenceCoordinateX,referenceCoordinateY,referenceCoordinateZ;
void setReferenceCoordinate(float x, float y, float z){
	referenceCoordinateX = x;
	referenceCoordinateY = y;
	referenceCoordinateZ = z;

}

/*
 * Returns the first object point coordinate, the one the matrix refers to
 *
 */
void getReferenceCoordinate(float* x, float* y, float* z){
	*x = referenceCoordinateX;
	*y = referenceCoordinateY;
	*z = referenceCoordinateZ;
}



/*
 *
 * Returns the threshold based in a 95% inlier probability,
 * in case a gaussian distribuition for the error is used as hypothesis
 * according to Multiple View Geometry in Computer Vision
 *
 */
double getDistanceThreshold(CvMat Ma,CvMat Mp,CvMat* Mpoints,CvMat Mlook,int NUMPTS,vector<CvPoint2D32f> imagePoints){
	vector<double> distanceErrors;
	CvMat* Mr1 =  cvCreateMat(4,1,CV_32FC1);

	double errorSum = 0;
	//printf("Distances\n");
	for(int w=0;w<NUMPTS;w++){
		cvMatMul(&Ma,&Mpoints[w],Mr1);
		cvMatMul(&Mp,Mr1,Mr1);

		//fazer o look at para o posit
		cvMatMul(&Mlook,Mr1,Mr1);

		double xWinPosition =cvmGet(Mr1,0,0)/cvmGet(Mr1,3,0)*320;
		double yWinPosition =cvmGet(Mr1,1,0)/cvmGet(Mr1,3,0)*240;
		double dx = xWinPosition - imagePoints[w].x ;
		double dy = yWinPosition - (-imagePoints[w].y );//estao com sinais trocados
		double pointDistance = sqrt(dx*dx+dy*dy);
		errorSum+=pointDistance;
		distanceErrors.push_back(pointDistance);
		//printf("%3.2lf ",pointDistance);

	}
	//printf("\n");
	double deviations = 0;
	double meanError = errorSum/NUMPTS;
	for(int i=0;i<NUMPTS;i++){
		deviations+=(distanceErrors[i]-meanError)*(distanceErrors[i]-meanError);
	}
	//printf("Deviations %lf numpts %d mean error %lf errorSum %lf\n",deviations, NUMPTS,meanError,errorSum);
	double sigma = sqrt(deviations/NUMPTS);


	cvReleaseMat(&Mr1);

	return sqrt(5.99 * sigma*sigma);

}
//returns the inliers indexes in the inliers vector
//returns the sum of the distances
//in case keyframe is using this function, it plots original 2d keyframe points to new
//positions using current matrixes
double plot2dModel(CvMatr32f rotation_matrix,CvVect32f translation_vector,
		vector<CvPoint3D32f> objectPoints,IplImage *image,vector<CvPoint2D32f> imagePoints,vector<int>* inliers, int flag,CvPoint2D32f* correctedPoints ){
	if(objectPoints.size()==0) return 100000000.0;

	float a[] = {  rotation_matrix[0],  rotation_matrix[1],  rotation_matrix[2], translation_vector[0],
			rotation_matrix[3],  rotation_matrix[4],  rotation_matrix[5], translation_vector[1],
			rotation_matrix[6],  rotation_matrix[7],  rotation_matrix[8], translation_vector[2],
			0,  0,  0,  1 };

	float projectionMatrix[16];
	double farPlane=10000.0;
	double nearPlane=1.0;
	double width = 640;
	double height = 480;
	double focalLength = EHCIFOCUS;

	//TODO: trocar para os originais comentados
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
	projectionMatrix[10] = - ( farPlane+nearPlane ) / ( farPlane - nearPlane );
	projectionMatrix[11] =-2.0 * farPlane * nearPlane / ( farPlane - nearPlane );


	projectionMatrix[12] = 0.0;
	projectionMatrix[13] = 0.0;
	projectionMatrix[14]= -1.0;
	projectionMatrix[15] = 0.0;






	float pos[] = {0,0,0};

	CvMat Ma = cvMat(4, 4, CV_32FC1, a);

	CvMat Mp = cvMat(4, 4, CV_32FC1, projectionMatrix);

	int NUMPTS = objectPoints.size();
	//printf("Plot2d receiving %d\n",NUMPTS);
	int w;
	CvMat Mpoints[500];
	float pontos[500][4];


	pontos[0][0] =  0;
	pontos[0][1] =  0;
	pontos[0][2] =  0;
	pontos[0][3] =  1.0;

	pontos[1][0] =  50.0;
	pontos[1][1] =  0;
	pontos[1][2] =  0;
	pontos[1][3] =  1.0;

	pontos[2][0] =  0.0;
	pontos[2][1] =  50.0;
	pontos[2][2] =  0;
	pontos[2][3] =  1.0;


	//printf("new round\n");
	for(w=0;w<NUMPTS;w++){
		//fazendo tambem a mudanca de coordenada para o ponto 0
		//printf("Obj %+3.2f %+3.2f %+3.2f ", objectPoints[w].x, objectPoints[w].y,objectPoints[w].z);
		pontos[w][0] =  objectPoints[w].x - objectPoints[0].x;
		pontos[w][1] =  objectPoints[w].y - objectPoints[0].y;
		pontos[w][2] =  objectPoints[w].z - objectPoints[0].z;
		pontos[w][3] =  1.0;
		//printf("Obj %+3.2f %+3.2f %+3.2f\n", pontos[w][0], pontos[w][1],pontos[w][2]);

		Mpoints[w] = cvMat( 4, 1, CV_32FC1,&pontos[w]);//&objectPoints[w]);


		/*cvmSet(&Mpoints[w],0,0,objectPoints[w].x);
		cvmSet(&Mpoints[w],1,0,objectPoints[w].y);
		cvmSet(&Mpoints[w],2,0,objectPoints[w].z);
		cvmSet(&Mpoints[w],3,0,2.0);*/
	}

	float origem[4];
	origem[0] = 0 - objectPoints[0].x;
	origem[1] = 0 - objectPoints[0].y;
	origem[2] = 0 - objectPoints[0].z;
	origem[3] = 1.0;

	Mpoints[w]= cvMat( 4, 1, CV_32FC1,&origem);

	//printf("Obj- %+3.2f %+3.2f %+3.2f\n", origem[0], origem[1],origem[2]);

	float origem1[4];
	origem1[0] = 1.0 - objectPoints[0].x;
		origem1[1] = -1.0 - objectPoints[0].y;
		origem1[2] = 0.0 - objectPoints[0].z;
		origem1[3] = 1.0;

	Mpoints[w+1]= cvMat( 4, 1, CV_32FC1,&origem1);
	//printf("Obj- %+3.2f %+3.2f %+3.2f\n", origem1[0], origem1[1],origem1[2]);
	/*cvmSet(&Mpoints[w],0,0,0);//1.5 - objectPoints[0].x);
	cvmSet(&Mpoints[w],1,0,0);//-.5 - objectPoints[0].y);
	cvmSet(&Mpoints[w],2,0,0);//-.5 - objectPoints[0].z);
	cvmSet(&Mpoints[w],3,0,1.0);*/
	//printf("Test %f %f %f.\n", 1.5 - objectPoints[0].x,-.5- objectPoints[0].y, 0.0f);//-.5 - objectPoints[0].z);


	CvMat* Mr1 =  cvCreateMat(4,1,CV_32FC1);

	float up[] = {0.0 ,-1.0 , 0.0 };
	float s[] =  {-1.0 , 0.0,  0.0};
	float f[] =  { 0.0 , 0.0 , 1.0 };
	float u[] =  { 0.0 , 1.0 , 0.0 };

	float look[] = {s[0], s[1], s[2], 0,
			u[0], u[1], u[2], 0,
			-f[0],-f[1],-f[2],0,
			0   ,  0   , 0   ,1};
	CvMat Mlook = cvMat(4, 4, CV_32FC1, look);

	double distance = 0;
	double threshold = getDistanceThreshold(Ma,Mp,Mpoints,Mlook,NUMPTS,imagePoints);
	//printf("Sigma = %lf\n",threshold);

	//TODO: find out why some sigmas are nan

	if(!isnan(threshold)){
		for(w=0;w<NUMPTS;w++){
			//for(w=0;w<=2;w++){
			cvMatMul(&Ma,&Mpoints[w],Mr1);
			cvMatMul(&Mp,Mr1,Mr1);


			//fazer o look at para o posit
			cvMatMul(&Mlook,Mr1,Mr1);

			/*if(w==NUMPTS){
			printf("CvModelView Matrix:\n");
			for(int i=0;i<4;i++){
				for(int j=0;j<4;j++){
					printf("%lf ",cvmGet(&Ma,i,j));//
				}
				printf("\n");
			}
			printf("\n");

			CvMat* Mpro =  cvCreateMat(4,4,CV_32FC1);
			cvMatMul(&Mlook,&Mp,Mpro);

			printf("CvProjection Matrix:\n");
			for(int i=0;i<4;i++){
				for(int j=0;j<4;j++){
					printf("%lf ",cvmGet(Mpro,i,j));//
				}
				printf("\n");
			}
			printf("\n");

		}*/


			double xWinPosition =cvmGet(Mr1,0,0)/cvmGet(Mr1,3,0)*320;
			double yWinPosition =cvmGet(Mr1,1,0)/cvmGet(Mr1,3,0)*240;
			double dx = xWinPosition - imagePoints[w].x ;
			double dy = yWinPosition - (-imagePoints[w].y );//estao com sinais trocados

			double pointDistance = sqrt(dx*dx+dy*dy);


			//if(pointDistance>distance) distance = pointDistance;
			distance += pointDistance;


			if(pointDistance>RANSAC_DISTANCE_THRESHOLD){//threshold){//
				//printf("Outlier %d %lf\n",w,pointDistance);
			}
			else{
				//cvCircle( image,cvPoint(xWinPosition+320,-yWinPosition+240), 1, CV_RGB(0,200,200) , -1, 8,0);
				inliers->push_back(w);
			}
			//printf("%3.2lf %3.2lf ipx %3.2lf ipy %3.2lf dx %3.2lf dy %3.2lf pdist %3.2lf dist %3.2lf\n",xWinPosition,yWinPosition,imagePoints[w].x,imagePoints[w].y,dx,dy,pointDistance,distance);

			//		printf("coord %f %f ",cvmGet(&Mpoints[w],0,0),cvmGet(&Mpoints[w],1,0));
			//		printf("coord %f %f\n",cvmGet(Mr1,0,0),cvmGet(Mr1,1,0));

			//Debug information
			//cvRectangle(image,cvPoint(0,0),cvPoint(160,120),CV_RGB(0,150,0),2,0,0);
			if(flag){
				cvCircle( image,cvPoint(xWinPosition+320,-yWinPosition+240), w==0?4:1, CV_RGB(0,0,200) , -1, 8,0);
				if(pointDistance>RANSAC_DISTANCE_THRESHOLD){
					cvCircle( image,cvPoint(xWinPosition+320,-yWinPosition+240), 3, CV_RGB(180,36,255) , -1, 8,0);
				}
				correctedPoints[w]= cvPoint2D32f(xWinPosition+320,-yWinPosition+240);

				if(w>=NUMPTS){
					//printf("Model origin %lf %lf\n",xWinPosition+320,-yWinPosition+240);
					cvCircle( image,cvPoint(xWinPosition+320,-yWinPosition+240), 5, CV_RGB(0,255,0) , -1, 8,0);
				}
			}


			//printf("%3.3lf %3.3lf %3.3lf %3.3lf (%3.3lf,%3.3lf,%3.3lf,%3.3lf)\n",cvmGet(Mr1,0,0),cvmGet(Mr1,1,0),
			//				cvmGet(Mr1,2,0),cvmGet(Mr1,3,0),cvmGet(&Mpoints[w],0,0),cvmGet(&Mpoints[w],1,0),cvmGet(&Mpoints[w],2,0),cvmGet(&Mpoints[w],3,0));
			//cvCircle( image, cvPoint(160+1000*cvmGet(Mr1,0,0)/cvmGet(Mr1,2,0),120+1000*cvmGet(Mr1,1,0)/cvmGet(Mr1,2,0)), w==0?8:3, CV_RGB(0,0,200) , -1, 8,0);

		}
		//printf("\n");
		//printf("Sum of distances %lf\n",distance);
	}

	cvReleaseMat(&Mr1);

	return distance;

}


vector<int> getRandomSet(int sampleSize, int setSize){
	if(sampleSize<setSize){
		printf("Set size too small in RANSAC\n");
		exit(0);
	}
	vector<int> lista;
	for(int i=0;i<sampleSize;i++){
		lista.push_back(i);
	}
	vector<int> randomSet;
	while(randomSet.size()<setSize){
		int pos = rand()%lista.size();
		randomSet.push_back(lista[pos]);
		lista.erase(lista.begin()+pos);
	}
	/*printf("Random set: ");
	for(int i=0;i<randomSet.size();i++){
		printf("%d ",randomSet[i]);
	}
	printf("\n");*/
	return randomSet;

}

//TODO: correct this function and ransac() so that the reference point can be an outlier

int getMatrixUsingPosit(vector<CvPoint2D32f> imagePoints, vector<CvPoint3D32f> objectPoints,
		vector<int> selectedPoints,int focus, CvMatr32f rotation_matrix, CvVect32f translation_vector){

	vector<CvPoint3D32f> chosenPoints;
	vector<CvPoint2D32f> chosenImagePoints;
	//do not change the first one, because it's the reference
	chosenPoints.push_back(objectPoints[0]);
	chosenImagePoints.push_back(imagePoints[0]);

	for(int i=1;i<selectedPoints.size();i++){
		int index = selectedPoints[i];
		if(index<objectPoints.size()){
			chosenPoints.push_back(objectPoints[index]);
			chosenImagePoints.push_back(imagePoints[index]);
			//printf("%f %f\n",imagePoints[index].x,imagePoints[index].y);
		}

	}

	if(selectedPoints.size()>=4){

		CvPOSITObject *positObject = cvCreatePOSITObject( &chosenPoints[0], static_cast<int>(chosenPoints.size()) );
		//set posit termination criteria: 1000 max iterations, convergence epsilon 1.0e-5

		CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 1000, 1.0e-5 );
		cvPOSIT( positObject, &chosenImagePoints[0], focus, criteria, rotation_matrix, translation_vector );

		cvReleasePOSITObject (&positObject);
	}
	else{
		printf("Error: no posit: number of points %d\n",chosenPoints.size());
		return 1;//error

	}

	return 0;

}


/**
 * This function is implemented as in 'Multiple View Geometry in Computer Vision'
 * 1 - Randomly select samples from the 2d tracked set
 * 2 - Use POSIT from this set
 * 3 - Use distance to filter outliers
 * 4 - Keep doing until some threshold of inliers size is found
 *
 * correctedPoints will update the points vector without the outliers, and newNumPoints will return the number of inliers
 */

int ransac( vector<CvPoint2D32f> imagePoints, vector<CvPoint3D32f> objectPoints,
		CvMatr32f rotation_matrix, CvVect32f translation_vector, int focus,IplImage* myImage,
		CvPoint2D32f* correctedPoints,int* numberOfInliers){

	vector<int> bestSet;
	double minDist=10000000;
	CvMatr32f rotationTestMatrix = new float[9];
	CvVect32f translationTestVector = new float[3];
	int bestSize = 0;

	//printf("Ransac using %f %f %f\n",objectPoints[0].x,objectPoints[0].y,			objectPoints[0].z);

	for(int k=0;k<RANSAC_ITERATIONS;k++){
		//only choose first 4 points:
		vector<int> randomPoints = getRandomSet(imagePoints.size(),RANSAC_SAMPLES);
		getMatrixUsingPosit(imagePoints,objectPoints,randomPoints,focus,rotationTestMatrix, translationTestVector);

		vector<int> inliers;
		double distance = plot2dModel(rotationTestMatrix ,translationTestVector,objectPoints,myImage,imagePoints,&inliers,0,NULL);
		//printf("Found %d inliers\n",inliers.size());
		if(inliers.size()>bestSize){
			//if(distance<minDist){
			bestSize = inliers.size();
			//minDist = distance;
			bestSet.clear();
			ransacDeleted.clear();
			int dCount = 0;
			for(int i=0;i<inliers.size();i++){
				if(dCount != inliers[i]){
					ransacDeleted.push_back(dCount);
					dCount++;
				}

				bestSet.push_back(inliers[i]);
				dCount++;
			}
			while(dCount <imagePoints.size()){
				ransacDeleted.push_back(dCount);
				dCount++;
			}
		}
	}

	delete [] rotationTestMatrix;
	delete [] translationTestVector;

	//update corrected points
	*numberOfInliers = bestSet.size();
	//printf("Old %d New %d\n",imagePoints.size(),bestSet.size());
	//printf("Trying to update points\n");
	for(int i=0;i<bestSet.size();i++){
		//TODO: check if this is necessary
	//	if(i==0) printf("Updated points\n");
		CvPoint2D32f centeredPoint =imagePoints[bestSet[i]];
		centeredPoint.x +=320;
		centeredPoint.y +=240;
		correctedPoints[i] = centeredPoint;
	}
	//printf("Deleted %d\n",ransacDeleted.size());
	for(int i=0;i<ransacDeleted.size();i++){
		printf("ehci: Deleted %d\n",ransacDeleted[i]);
		CvPoint2D32f centeredPoint = imagePoints[ransacDeleted[i]];
		cvCircle( image,cvPoint(centeredPoint.x+320,centeredPoint.y+240) , 4, CV_RGB(200,200,0), 2, 8,0);
	}



	//generate rot and trans matrixes with best inliers
	//now use the bestSet to retrieve the rotation and translation matrixes
	return getMatrixUsingPosit(imagePoints,objectPoints,bestSet,focus,rotation_matrix, translation_vector);
}




void addPointCenteredReference(std::vector<CvPoint2D32f> & imagePoints,CvPoint2D32f point){
	CvPoint2D32f point2D;
				point2D.x = cvPointFrom32f(point).x-320;
				point2D.y = cvPointFrom32f(point).y-240;
				imagePoints.push_back( point2D );
}

//converts to the sinusoidal model
//points should be relative to upper left head corner
void get3dModelCoordinates(int px,int py,float *fx,float* fy,float* fz, int headWidth, int headHeight){
	*fx = (1.6667 * px/(1.0*headWidth)) - 0.332;
	*fy = (1.6667 * py/(1.0*headHeight)) - 0.332;
	*fz = .5*sin(*fx*3.141593);
}

IplImage* generatedImage;
int ehciLoop(int mode,int initialGuess,IplImage* createdImage){
	generatedImage = createdImage;
	ehciLoop(mode,initialGuess);
}



std::vector<CvPoint3D32f> modelPoints;

/**
 * This function will retrieve the rotation and translation matrixes
 * using the POSIT algorithm
 * In case the initialGuess parameter is set to 1, the algorithm will
 * map points to the sinusoidal head, else it will only track to the original
 * ones. In a future version, this function should also map new points back
 * to the current head position
 *
 */

void getPositMatrix(IplImage* myImage,int initialGuess, CvMatr32f rotation_matrix, CvVect32f translation_vector,
		int numOfTrackingPoints,int focus,CvPoint2D32f* points, CvPoint upperHeadCorner,
		int headWidth, int headHeight, float modelScale, int* newNumberOfPoints){

	int i;


	std::vector<CvPoint2D32f> imagePoints;

	//will need to rebuild the model either way because of ransac
	if(initialGuess){
		modelPoints.clear();
	}

	//setInitialRTMatrix();
	int ransacInAction = numOfTrackingPoints < modelPoints.size();
	//printf("Posit receiving %d points. RansacInAction? %d\n",numOfTrackingPoints,ransacInAction);

	if(ransacInAction && (ransacDeleted.size()>0)){
		//it seems ransac has removed some outliers
		//we need to remove modelPoints that don't fit anymore
		printf("Ransac in action: updating modelPoints tp %d mp %d\n",
				numOfTrackingPoints,modelPoints.size());
		int tCount = 0;
		int target = ransacDeleted[tCount];

		std::vector<CvPoint3D32f> updatedModelPoints;

		for(int i=0;i<modelPoints.size();i++){
			if(i==target){
				printf("Erasing %d\n",i);
				//skip this

				//update target
				if(tCount<ransacDeleted.size()-1)
					target = ransacDeleted[++tCount];

			}
			else{
				updatedModelPoints.push_back(modelPoints[i]);
			}
		}
		printf("Old size %d new size %d\n",modelPoints.size(),updatedModelPoints.size());
		modelPoints.clear();
		for(int i=0;i<updatedModelPoints.size();i++){
			modelPoints.push_back(updatedModelPoints[i]);
		}




	}

	for(int i=0;i<numOfTrackingPoints;i++){
		float myPixel[4];

		int px = cvPointFrom32f(points[i]).x - upperHeadCorner.x ;
		int py = cvPointFrom32f(points[i]).y - upperHeadCorner.y ;

		cvCircle( image, cvPointFrom32f(points[i]), 3, CV_RGB(0,0,200), 0, 8,0);

		//glReadPixels(px,480-py,1,1,GL_RGBA,GL_FLOAT,&myPixel);

		//will need to rebuild the model either way because of ransac
		if(initialGuess){
			//printf("initial guessing\n");
			float fx,fy,fz;
			get3dModelCoordinates(px,py,&fx,&fy,&fz,headWidth, headHeight);

			//cos((px-0.5*headWidth)/headWidth * 1.2 *3.141593);
			//printf("px %d py %d hw %d hh %d fx %f ifx %d fy %f fz %f\n",px,py,headWidth,headHeight,fx,int(fx*modelScale),fy,fz);

			/*modelPoints.push_back(   cvPoint3D32f( 	(int)(fx * modelScale),
					-(int)(fy * modelScale),
					(int)(fz * modelScale)));*/

			modelPoints.push_back(   cvPoint3D32f( 	(fx * modelScale),
					-(fy * modelScale),
					(fz * modelScale)));
			if(i==0){
				setReferenceCoordinate((fx * modelScale), -(fy * modelScale),
						(fz * modelScale));
			}

			/*modelPoints.push_back(   cvPoint3D32f( 	(int)(px/(1.0*headWidth)*cScale),
					-(int)(py/(1.0*headHeight)*cScale),
					(int)(1.1*cScale*sin(px*3.141593/headWidth)) ));*/
			//(px<headWidth/2.0)?(int)((headWidth/4.0)*(px/(headWidth/2.0))):(int)((headWidth/4.0)*((headWidth -px)/(1.0*headWidth)))


			addPointCenteredReference(imagePoints,points[i]);
			//printf("Ip %f %f\n", point2D.x, point2D.y);

		}
		else {
			//was: else  if(numOfTrackingPoints==modelPoints.size()){
			addPointCenteredReference(imagePoints,points[i]);
			//printf("Ip %f %f\n", point2D.x, point2D.y);

		}



		//}

	}


	//was: if(modelPoints.size()==numOfTrackingPoints){
	if(modelPoints.size()>0){
		//printf("Updating\n");
		if(USE_RANSAC){
			int inliersNumber;
			ransac(imagePoints,modelPoints,rotation_matrix, translation_vector,focus,myImage,points,&inliersNumber);
			*newNumberOfPoints = inliersNumber;
		}
		else{

			CvPOSITObject *positObject = cvCreatePOSITObject( &modelPoints[0], static_cast<int>(modelPoints.size()) );
			//set posit termination criteria: 1000 max iterations, convergence epsilon 1.0e-5
			CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 1000, 1.0e-5 );

			cvPOSIT( positObject, &imagePoints[0], focus, criteria, rotation_matrix, translation_vector );
			cvReleasePOSITObject (&positObject);
			*newNumberOfPoints = numOfTrackingPoints;
		}

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

IplImage *grey = 0, *prev_grey = 0, *pyramid = 0, *prev_pyramid = 0, *swap_temp;
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
CvVideoWriter* videoWriter = 0;
//TODO: in case different types of input are required, change this variable
int initializeCapture(){
	if(!capture){

		if(strcmp(movieFile,"NO")==0){
			capture = cvCaptureFromCAM(chosenCamera);
		}
		else{
			//printf("Trying to open %s\n",movieFile);
			char temp[200];
			sprintf(temp,"%s",movieFile);

			//cvCreateFileCapture parameter is const char
			capture = cvCreateFileCapture(temp);
		}
	}
	if( !capture )
	{
		fprintf(stderr,"Could not initialize capturing...\n");
		return 0;
	}
	return 1;
}

//converts a 2d texture point to a 3d world point using current glMatrixes
//2d points should be relative to upper left head corner

void texture2world( CvPoint3D32f* worldCoordinate, int headWidth, int headHeight, int upperHeadx, int upperHeady,
		int numberOfTrackingPoints, CvPoint2D32f* points,IplImage* myImage){

	CvMatr32f rotationMatrix = new float[9];
	CvVect32f translationVector = new float[3];

	rotationMatrix[0] = glPositMatrix[0];
	rotationMatrix[1] = glPositMatrix[4];
	rotationMatrix[2] = glPositMatrix[8];
	rotationMatrix[3] = glPositMatrix[1];
	rotationMatrix[4] = glPositMatrix[5];
	rotationMatrix[5] = glPositMatrix[9];
	rotationMatrix[6] = glPositMatrix[2];
	rotationMatrix[7] = glPositMatrix[6];
	rotationMatrix[8] = glPositMatrix[10];


	translationVector[0] = glPositMatrix[12];
	translationVector[1] = glPositMatrix[13];
	translationVector[2] = glPositMatrix[14];

	vector<CvPoint3D32f> modelPoints;
	vector<CvPoint2D32f> vectorPoints;

	//printf("Head coord %d %d\n",upperHeadx, upperHeady);
	for(int i=0;i<numberOfReferencePoints;i++){
			int px = cvPointFrom32f(referencePoints[i]).x - upperHeadx ;
			int py = cvPointFrom32f(referencePoints[i]).y - upperHeady ;

			float fx,fy,fz;

			get3dModelCoordinates(px,py,&fx,&fy,&fz,headWidth, headHeight);
		//	printf("From %d %d to %+5.5f %+5.5f %+5.5f\n",px, py, fx,fy,fz);
			modelPoints.push_back( cvPoint3D32f(fx, -fy , fz ));
			addPointCenteredReference(vectorPoints,referencePoints[i]);
	}

	/*printf("Matrix headWidth %d headHeight %d upperHeadx %d upperHeady %d\n",headWidth,
			headHeight, upperHeadx, upperHeady);*/

	vector<int> inliers;
	//TODO: refactor plot2dModel so that ransac and this function can use it differently
	plot2dModel(rotationMatrix ,translationVector,modelPoints,myImage,vectorPoints,&inliers,1,points);
	printf("Keyframe original %d inliers %d\n",modelPoints.size(),inliers.size());

/*
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

*/


}

/**
 * Updates 6 degrees of freedom head tracking.
 * returns the newNumberOfTrackingPoints, because they might have changed if ransac is being used
 */
static int bootstrap=0;
int update6dof(int headHeight, int headWidth,int initialGuess,int numberOfTrackingPoints,int mode){
	static int flags = 0;
	int i,k;

	if(mode == EHCI6DFACEDETECTKEYFRAME && bootstrap){//generatedImage && !initialGuess && bootstrap){
		//printf("Third\n");
		//TODO: clean worldCoordinate
		CvPoint3D32f worldCoordinate;
		int headRefX,headRefY,lastHeadW,lastHeadH;
		getKeyFrameHeadPoints(&headRefX,&headRefY,&lastHeadW,&lastHeadH);
		texture2world(&worldCoordinate,lastHeadW,lastHeadH,headRefX,
				headRefY, numberOfTrackingPoints,points[0], image);
		//printf("Modifying points!\n");
	}

	if( numberOfTrackingPoints > 0 )
	{
		if(mode ==EHCI6DFACEDETECTKEYFRAME && generatedImage){
			//printf("Second\n");
			//should make optical flow from generated image
			//printf("Enabling\n");
			bootstrap=1;
			//TODO: use only the generated image, not the background that's available

			cvCvtColor( generatedImage, prev_grey, CV_BGR2GRAY );
			cvFlip( prev_grey, prev_grey,1 );

			//need to rotate,translate points here

			cvCalcOpticalFlowPyrLK( prev_grey, grey, prev_pyramid, pyramid,
					points[0], points[1], numberOfTrackingPoints, cvSize(20,20), 3, status, 0,
					cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), flags );
			//cvShowImage("PrevGrey",prev_grey);
			//cvShowImage("Grey",grey);

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
		else{
			//printf("First\n");
			cvCalcOpticalFlowPyrLK( prev_grey, grey, prev_pyramid, pyramid,
					points[0], points[1], numberOfTrackingPoints, cvSize(win_size,win_size), 3, status, 0,
					cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), flags );
			//cvShowImage("PrevGrey",prev_grey);
			//cvShowImage("Grey",grey);

			flags |= CV_LKFLOW_PYR_A_READY;
			for( i = k = 0; i < numberOfTrackingPoints; i++ )
			{

				if( !status[i] )
					continue;


				//draws points on image for debugging
				//cvCircle( image, cvPointFrom32f(points[1][i]), 4, CV_RGB(0,0,0), 0, 8,0);
				///cvCircle( image, cvPointFrom32f(points[1][i]), 3, CV_RGB(200,0,0), 0, 8,0);

				/*double dx = cvPointFrom32f(points[1][i]).x - cvPointFrom32f(points[0][i]).x;
				double dy = cvPointFrom32f(points[1][i]).y - cvPointFrom32f(points[0][i]).y;
				double dist = dx*dx + dy*dy;

				if(sqrt(dist)>20)
					continue;*/

				points[1][k++] = points[1][i];

				cvLine(image,cvPointFrom32f(points[0][i]),cvPointFrom32f(points[1][i]),CV_RGB(200,0,0),1);



			}
			//cvWaitKey(0);
			numberOfTrackingPoints = k;
		}
	}



	CvMatr32f rotation_matrix = new float[9];
	CvVect32f translation_vector = new float[3];

	int newNumberOfPoints=numberOfTrackingPoints;

	if(numberOfTrackingPoints >=NUMPTS){
		//getPositMatrix uses points[1] obtained from cvCalcOpticalFlowPyrLK
		//if ransac is used, some points will be discarded and numberOfTrackinPoints is changed

		getPositMatrix(image,initialGuess, rotation_matrix,translation_vector,
				numberOfTrackingPoints,EHCIFOCUS,points[1],upperHeadCorner,
				headWidth,headHeight,EHCIMODELSCALE,&newNumberOfPoints);
		//printf("Old %d New %d\n",numberOfTrackingPoints,newNumberOfPoints);
		updateGlPositMatrix(rotation_matrix,translation_vector);

	}


	CV_SWAP( prev_grey, grey, swap_temp );
	CV_SWAP( prev_pyramid, pyramid, swap_temp );
	CV_SWAP( points[0], points[1], swap_points );
	return newNumberOfPoints;

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

int readInt(char* value){
	int val;
	sscanf(value,"%d",&val);
	return val;
}

double readDouble(char* value){
	double val;
	sscanf(value,"%lf",&val);
	return val;
}
string readString(char* value){
	string ans = "";
	char temp[200];
	sscanf(value,"%s",temp);
	ans = (string) temp;
	return ans;
}

//TODO: use an xml and some parsing
void readParameters(){
	FILE* in = fopen("data/config.ini","r");
	if(in){

		char key[200],value[200];
		while(fscanf(in,"%s%s",key,value)!=EOF){

			if(strcmp(key,"USE_RANSAC")==0){
				USE_RANSAC = readInt(value);
			}
			else if(strcmp(key,"RANSAC_SAMPLES")==0){
				RANSAC_SAMPLES = readInt(value);
			}
			else if(strcmp(key,"RANSAC_DISTANCE_THRESHOLD")==0){
				RANSAC_DISTANCE_THRESHOLD = readDouble(value);
			}
			else if(strcmp(key,"RANSAC_ITERATIONS")==0){
				RANSAC_ITERATIONS = readInt(value);
			}
			else if(strcmp(key,"CAMERA_INDEX")==0){
				chosenCamera = readInt(value);
			}
			else if(strcmp(key,"OPEN_MOVIE")==0){
				movieFile = (char*)(readString(value)).c_str();
			}

		}

		/*printf("RAN %d SAMP %d DIST %lf ITER %d\n"
				,USE_RANSAC,RANSAC_SAMPLES,RANSAC_DISTANCE_THRESHOLD,
				RANSAC_ITERATIONS);*/
		fclose(in);
	}
	else{
		char movie[] = "NO";
		movieFile = (char*) movie;
	}

}

/**
 * Deals with library initialization and creating debug windows.
 */

void ehciInit(){
	cvNamedWindow( "EHCI Window", CV_WINDOW_AUTOSIZE );
	//cvNamedWindow( "PrevGrey", CV_WINDOW_AUTOSIZE );
	//cvNamedWindow( "Grey", CV_WINDOW_AUTOSIZE );

	readParameters();

}



int ehciLoop(int mode,int initialGuess){

	//main cvLoop, used to process events
	cvWaitKey(5);
	//cvWaitKey(0);


	static int numberOfTrackingPoints=0;
	int i, k, c;
	int headWidth, headHeight,detectedHead=0;
	CvPoint upperHandCorner;
	int handWidth,handHeight,detectedHand=0;

	readParameters();

	frame = 0;


	if(!initializeCapture())
		return 0;


	if(READFROMIMAGEFILE){
		frame = cvLoadImage( "head.jpg", 1 );
	}
	else{
		frame = cvQueryFrame( capture );

		if( !frame ){
			//TODO: in case it's a movie, it needs to finish here
			printf("Couldn't grab frame\n");
			exit(0);
			//return 0;
			//exit(0);
			//TODO: loop movie... isn't safe... correct this
			capture=0;
			initializeCapture();
			frame = cvQueryFrame( capture );
			//return 0;
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


	detectedHead = getObjectPosition(image, mode,&upperHeadCorner,&headWidth,&headHeight );

	//TODO: refactor initialGuess code below to work with upperHandCorner, handWidth, and handHeight
	//detectedHand = getObjectPosition(image, EHCI2DHANDDETECT,&upperHandCorner,&handWidth,&handHeight );


	updateInternalHeadPosition(upperHeadCorner.x,upperHeadCorner.y,
			headWidth,headHeight);


	if(initialGuess){
		//automatic initialization won't work in case face was not detected
		//TODO: correct this return so that cvShowImage( "EHCI Window", image ) always happens
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

		//create keyframe texture for first frame
		if((mode == EHCI6DFACEDETECTKEYFRAME) &&(image)){

			storeReferencePoints(points[0],numberOfTrackingPoints);
			storeKeyFrameHeadPoints(upperHeadCorner.x,upperHeadCorner.y,headWidth,headHeight);
			int texWidth = (int)(headWidth*0.7);
			int texHeight = (int)(headHeight*0.7);
			textureCreated = 1;
			//TODO: verify why some different sizes don't work
			texWidth = texWidth%2==0?texWidth:texWidth+1;
			texHeight = texHeight%2==0?texHeight:texHeight+1;
			texWidth = ((texWidth>>2)<<2);
			texHeight = ((texHeight>>2)<<2);
			glTexture.sizeX = texWidth;
			glTexture.sizeY = texHeight;


			int texUpperX = upperHeadCorner.x+(headWidth-texWidth)/2;
			int texUpperY = upperHeadCorner.y + (headHeight-texHeight)/2;

			/*printf("Texwidth,height %d %d texUpperX %d texUpperY %d Head %d %d hw %d hh %d\n",texWidth,
							texHeight,texUpperX,texUpperY,refX,refY,myLastHeadW,myLastHeadH);*/

			if( (texUpperX<=image->width) && (texUpperX >=0) &&
				(texUpperY<=image->height) && (texUpperY >=0) &&
				(texWidth <=image->width) && (texHeight <=image->height))
				cvSetImageROI(image,cvRect(texUpperX , texUpperY, texWidth, texHeight));
			else
				return 0;




			glTexture.data = (uchar*)(malloc(texWidth*texHeight*3*sizeof(uchar)));
			int dataCounter = 0;
			for( int y=texUpperY; y<texUpperY+texHeight; y++ ) {
			  uchar* ptr = (uchar*) (
					  image->imageData + y * image->widthStep +
					  3*texUpperX);

			  for( int x=0; x<texWidth; x++ ) {
				 if(y==texUpperY && x==0) printf("%d %d %d\n",ptr[3*x],ptr[3*x+1],ptr[3*x+2]);
				 glTexture.data[dataCounter++] = ptr[3*x+0];
				 glTexture.data[dataCounter++] = ptr[3*x+1];
				 glTexture.data[dataCounter++] = ptr[3*x+2];
			  }
			}
			cvAddS(image, cvScalar(100),image);
			cvResetImageROI(image);


		}

	}

	int newNumberOfTrackingPoints=0;
	if(initialGuess) { printf("Disabling\n");bootstrap=0;generatedImage=NULL;}
	if((mode & EHCI6DFACEDETECT) || (mode & EHCI6DHANDDETECT)){
		//numberOfTrackingPoints = update6dof(headHeight, headWidth, initialGuess,numberOfTrackingPoints,mode);
		newNumberOfTrackingPoints = update6dof(headHeight, headWidth, initialGuess,numberOfTrackingPoints,mode);
	}
	else if (mode == EHCI6DFACEDETECTKEYFRAME){
		//numberOfTrackingPoints = update6dof(headHeight, headWidth, initialGuess,numberOfTrackingPoints,mode);
		newNumberOfTrackingPoints = update6dof(headHeight, headWidth, initialGuess,numberOfTrackingPoints,mode);
	}


	cvShowImage( "EHCI Window", image );

	if(numberOfTrackingPoints<NUMPTS)
		return 0;
	else{
		numberOfTrackingPoints = newNumberOfTrackingPoints;
		return 1;
	}
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
