#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <ctype.h>

#include <vector>

IplImage *image = 0, *grey = 0, *prev_grey = 0, *pyramid = 0, *prev_pyramid = 0, *swap_temp;
CvMatr32f rotation_matrix = new float[9];
CvVect32f translation_vector = new float[3];
float vertices[4719];
int model=0;

	float theta = 3.1415;
/*	float myRot[] = { 1,  0,  0,  			translation_vector[0],
	               0, cos(theta),  -sin(theta),  	translation_vector[1],
	               0, sin(theta),   cos(theta),  	translation_vector[2],
	               0,  0,  0,  0 };*/

float myRoty[] = { cos(theta),   0,  sin(theta),   0,
	               0, 	1,  	0,  	  0,
	         -sin(theta),   0,   cos(theta),  0,
	               0,  0,  0,  1 };

float gama = 3.1415;
float myRotz[] = { cos(gama),  -sin(gama),    0,  +30,
	           sin(gama),   cos(gama),    0,  50,
	               0, 	0,  	1,  	    +50,
	               0,  0,  0,  1 };




#define NUMPTS 8
float scale = 1.0;
float points3d[NUMPTS][4]={ {0,0,0,1},{100*scale,0,0,1},{75*scale,100*scale,0,1},{25*scale,100*scale,0,1},
			       {40*scale,40*scale,-10*scale,1},{60*scale,40*scale,-10*scale,1},
			       {60*scale,60*scale,-10*scale,1},{40*scale,60*scale,-10*scale,1}
};//, {0,0,-50}};//,{60,60,10,1},{40,60,10,1}};*/

int win_size = 10;
const int MAX_COUNT = 500;
CvPoint2D32f* points[2] = {0,0}, *swap_points;
char* status = 0;
int count = 0;
int need_to_init = 0;
int night_mode = 0;
int flags = 0;
int add_remove_pt = 0;
CvPoint pt;


void on_mouse( int event, int x, int y, int flags, void* param )
{
    if( !image )
        return;

    if( image->origin )
        y = image->height - y;

    if( event == CV_EVENT_LBUTTONDOWN )
    {
        pt = cvPoint(x,y);
        add_remove_pt = 1;
    }
}

void loadVertices(){
	int pos = 0;
	FILE* in = fopen("vertices.txt","r");
	while(fscanf(in,"%f",&vertices[pos++])!=EOF);
}
int main( int argc, char** argv )
{
    CvCapture* capture = 0;
    loadVertices();
    if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
        capture = cvCaptureFromCAM( argc == 2 ? argv[1][0] - '0' : 0 );
    else if( argc == 2 )
        capture = cvCaptureFromAVI( argv[1] );

    if( !capture )
    {
        fprintf(stderr,"Could not initialize capturing...\n");
        return -1;
    }

    /* print a welcome message, and the OpenCV version */
    printf ("Welcome to lkdemo, using OpenCV version %s (%d.%d.%d)\n",
	    CV_VERSION,
	    CV_MAJOR_VERSION, CV_MINOR_VERSION, CV_SUBMINOR_VERSION);

    printf( "Hot keys: \n"
            "\tESC - quit the program\n"
            "\tr - auto-initialize tracking\n"
            "\tc - delete all the points\n"
            "\tn - switch the \"night\" mode on/off\n"
            "To add/remove a feature point click it\n" );

    cvNamedWindow( "6dofHead", 0 );
    //cvNamedWindow( "Harris", 0 );
    cvSetMouseCallback( "6dofHead", on_mouse, 0 );
    int gCount = 0;
    for(;;)
    {	
        IplImage* frame = 0;
        int i, k, c;
	gCount++;
	if(gCount>=30 && gCount <=30+NUMPTS-1){
		int source = 200;
		if(gCount==30) on_mouse(CV_EVENT_LBUTTONDOWN,source    ,source,0,NULL);
		if(gCount==31) on_mouse(CV_EVENT_LBUTTONDOWN,source+100,source,0,NULL);
		if(gCount==32) on_mouse(CV_EVENT_LBUTTONDOWN,source +75,source+100,0,NULL);
		if(gCount==33) on_mouse(CV_EVENT_LBUTTONDOWN,source +25,source+100,0,NULL);
		if(gCount==34) on_mouse(CV_EVENT_LBUTTONDOWN,source +40,source +40,0,NULL);
		if(gCount==35) on_mouse(CV_EVENT_LBUTTONDOWN,source +60,source +40,0,NULL);
		if(gCount==36) on_mouse(CV_EVENT_LBUTTONDOWN,source +60,source +60,0,NULL);
		if(gCount==37) on_mouse(CV_EVENT_LBUTTONDOWN,source +40,source +60,0,NULL);

	}

        frame = cvQueryFrame( capture );
        if( !frame )
            break;
//IplImage* src = frame1;
//IplImage* frame = cvCreateImage( cvSize(250,250), 8, 1 );


//frame = (IplImage*)cvGetSubRect( (CvMat*)src, (CvMat*)frame, cvRect(160,60,250,250));


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
            flags = 0;
        }

        cvCopy( frame, image, 0 );
        cvCvtColor( image, grey, CV_BGR2GRAY );

        if( night_mode )
            cvZero( image );

	if(count>10){
		for(i=0;i<count;i++){
			cvCircle( image, cvPointFrom32f(points[0][i]), 3, CV_RGB(0,0,100+30*i), -1, 8,0);
		}
	}
        
        if( need_to_init )
        {
            /* automatic initialization */


            IplImage* eig = cvCreateImage( cvGetSize(grey), 32, 1 );
            IplImage* temp = cvCreateImage( cvGetSize(grey), 32, 1 );

	    printf("Corner detection\n");
	    IplImage* harrisResult = cvCreateImage(cvGetSize(frame),IPL_DEPTH_32F, 1);//cvCloneImage(image);
	    printf("iplgrey %d iplharris %d IPL_DEPTH_8U %d\n",grey->depth,harrisResult->depth,IPL_DEPTH_8U);
	    cvCornerHarris(grey, harrisResult,9,7,0.01);
//	    cvShowImage( "Harris", harrisResult );



            double quality = 0.01;
            double min_distance = 5;

            count = MAX_COUNT;
            cvGoodFeaturesToTrack( grey, eig, temp, points[1], &count,
                                   quality, min_distance, 0, 3, 0, 0.04 );
	    cvShowImage("Harris",eig);
            cvFindCornerSubPix( grey, points[1], count,
                cvSize(win_size,win_size), cvSize(-1,-1),
                cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
            cvReleaseImage( &eig );
            cvReleaseImage( &temp );
	    cvReleaseImage( &harrisResult);

	    

            add_remove_pt = 0;
        }
        else if( count > 0 )
        {
            cvCalcOpticalFlowPyrLK( prev_grey, grey, prev_pyramid, pyramid,
                points[0], points[1], count, cvSize(win_size,win_size), 3, status, 0,
                cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), flags );
            flags |= CV_LKFLOW_PYR_A_READY;
            for( i = k = 0; i < count; i++ )
            {
                if( add_remove_pt )
                {
                    double dx = pt.x - points[1][i].x;
                    double dy = pt.y - points[1][i].y;

                    if( dx*dx + dy*dy <= 25 )
                    {
                        add_remove_pt = 0;
                        continue;
                    }
                }
                
                if( !status[i] )
                    continue;
                
                points[1][k++] = points[1][i];
                cvCircle( image, cvPointFrom32f(points[1][i]), 3, CV_RGB(0,200+20*i,0), -1, 8,0);
            }
            count = k;
        }

        if( add_remove_pt && count < MAX_COUNT )
        {
            points[1][count++] = cvPointTo32f(pt);
            cvFindCornerSubPix( grey, points[1] + count - 1, 1,
                cvSize(win_size,win_size), cvSize(-1,-1),
                cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
            add_remove_pt = 0;
        }

	if(count >=NUMPTS){
		float cubeSize = 100.0;
		float alturaNariz = -20.0;
		std::vector<CvPoint3D32f> modelPoints;

//modelPoints.push_back(cvPoint3D32f(0.0f, 0.0f, 0.0f));

/*modelPoints.push_back(cvPoint3D32f(0.0f, 0.0f, cubeSize));
modelPoints.push_back(cvPoint3D32f(cubeSize, 0.0f, cubeSize));
modelPoints.push_back(cvPoint3D32f(0.0f, cubeSize, cubeSize));
modelPoints.push_back(cvPoint3D32f(0.0f, cubeSize, 0.0f));

modelPoints.push_back(cvPoint3D32f(cubeSize, 0.0f, 0.0f));
modelPoints.push_back(cvPoint3D32f(cubeSize, cubeSize, 0.0f));
modelPoints.push_back(cvPoint3D32f(cubeSize, cubeSize, cubeSize));*/


		modelPoints.push_back(cvPoint3D32f(0.0f, 0.0f, 0.0f));
		modelPoints.push_back(cvPoint3D32f(cubeSize, 0.0f, 0.0f));
		modelPoints.push_back(cvPoint3D32f((3/4.0)*cubeSize, cubeSize, 0.0f));
		modelPoints.push_back(cvPoint3D32f((1/4.0)*cubeSize, cubeSize, 0.0f));

		modelPoints.push_back(cvPoint3D32f(40.0f, 40.0f, alturaNariz));
		modelPoints.push_back(cvPoint3D32f(60.0f, 40.0f, alturaNariz));
		modelPoints.push_back(cvPoint3D32f(40.0f, 60.0f, alturaNariz));
		modelPoints.push_back(cvPoint3D32f(60.0f, 60.0f, alturaNariz));
/*		modelPoints.push_back(cvPoint3D32f(0.0f, 0.0f, -50.0f));

		modelPoints.push_back(cvPoint3D32f(60.0f, 60.0f, alturaNariz));
		modelPoints.push_back(cvPoint3D32f(40.0f, 60.0f, alturaNariz));*/

		CvPOSITObject *positObject = cvCreatePOSITObject( &modelPoints[0], static_cast<int>(modelPoints.size()) );

		std::vector<CvPoint2D32f> imagePoints;

		for(i=0;i<NUMPTS;i++){
			CvPoint2D32f point2D;
		        //The central point is not add because POSIT needs the image point coordinates related to the middle point of the image
			point2D.x = cvPointFrom32f(points[1][i]).x-320;
			point2D.y = cvPointFrom32f(points[1][i]).y-240; 
			imagePoints.push_back( point2D );
			printf("%f %f\n",point2D.x,point2D.y);
		}
		printf("\n");

		//set posit termination criteria: 100 max iterations, convergence epsilon 1.0e-5
		CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_EPS, 300, 1.0e-4 );
		FILE* in = fopen("in.txt","r");
		int myFocus;
		fscanf(in,"%d",&myFocus);
		cvPOSIT( positObject, &imagePoints[0], myFocus, criteria, rotation_matrix, translation_vector ); 
		printf("Matrix data\n");
		for(i=0;i<9;i++){
			printf("%.2f ",rotation_matrix[i]);
		}
		for(i=0;i<3;i++){
			printf("%.2f ",translation_vector[i]);
		}
		printf("\n");

	}
	printf("Count %d\n",count);


        CV_SWAP( prev_grey, grey, swap_temp );
        CV_SWAP( prev_pyramid, pyramid, swap_temp );
        CV_SWAP( points[0], points[1], swap_points );
        need_to_init = 0;

//	CvMat* M = cvCreateMat(4,4,CV_32FC1);
	float theta = 0.0;//3.1415/3;
/*	float a[] = { 1,  0,  0,  			translation_vector[0],
	               0, cos(theta),  -sin(theta),  	translation_vector[1],
	               0, sin(theta),   cos(theta),  	translation_vector[2],
	               0,  0,  0,  0 };*/

	float a[] = {  rotation_matrix[0],  rotation_matrix[1],  rotation_matrix[2], translation_vector[0],
	               rotation_matrix[3],  rotation_matrix[4],  rotation_matrix[5], translation_vector[1],
	               rotation_matrix[6],  rotation_matrix[7],  rotation_matrix[8], translation_vector[2],
	               0,  0,  1,  0 };

	


	float pos[] = {0,0,0};

	CvMat Ma = cvMat(4, 4, CV_32FC1, a);

	float b[] =   {1, 0, 0,
			0, 1, 0,
			0, 0, 1 };
	CvMat Mb = cvMat(3, 3, CV_32FC1, b);
	
	int w;
	/*for(w=0;w<4;w++){
		points3d[w][0]= 0;
		points3d[w][1]= 20*w;
		points3d[w][2]= 0;
		
	}*/
	//points={ {0,0,0},{100,0,0},{100,-20,0},{0,-20,0}};



	CvMat Mpoints[NUMPTS];

	for(w=0;w<NUMPTS;w++){
//		printf("Values %f %f %f",points3d[0],points3d[1],points3d[2]);
//		printf("Values %f %f %f",points3d[w][0],points3d[w][1],points3d[w][2]);
//		printf(" pointer %d %d ",points3d,&points3d);
		Mpoints[w] = cvMat( 4, 1, CV_32FC1,&points3d[w]);
//		printf("Creating coord %f %f \n",cvmGet(&Mpoints[w],0,0),cvmGet(&Mpoints[w],1,0));
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
		cvCircle( image, cvPoint(320+1000*cvmGet(Mr1,0,0)/cvmGet(Mr1,2,0),240+1000*cvmGet(Mr1,1,0)/cvmGet(Mr1,2,0)), 3, CV_RGB(150+30*w,0,0), -1, 8,0);
		printf("new coords %f %f\n",320+1000*cvmGet(Mr1,0,0)/cvmGet(Mr1,2,0),240+1000*cvmGet(Mr1,1,0)/cvmGet(Mr1,2,0));
	}




//	cvReleaseMat(&M);




        cvShowImage( "6dofHead", image );

        c = cvWaitKey(10);
        if( (char)c == 27 )
            break;
        switch( (char) c )
        {
        case 'r':
            need_to_init = 1;
            break;
        case 'c':
            count = 0;
	    gCount = 0;
            break;
	case 'm':
            model ^= 1;
            break;
        case 'n':
            night_mode ^= 1;
            break;
        default:
            ;
        }
    }

    cvReleaseCapture( &capture );
    cvDestroyWindow("6dofHead");

    return 0;
}

