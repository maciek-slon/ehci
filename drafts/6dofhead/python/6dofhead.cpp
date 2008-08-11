#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <ctype.h>
#include <vector>
//..
//opengl related includes
#include <GL/glut.h>    // Header File For The GLUT Library 
#include <GL/gl.h>	// Header File For The OpenGL32 Library
#include <GL/glu.h>	// Header File For The GLu32 Library

#define READFROMIMAGEFILE 0
#define MYFOCUS 602

/* white ambient light at half intensity (rgba) */
GLfloat LightAmbient[] = { 0.5f, 0.5f, 0.5f, 1.0f };

/* super bright, full intensity diffuse light. */
GLfloat LightDiffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };

/* position of light (x, y, z, (position of light)) */
GLfloat LightPosition[] = { 0.0f, 0.0f, -100.0f, 1.0f };

int headXNow,headYNow;

/* The number of our GLUT window */
int window; 
int light;

double glPositMatrix[16];
double projectionMatrix[16];

void updateGlPositMatrix(CvMatr32f rotation_matrix,CvVect32f translation_vector);
void setInitialRTMatrix();

IplImage *image = 0, *grey = 0, *prev_grey = 0, *pyramid = 0, *prev_pyramid = 0, *swap_temp;
CvMatr32f rotation_matrix = new float[9];
CvVect32f translation_vector = new float[3];
float vertices[4719];
int model=0;

int refX,refY,refZ;
int lastHeadW,lastHeadH;

int initialGuess=1;

float theta = 0;//3.1415;
/*	float myRot[] = { 1,  0,  0,  			translation_vector[0],
	               0, cos(theta),  -sin(theta),  	translation_vector[1],
	               0, sin(theta),   cos(theta),  	translation_vector[2],
	               0,  0,  0,  0 };*/


int gCount = 0;

float myRoty[] = { cos(theta),   0,  sin(theta),   0,
		0, 	1,  	0,  	  0,
		-sin(theta),   0,   cos(theta),  0,
		0,  0,  0,  1 };

float gama = 0;//3.1415;
float dScale = 10.0;
float myRotz[] = { cos(gama),  -sin(gama),    0,  1.874*dScale,
		sin(gama),   cos(gama),    0,  -1.999*dScale,
		0, 	0,  	1,  	    -2.643*dScale,
		0,  0,  0,  1 };




#define NUMPTS 8
float scale = 1.0;
float points3d[NUMPTS][4]={ {0,0,0,1},{100*scale,0,0,1},{75*scale,-100*scale,0,1},{25*scale,-100*scale,0,1},
		{40*scale,-40*scale,+100*scale,1},{60*scale,-40*scale,+100*scale,1},
		{60*scale,-60*scale,+100*scale,1},{50*scale,-50*scale,+140*scale,1}//{40*scale,60*scale,+100*scale,1}
};//, {0,0,-50}};//,{60,60,10,1},{40,60,10,1}};*/

struct triangle{
	float vert[3][3];
};

triangle triangles[3036];

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
CvCapture* capture = 0;


CvPoint upperHeadCorner = cvPoint(0,0);
int headWidth, headHeight;



void cvLoop();

//terminar funcao para inserir novos pontos
/*
CvPoint newFeaturePoints[1000];
int newFeaturePointsCount=0;

void insertFeature(int x, int y){
	newFeaturePoints
}*/


void loadRaw(char* file){

	float p1[3],p2[3],p3[3];
	FILE* in = fopen("head.raw","r");
	//if(in!=NULL) printf("Good\n");
	char temp[200];
	//fscanf(in,"%s",&temp);
	//fscanf(in,"%f",&p1[0]);
	//	printf("Temp %s %f\n",temp,p1[0]);
	//	return;
	int tcount=0;
	float mScale= 1.0;
	//float deltaX=1.874,deltaY=-1.999,deltaZ=-2.643;
	float deltaX=0.0,deltaY=0.0,deltaZ=0.0;

	while( fscanf(in,"%f%f%f%f%f%f%f%f%f",&p1[0],&p1[1],&p1[2],&p2[0],&p2[1],&p2[2],&p3[0],&p3[1],&p3[2])!=EOF){
		triangles[tcount].vert[0][0]=(p1[0]+deltaX)*mScale;
		triangles[tcount].vert[0][1]=(p1[1]+deltaY)*mScale;
		triangles[tcount].vert[0][2]=(p1[2]+deltaZ)*mScale;

		triangles[tcount].vert[1][0]=(p2[0]+deltaX)*mScale;
		triangles[tcount].vert[1][1]=(p2[1]+deltaY)*mScale;
		triangles[tcount].vert[1][2]=(p2[2]+deltaZ)*mScale;

		triangles[tcount].vert[2][0]=(p3[0]+deltaX)*mScale;
		triangles[tcount].vert[2][1]=(p3[1]+deltaY)*mScale;
		triangles[tcount].vert[2][2]=(p3[2]+deltaZ)*mScale;

		tcount++;
		//printf("Tcount %d\n",tcount);
	}
	fclose(in);


}

void loadVertices(){
	int pos = 0;
	FILE* in = fopen("vertices.txt","r");
	while(fscanf(in,"%f",&vertices[pos++])!=EOF);
	fclose(in);
}


//TODO: correct projection matrix
void setProjectionMatrix(){
	double farPlane=10000.0;
	double nearPlane=1.0;
	double width = 640;
	double height = 480;
	double focalLength = MYFOCUS;
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

//uses ViolaJones to find head position
//returns upperHeadCorner, headWidth, and headHeight
void getObjectPosition(IplImage* frame, CvPoint* upperHeadCorner,int* headWidth,int* headHeight );

static CvHaarClassifierCascade* cascade = 0;
int cascadeLoaded=0;
static CvMemStorage* storage = 0;

void detect_and_draw( IplImage* img,CvPoint* upperHeadCorner,int* headWidth,int* headHeight  )
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
			headXNow=r->x*scale;
			headYNow=r->y*scale;
			*headWidth=r->width*scale;
			*headHeight=r->height*scale;					  	 
		}
	}

	//    cvShowImage( "result", img );
	cvReleaseImage( &gray );
	cvReleaseImage( &small_img );
}

//loads cascade xml
void loadCascade(){	

	cascade = (CvHaarClassifierCascade*)cvLoad( "haarcascade_frontalface_default.xml", 0, 0, 0 );
	if( !cascade ){
		printf( "ERROR: Could not load classifier cascade\n" );
	}
	storage = cvCreateMemStorage(0);
	cascadeLoaded=1;

}
void getObjectPosition(IplImage* frame, CvPoint* upperHeadCorner,int* headWidth,int* headHeight ){
	if(!cascadeLoaded){
		printf("Cascade not loaded!\n");
		return;
	}
	IplImage *frame_copy = 0;
	frame_copy = cvCreateImage( cvSize(frame->width,frame->height), IPL_DEPTH_8U, frame->nChannels );
	cvCopy( frame, frame_copy, 0 );
	detect_and_draw(frame_copy,upperHeadCorner,headWidth,headHeight);



	cvRectangle(frame, *upperHeadCorner, cvPoint(upperHeadCorner->x + *headWidth,upperHeadCorner->y + *headHeight), cvScalar(0,0,255), 1);



	cvReleaseImage( &frame_copy );

}

void do2dDrawing(){

	/*int XSize = 640, YSize = 480;
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	glOrtho (0, XSize, YSize, 0, 0, 1);
	glMatrixMode (GL_MODELVIEW);
	glBegin(GL_POINTS);
	glColor3f(1.0f, 1.0f, 1.0f);
	for(int i=0;i<count;i++)
		glVertex2f(cvPointFrom32f(points[0][i]).x - upperHeadCorner.x  , cvPointFrom32f(points[0][i]).y - upperHeadCorner.y);
	glEnd();*/




}



/* The main drawing function. */
void DrawGLScene(void)
{  
	//cvLoop();
	//	printf("Drawing\n");

	//  	glClearColor(1.0f,1.0f,1.0f,1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		// Clear The Screen And The Depth Buffer
	glLoadIdentity();				// Reset The View

	if(initialGuess){

		setInitialRTMatrix();
		updateGlPositMatrix(rotation_matrix,translation_vector);	
	}



	printf("Trans %lf %lf %lf\n",glPositMatrix[12],glPositMatrix[13],glPositMatrix[14]);


	//needs to correct the projection so that it works according to camera internal parameters

	//    gluPerspective(40.0f,(GLfloat)320/(GLfloat)240.0,0.1f,10000.0f);	// Calculate The Aspect Ratio Of The Window
	//    glOrtho(-1000,+1000,-1000,+1000,0.1f,10000.0f);

	//look in +z direction
	if(initialGuess){
		//		initialGuess=1;//turn to false

		glMatrixMode( GL_MODELVIEW );
		glLoadIdentity();
		//		glLoadMatrixd( glPositMatrix );

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();				// Reset The Projection Matrix

		float mScale = 10000;

		glOrtho(-10*mScale/headWidth*0.05,+10*mScale/headWidth,-9*mScale/headHeight,+10*mScale/headHeight*0.1,-1000.0f,100.0f);






		glBegin(GL_TRIANGLES);

		for(int i=0;i<3036;i++){
			//glColor3d ((GLdouble)i/3036.0, 0.0, 0.0);
			glColor3d (0.0, 0.0, 1.0);

			for(int j=0;j<3;j++){
				//printf(" Vert %f %f %f\n",triangles[i].vert[j][0],triangles[i].vert[j][1],triangles[i].vert[j][2]-5);

				glVertex3f( triangles[i].vert[j][0],triangles[i].vert[j][1],triangles[i].vert[j][2]);
			}
		}

		glEnd();

		glBegin(GL_LINES);

		glColor3f(1.0f, 1.0f, 1.0f);
		glVertex3f(0, 0,1.0f);
		glVertex3f(100, 100,1.0f);
		glEnd();

	}
	else{
		
		

		glMatrixMode( GL_MODELVIEW );
		glLoadMatrixd( glPositMatrix );

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();				// Reset The Projection Matrix
		glLoadMatrixd( projectionMatrix );


		gluLookAt(0, 0, 0 , 0, 0, +1.0, 0, -1, 0);
		
		


		glBegin(GL_LINES);
		glColor3f(1.0f,0.0f,0.0);
		glVertex3f( 0.0f,0.0f,0.0f);
		glVertex3f( 1000.0f,0.0f,0.0f);	
		glEnd();


		glBegin(GL_LINES);
		glColor3f(0.0f,1.0f,0.0);
		glVertex3f(0.0f, 0.0f,0.0f);
		glVertex3f(0.0f, 1000.0f,0.0f);	
		glEnd();
		
		glBegin(GL_LINES);
				glColor3f(0.0f,1.0f,0.0);
				glVertex3f(100.0f, 0.0f,0.0f);
				glVertex3f(100.0f, 1000.0f,0.0f);	
		glEnd();

		glBegin(GL_LINES);
		glColor3f(0.0f,0.0f,1.0);
		glVertex3f( 0.0f,0.0f,0.0f);
		glVertex3f( 0.0f,0.0f,1000.0f);	
		glEnd();

		
		
		float cScale = 100;
		
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


		
		
		/*float deltaX = (0.1*lastHeadW - refX)/(1.0*lastHeadW)*cScale;
		float deltaY = -(0.1*lastHeadH - refY)/(1.0*lastHeadH)*cScale;//openGl axis is pointing down now
		float deltaZ = 0*cScale;*/

		//glTranslatef(0,100,0);
		//glTranslatef( -deltaX,deltaY,-deltaZ);

		for(int i=0;i<lastHeadW;i+=8){

			float fx = (1.6667 * i/(1.0*lastHeadW)) - 0.332;			
			float fy = 0;//(1.6667 * py/(1.0*lastHeadH)) - 0.332;
			float fz = sin(fx*3.141593);//cos((i-0.5*lastHeadW)/lastHeadW* 1.2 *3.141593);
			deltaX = -((1.6667 * refX/(1.0*lastHeadW)) - 0.332)*cScale;
			deltaY = -((1.6667 * refY/(1.0*lastHeadH)) - 0.332)*cScale;
			deltaZ = -sin(deltaX*3.141593)*cScale;
			
			glBegin(GL_LINES);
			glColor3f(0.0f,1.0f,0.0);
			glVertex3f(fx*cScale+deltaX, +0.332*cScale   -deltaY, fz*cScale+deltaZ);
			glVertex3f(fx*cScale+deltaX, -1.3347*cScale  -deltaY, fz*cScale+deltaZ);	
			glEnd();


		}
					

		//This will translate the first Posit point to the top right part of the head (its origin)
		//150 is the width of the head (points -2.5 up to 2.5 times 30, which is the modelscale)
		//180 is the height of the head (points -5.0 up to 1.0 times 30)		
		//glTranslatef( -150.0*(refX-10)/100.0,+180.0*(refY+10)/100.0f, 0.0f);




		//					refX = (int)(px/(1.0*headWidth)*100);

		//glTranslatef( (refX - 0.1*lastHeadW*100)*.05, (refY - 0.1*lastHeadH*100)*.05, 0.0f);
		glBegin(GL_TRIANGLES);





		//glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
		//  glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
		// glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad

		for(int i=0;i<3036;i++){
			//glColor3f (i/3036.0, 0.0, 0.0);
			glColor3d (0.0, 0.0, 1.0);


			float v1x = triangles[i].vert[1][0] - triangles[i].vert[0][0];
			float v1y = triangles[i].vert[1][1] - triangles[i].vert[0][1];
			float v1z = triangles[i].vert[1][2] - triangles[i].vert[0][2];

			float v2x = triangles[i].vert[2][0] - triangles[i].vert[0][0];
			float v2y = triangles[i].vert[2][1] - triangles[i].vert[0][1];
			float v2z = triangles[i].vert[2][2] - triangles[i].vert[0][2];

			float nx = v1y*v2z-v2y*v1z;
			float ny = v1z*v2x-v2z*v1x;
			float nz = v1x*v2y-v2x*v1y;
			glEnable(GL_NORMALIZE);
			glNormal3f( nx,ny,nz);
			glDisable(GL_NORMALIZE);


			for(int j=0;j<3;j++){
				//printf(" Vert %f %f %f\n",triangles[i].vert[j][0],triangles[i].vert[j][1],triangles[i].vert[j][2]-5);
				glVertex3f( triangles[i].vert[j][0],triangles[i].vert[j][1],triangles[i].vert[j][2]);
			}
		}

		glEnd();
	}



	do2dDrawing();
	// since this is double buffered, swap the buffers to display what just got drawn.
	glutSwapBuffers();
	cvLoop();
}

/* The function called when our window is resized (which shouldn't happen, because we're fullscreen) */
void ReSizeGLScene(GLsizei Width, GLsizei Height)
{
	if (Height==0)				// Prevent A Divide By Zero If The Window Is Too Small
		Height=1;

	glViewport(0, 0, Width, Height);		// Reset The Current Viewport And Perspective Transformation

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);
	glMatrixMode(GL_MODELVIEW);
}

/* The function called whenever a normal key is pressed. */
void keyPressed(unsigned char key, int x, int y) 
{

	switch (key) {    
	case 27: // kill everything.
		/* shut down our window */
		glutDestroyWindow(window); 

		/* exit the program...normal termination. */
		exit(1);                   	
		break; // redundant.
	case 'm':
		model ^= 1;
		break;
	case 'w':
		count=0;
		gCount=29;
		break;
	case 'i':
		initialGuess = !initialGuess;
		printf("Initial guess now is %d\n",initialGuess);
		break;
	case 76: 
	case 108: // switch the lighting.
		printf("L/l pressed; light is: %d\n", light);
		light = light ? 0 : 1;              // switch the current value of light, between 0 and 1.
		printf("Light is now: %d\n", light);
		if (!light) {
			glDisable(GL_LIGHTING);
		} else {
			glEnable(GL_LIGHTING);
		}
		break;


	default:
		break;
	}	
}

/* A general OpenGL initialization function.  Sets all of the initial parameters. */
void InitGL(GLsizei Width, GLsizei Height)	// We call this right after our OpenGL window is created.
{
	glEnable(GL_TEXTURE_2D);                    // Enable texture mapping.


	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);	// This Will Clear The Background Color To Black
	glClearDepth(1.0);				// Enables Clearing Of The Depth Buffer
	glDepthFunc(GL_LESS);			// The Type Of Depth Test To Do
	glEnable(GL_DEPTH_TEST);			// Enables Depth Testing
	glShadeModel(GL_SMOOTH);			// Enables Smooth Color Shading

	//float specReflection[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	//glMaterialfv(GL_FRONT, GL_SPECULAR, specReflection);
	//glMateriali(GL_FRONT, GL_SHININESS, 96);





	glEnable ( GL_COLOR_MATERIAL ) ;

	//float colorBlue[] = { 0.0f, 0.0f, 1.0f, 1.0f };
	//glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, colorBlue);
	//glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);





	glPolygonMode(GL_FRONT, GL_LINE);
	glPolygonMode(GL_BACK, GL_LINE);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();				// Reset The Projection Matrix

	gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);	// Calculate The Aspect Ratio Of The Window

	glMatrixMode(GL_MODELVIEW);

	// set up light number 1.
	//    glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);  // add lighting. (ambient)
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);  // add lighting. (diffuse).
	glLightfv(GL_LIGHT1, GL_POSITION,LightPosition); // set light position.
	glEnable(GL_LIGHT1);                             // turn light 1 on.
	//    glEnable(GL_LIGHTING);
}

void openGLCustomInit(int argc, char** argv ){

	printf("Before glutinit %d\n",argc);//,argv[0]);
	glutInit(&argc, argv);  
	printf("after\n");

	/* Select type of Display mode:   
     Double buffer 
     RGBA color
     Alpha components supported 
     Depth buffer */  
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);  

	/* get a 640 x 480 window */
	glutInitWindowSize(640, 480);  

	/* the window starts at the upper left corner of the screen */
	glutInitWindowPosition(500, 0);  

	/* Open a window */  
	window = glutCreateWindow("6DOF Head Tracking OpenGL - EHCI - Daniel Lelis Baggio");

	/* Register the function to do all our OpenGL drawing. */
	glutDisplayFunc(&DrawGLScene);  

	/* Go fullscreen.  This is as soon as possible. */
	//    glutFullScreen();
	//   glutReshapeWindow(640,480);

	/* Even if there are no events, redraw our gl scene. */
	glutIdleFunc(&DrawGLScene);

	/* Register the function called when our window is resized. */
	glutReshapeFunc(&ReSizeGLScene);

	/* Register the function called when the keyboard is pressed. */
	glutKeyboardFunc(&keyPressed);

	/* Register the function called when special keys (arrows, page down, etc) are pressed. */
	//    glutSpecialFunc(&specialKeyPressed);

	/* Initialize our window. */
	InitGL(640, 480);


	/* Start Event Processing Engine */  
	glutMainLoop();  

}

void plot2dModel(){

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
	glPositMatrix[14] =  translation_vector[2]; //negative
	glPositMatrix[15] = 1.0; //homogeneous

}

void setInitialRTMatrix(){


	rotation_matrix[0]=1.0; rotation_matrix[1]=  0; rotation_matrix[2]=  0;
	rotation_matrix[3]=  0; rotation_matrix[4]=1.0; rotation_matrix[5]=  0;	
	rotation_matrix[6]=  0; rotation_matrix[7]=  0; rotation_matrix[8]=1.0;

	translation_vector[0]=   -90;
	translation_vector[1]=    80;
	translation_vector[2]=+300.0;

}

std::vector<CvPoint3D32f> modelPoints;
void getPositMatrix(IplImage* myImage){

	float cubeSize = 100.0;
	float alturaNariz = 20.0;

	int i;



	std::vector<CvPoint2D32f> imagePoints;

	if(initialGuess) modelPoints.clear();

	//setInitialRTMatrix();

	for(int i=0;i<count;i++){
		float myPixel[4];

		int px = cvPointFrom32f(points[1][i]).x - upperHeadCorner.x ;
		int py = cvPointFrom32f(points[1][i]).y - upperHeadCorner.y ;
		int vertIndex = cvRound(3036.0*myPixel[0]);
		glReadPixels(px,480-py,1,1,GL_RGBA,GL_FLOAT,&myPixel);
		if(initialGuess){
			printf("Pixel color %d %d %d %f %f %f %f\n",px,-py,(int)(40*sin(px*3.141593/headWidth)), myPixel[0],myPixel[1],myPixel[2],myPixel[3]);
			printf("Vertex %f %d Coord %f %f %f headw %d\n",3036.0*myPixel[0], cvRound(3036.0*myPixel[0]),

					(triangles[vertIndex].vert[0][0]/30.0)-1.874,
					(triangles[vertIndex].vert[0][1]/30.0)+1.999,
					(triangles[vertIndex].vert[0][2]/30.0)+2.643,headWidth
			);
			cvCircle( myImage, cvPoint(20*(4+(triangles[vertIndex].vert[0][0]/30.0)-1.874),
					20*(4+(triangles[vertIndex].vert[0][1]/30.0)+1.999)), 3, CV_RGB(0,0,255), -1, 8,0);
			cvRectangle( myImage, cvPoint(px,py),cvPoint(px+3,py+3),CV_RGB(255,0,0),1);


		}
		//if(vertIndex>0){
		/*				modelPoints.push_back( 	cvPoint3D32f( triangles[vertIndex].vert[0][0],
									triangles[vertIndex].vert[0][1],

									triangles[vertIndex].vert[0][2]));*/
		if(initialGuess){
			if(i==0){
				
				refX = cvPointFrom32f(points[1][i]).x-upperHeadCorner.x;//(int)(px/(1.0*headWidth)*100);
				refY = cvPointFrom32f(points[1][i]).y-upperHeadCorner.y;//-(int)(py/(1.0*headHeight)*100);
				refZ = 0;//(int)(40*sin(px*3.141593/headWidth));
				lastHeadW = headWidth;
				lastHeadH = headHeight;
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
			point2D.x = cvPointFrom32f(points[1][i]).x-320;
			point2D.y = cvPointFrom32f(points[1][i]).y-240;				
			imagePoints.push_back( point2D );
			printf("Ip %f %f\n", point2D.x, point2D.y);

		}
		else if(count==modelPoints.size()){
			CvPoint2D32f point2D;
			point2D.x = cvPointFrom32f(points[1][i]).x-320;
			point2D.y = cvPointFrom32f(points[1][i]).y-240;
			imagePoints.push_back( point2D );
			printf("Ip %f %f\n", point2D.x, point2D.y);

		}



		//}

	}

	/*		modelPoints.push_back(cvPoint3D32f(0.0f, 0.0f, 0.0f));
		modelPoints.push_back(cvPoint3D32f(cubeSize, 0.0f, 0.0f));
		modelPoints.push_back(cvPoint3D32f((3/4.0)*cubeSize, -cubeSize, 0.0f));
		modelPoints.push_back(cvPoint3D32f((1/4.0)*cubeSize, -cubeSize, 0.0f));

		modelPoints.push_back(cvPoint3D32f(40.0f, -40.0f, alturaNariz));
		modelPoints.push_back(cvPoint3D32f(60.0f, -40.0f, alturaNariz));
		modelPoints.push_back(cvPoint3D32f(40.0f, -60.0f, alturaNariz));
		modelPoints.push_back(cvPoint3D32f(60.0f, -60.0f, alturaNariz));*/

	if(modelPoints.size()==count){
		printf("Creating posit with %d points\n",modelPoints.size());
		CvPOSITObject *positObject = cvCreatePOSITObject( &modelPoints[0], static_cast<int>(modelPoints.size()) );



		/*		for(i=0;i<NUMPTS;i++){
				CvPoint2D32f point2D;
				   //The central point is not add because POSIT needs the image point coordinates related to the middle point of the image
				point2D.x = cvPointFrom32f(points[1][i]).x-320;
				point2D.y = cvPointFrom32f(points[1][i]).y-240; 
				imagePoints.push_back( point2D );
				printf("%f %f\n",point2D.x,point2D.y);
			}
			printf("\n");*/

		//set posit termination criteria: 1000 max iterations, convergence epsilon 1.0e-5
		CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 1000, 1.0e-5 );
		
		//FILE* in = fopen("in.txt","r");
		int myFocus= MYFOCUS;
		//fscanf(in,"%d",&myFocus);
		//fclose(in);
		cvPOSIT( positObject, &imagePoints[0], myFocus, criteria, rotation_matrix, translation_vector ); 
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

void insertDefaultPoints(IplImage* grey,int headX,int headY){

	count=0;

	points[0][count++] = cvPointTo32f( cvPoint(headX     , headY) );
	points[0][count++] = cvPointTo32f( cvPoint(headX+100 , headY) );
	points[0][count++] = cvPointTo32f( cvPoint(headX+ 75 , headY+100) );
	points[0][count++] = cvPointTo32f( cvPoint(headX+ 25 , headY+100) );
	points[0][count++] = cvPointTo32f( cvPoint(headX+ 40 , headY +40) );
	points[0][count++] = cvPointTo32f( cvPoint(headX+ 60 , headY +40) );
	points[0][count++] = cvPointTo32f( cvPoint(headX+ 60 , headY +60) );
	points[0][count++] = cvPointTo32f( cvPoint(headX+ 40 , headY +60) );
	cvFindCornerSubPix( grey, points[0] , 8,
			cvSize(win_size,win_size), cvSize(-1,-1),
			cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
}

void insertNewPoints(IplImage* grey, int headX,int headY,int width, int height){


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
	double min_distance = 10;

	count = 10;//MAX_COUNT;
	cvGoodFeaturesToTrack( result, eig, temp, points[0], &count,
			quality, min_distance, 0, 3, 0, 0.04 );	    	
	//TODO: ENABLE SUBPIX AFTER TESTS  
	/*cvFindCornerSubPix( result, points[0], count,
            cvSize(win_size,win_size), cvSize(-1,-1),
            cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));*/
	cvReleaseImage( &eig );
	cvReleaseImage( &temp );



	for(int i=0;i<count;i++){
		CvPoint pt = cvPointFrom32f(points[0][i]);
		points[0][i] = cvPoint2D32f(pt.x+headX,pt.y+headY);
	}


}

void cvLoop(){

	IplImage* frame = 0;
	int i, k, c;
	gCount++;


	if(READFROMIMAGEFILE){
		frame = cvLoadImage( "head.jpg", 1 );
	}
	else{
		frame = cvQueryFrame( capture );
		if( !frame )
			return;
	}

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





	getObjectPosition(image, &upperHeadCorner,&headWidth,&headHeight );

	/*
	if(gCount>=30 && gCount <=30+NUMPTS-1){
		int source = upperHeadCorner.x+50,sourcey=upperHeadCorner.y+50;

		if(gCount==30) on_mouse(CV_EVENT_LBUTTONDOWN,source    ,sourcey,0,NULL);
		if(gCount==31) on_mouse(CV_EVENT_LBUTTONDOWN,source+100,sourcey,0,NULL);
		if(gCount==32) on_mouse(CV_EVENT_LBUTTONDOWN,source +75,sourcey+100,0,NULL);
		if(gCount==33) on_mouse(CV_EVENT_LBUTTONDOWN,source +25,sourcey+100,0,NULL);
		if(gCount==34) on_mouse(CV_EVENT_LBUTTONDOWN,source +40,sourcey +40,0,NULL);
		if(gCount==35) on_mouse(CV_EVENT_LBUTTONDOWN,source +60,sourcey +40,0,NULL);
		if(gCount==36) on_mouse(CV_EVENT_LBUTTONDOWN,source +60,sourcey +60,0,NULL);
		if(gCount==37) on_mouse(CV_EVENT_LBUTTONDOWN,source +40,sourcey +60,0,NULL);

	}*/

	if(gCount==30){
		//		insertDefaultPoints(grey,upperHeadCorner.x+50,upperHeadCorner.y+50);		
		printf("Head x %d head y %d width %d height %d\n",upperHeadCorner.x,upperHeadCorner.y,headWidth,headHeight);
		if((upperHeadCorner.x>=0)&&(upperHeadCorner.y>=0)&&
				(upperHeadCorner.x+headWidth< cvGetSize(grey).width) && (upperHeadCorner.y+headHeight< cvGetSize(grey).height))
			insertNewPoints(grey,upperHeadCorner.x+(int)(0.25*headWidth),upperHeadCorner.y+(int)(0.25*headHeight),
					(int)(headWidth*0.5),(int)(headHeight*0.5));		
	}




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


		double quality = 0.01;
		double min_distance = 5;

		count = MAX_COUNT;
		cvGoodFeaturesToTrack( grey, eig, temp, points[1], &count,
				quality, min_distance, 0, 3, 0, 0.04 );	    	  
		cvFindCornerSubPix( grey, points[1], count,
				cvSize(win_size,win_size), cvSize(-1,-1),
				cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
		cvReleaseImage( &eig );
		cvReleaseImage( &temp );



		add_remove_pt = 0;
	}
	else if( count > 0 )
	{
		cvCalcOpticalFlowPyrLK( prev_grey, grey, prev_pyramid, pyramid,
				points[0], points[1], count, cvSize(win_size,win_size), 3, status, 0,
				cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,200,0.003), flags );
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
		//getPositMatrix uses points[1] obtained from cvCalcOpticalFlowPyrLK
		getPositMatrix(image);
		updateGlPositMatrix(rotation_matrix,translation_vector);	
		plot2dModel();

	}
	printf("Count %d\n",count);




	CV_SWAP( prev_grey, grey, swap_temp );
	CV_SWAP( prev_pyramid, pyramid, swap_temp );
	CV_SWAP( points[0], points[1], swap_points );
	need_to_init = 0;




	cvShowImage( "6dofHead", image );


	c = cvWaitKey(10);
	if( (char)c == 27 )
		exit(0);
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

int faceM(int argc, char** argv )
{


	printf("Before loadVertices ok!\n");
	loadVertices();
	char rawFile[]="head.raw";
	printf("Before loadraw!\n");
	loadRaw(rawFile);

	printf("Loaded raw\n");

	loadCascade();
	setProjectionMatrix();
	capture = cvCaptureFromCAM( 0);
	printf("camera ok!\n");
/*
	if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
		capture = cvCaptureFromCAM( argc == 2 ? argv[1][0] - '0' : 0 );
	else if( argc == 2 )
		capture = cvCaptureFromAVI( argv[1] );

	if( !capture )
	{
		fprintf(stderr,"Could not initialize capturing...\n");
		return -1;
	}*/

	cvNamedWindow( "6dofHead", 1 );
/*	char **teste=(char**) malloc(10*1*sizeof(char));//[10][10];
	teste[0]="6dofHead";*/
	printf("Before param\n");
	openGLCustomInit(argc,argv);



	cvReleaseCapture( &capture );
	cvDestroyWindow("6dofHead");

	return 0;
}

void face(){
//	char** argv= (char**) malloc(10*sizeof(char));//[1][10];
//	char*
//	strcpy(argv[0],"hello");
	char *argv[] = {"MyApp", NULL};

	printf("Hello python\n");
	faceM(1,argv);

}

