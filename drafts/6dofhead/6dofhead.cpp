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

/* white ambient light at half intensity (rgba) */
GLfloat LightAmbient[] = { 0.5f, 0.5f, 0.5f, 1.0f };

/* super bright, full intensity diffuse light. */
GLfloat LightDiffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };

/* position of light (x, y, z, (position of light)) */
GLfloat LightPosition[] = { 0.0f, 2.0f, 0.0f, 1.0f };

/* The number of our GLUT window */
int window; 
int light;

double glPositMatrix[16];
double projectionMatrix[16];

IplImage *image = 0, *grey = 0, *prev_grey = 0, *pyramid = 0, *prev_pyramid = 0, *swap_temp;
CvMatr32f rotation_matrix = new float[9];
CvVect32f translation_vector = new float[3];
float vertices[4719];
int model=0;

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

void cvLoop();


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
	float mScale= 30.0;
	float deltaX=1.874,deltaY=-1.999,deltaZ=-2.643;
	
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

void setProjectionMatrix(){
	double farPlane=10000.0;
	double nearPlane=1.0;
	double width = 320;
	double height = 240;
	projectionMatrix[0] = 1000.0/width;
	projectionMatrix[1] = 0.0;
	projectionMatrix[2] = 0.0;
	projectionMatrix[3] = 0.0;

	projectionMatrix[4] = 0.0;
	projectionMatrix[5] = 1000.0/ height;
	projectionMatrix[6] = 0.0;
	projectionMatrix[7] = 0.0;
	
	projectionMatrix[8] = 0;
	projectionMatrix[9] = 0;	
	projectionMatrix[10] = - 1.0;
	projectionMatrix[11] = -1.0;

	projectionMatrix[12] = 0.0;
	projectionMatrix[13] = 0.0;
	projectionMatrix[14] = -1.0;		
	projectionMatrix[15] = 0.0;

}

//uses ViolaJones to find head position
//returns upperHeadCorner, headWidth, and headHeight
void getHeadPosition(IplImage* frame, CvPoint* upperHeadCorner,int* headWidth,int* headHeight );

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
        printf( "detection time = %gms\n", t/((double)cvGetTickFrequency()*1000.) );
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

//loads cascade xml
void loadCascade(){	

	cascade = (CvHaarClassifierCascade*)cvLoad( "haarcascade_frontalface_default.xml", 0, 0, 0 );
	if( !cascade ){
		printf( "ERROR: Could not load classifier cascade\n" );
	}
	storage = cvCreateMemStorage(0);
	cascadeLoaded=1;

}
void getHeadPosition(IplImage* frame, CvPoint* upperHeadCorner,int* headWidth,int* headHeight ){
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



/* The main drawing function. */
void DrawGLScene(void)
{  
    //cvLoop();
	printf("Drawing\n");
cvLoop();
  
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		// Clear The Screen And The Depth Buffer
    glLoadIdentity();				// Reset The View


printf("Trans %lf %lf %lf\n",glPositMatrix[12],glPositMatrix[13],glPositMatrix[14]);
glMatrixMode( GL_MODELVIEW );
glLoadMatrixd( glPositMatrix );

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();				// Reset The Projection Matrix
	glLoadMatrixd( projectionMatrix );
    
	//needs to correct the projection so that it works according to camera internal parameters

//    gluPerspective(40.0f,(GLfloat)320/(GLfloat)240.0,0.1f,10000.0f);	// Calculate The Aspect Ratio Of The Window
//    glOrtho(-1000,+1000,-1000,+1000,0.1f,10000.0f);



	//look in +z direction
    gluLookAt(0, 0, 0 , 0, 0, +1.0, 0, -1, 0); //+ 5*headDist

//    glTranslatef(0.0f,0.0f,0);                  // move z units out from the screen.



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
	glColor3f(0.0f,0.0f,1.0);
	glVertex3f( 0.0f,0.0f,0.0f);
	glVertex3f( 0.0f,0.0f,1000.0f);	
    glEnd();



    glBegin(GL_TRIANGLES);

//glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
  //  glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
   // glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad

    for(int i=0;i<3036;i++){
			glColor3f (i/3036.0, 0.0, 0.0);
		for(int j=0;j<3;j++){
			//printf(" Vert %f %f %f\n",triangles[i].vert[j][0],triangles[i].vert[j][1],triangles[i].vert[j][2]-5);

			glVertex3f( triangles[i].vert[j][0],triangles[i].vert[j][1],triangles[i].vert[j][2]);
		}
    }

    glEnd();


 /*   glBegin(GL_QUADS);		                // begin drawing a cube
    
    // Front Face (note that the texture's corners have to match the quad's corners)
    glNormal3f( 0.0f, 0.0f, 1.0f);                              // front face points out of the screen on z.
    glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad
     glVertex3f(-1.0f,  1.0f,  1.0f);	// Top Left Of The Texture and Quad
  
    // Back Face
    glNormal3f( 0.0f, 0.0f,-1.0f);                              // back face points into the screen on z.
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Bottom Left Of The Texture and Quad
	
    // Top Face
    glNormal3f( 0.0f, 1.0f, 0.0f);                              // top face points up on y.
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
    
    // Bottom Face       
    glNormal3f( 0.0f, -1.0f, 0.0f);                             // bottom face points down on y. 
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    
    // Right face
    glNormal3f( 1.0f, 0.0f, 0.0f);                              // right face points right on x.
    glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    
    // Left Face
    glNormal3f(-1.0f, 0.0f, 0.0f);                              // left face points left on x.
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
    
    glEnd();                                    // done with the polygon.

    glTranslatef(-3.0f, 0.0f,0);                  // move z units out from the screen.

    glBegin(GL_QUADS);		                // begin drawing a cube
    
    // Front Face (note that the texture's corners have to match the quad's corners)
    glNormal3f( 0.0f, 0.0f, 1.0f);                              // front face points out of the screen on z.
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Top Left Of The Texture and Quad
    
    // Back Face
    glNormal3f( 0.0f, 0.0f,-1.0f);                              // back face points into the screen on z.
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Bottom Left Of The Texture and Quad
	
    // Top Face
    glNormal3f( 0.0f, 1.0f, 0.0f);                              // top face points up on y.
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
    
    // Bottom Face       
    glNormal3f( 0.0f, -1.0f, 0.0f);                             // bottom face points down on y. 
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    
    // Right face
    glNormal3f( 1.0f, 0.0f, 0.0f);                              // right face points right on x.
    glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    
    // Left Face
    glNormal3f(-1.0f, 0.0f, 0.0f);                              // left face points left on x.
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
    
    glEnd();                                    // done with the polygon.

    // 

    glTranslatef(+6.0f, 0.0f,0);                  // move z units out from the screen.

    glBegin(GL_QUADS);		                // begin drawing a cube
    
    // Front Face (note that the texture's corners have to match the quad's corners)
    glNormal3f( 0.0f, 0.0f, 1.0f);                              // front face points out of the screen on z.
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Top Left Of The Texture and Quad
    
    // Back Face
    glNormal3f( 0.0f, 0.0f,-1.0f);                              // back face points into the screen on z.
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Bottom Left Of The Texture and Quad
	
    // Top Face
    glNormal3f( 0.0f, 1.0f, 0.0f);                              // top face points up on y.
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
    
    // Bottom Face       
    glNormal3f( 0.0f, -1.0f, 0.0f);                             // bottom face points down on y. 
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    
    // Right face
    glNormal3f( 1.0f, 0.0f, 0.0f);                              // right face points right on x.
    glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    
    // Left Face
    glNormal3f(-1.0f, 0.0f, 0.0f);                              // left face points left on x.
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
    
    glEnd();                                    // done with the polygon.

    glTranslatef(0.0f, 0.0f,3.0);                  // move z units out from the screen.

    glBegin(GL_QUADS);		                // begin drawing a cube
    
    // Front Face (note that the texture's corners have to match the quad's corners)
    glNormal3f( 0.0f, 0.0f, 1.0f);                              // front face points out of the screen on z.
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Top Left Of The Texture and Quad
    
    // Back Face
    glNormal3f( 0.0f, 0.0f,-1.0f);                              // back face points into the screen on z.
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Bottom Left Of The Texture and Quad
	
    // Top Face
    glNormal3f( 0.0f, 1.0f, 0.0f);                              // top face points up on y.
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
    
    // Bottom Face       
    glNormal3f( 0.0f, -1.0f, 0.0f);                             // bottom face points down on y. 
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    
    // Right face
    glNormal3f( 1.0f, 0.0f, 0.0f);                              // right face points right on x.
    glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    
    // Left Face
    glNormal3f(-1.0f, 0.0f, 0.0f);                              // left face points left on x.
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
    
    glEnd();                                    // done with the polygon.
*/


    // since this is double buffered, swap the buffers to display what just got drawn.
    glutSwapBuffers();
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

//glPolygonMode(GL_FRONT, GL_LINE);
//glPolygonMode(GL_BACK, GL_LINE);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();				// Reset The Projection Matrix
    
    gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);	// Calculate The Aspect Ratio Of The Window
    
    glMatrixMode(GL_MODELVIEW);

    // set up light number 1.
    glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);  // add lighting. (ambient)
    glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);  // add lighting. (diffuse).
    glLightfv(GL_LIGHT1, GL_POSITION,LightPosition); // set light position.
//    glEnable(GL_LIGHT1);                             // turn light 1 on.
//    glEnable(GL_LIGHTING);
}

void openGLCustomInit(int argc, char** argv ){
	
 	glutInit(&argc, argv);  

    /* Select type of Display mode:   
     Double buffer 
     RGBA color
     Alpha components supported 
     Depth buffer */  
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);  

    /* get a 640 x 480 window */
    glutInitWindowSize(320, 240);  

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

void cvLoop(){

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
            return;



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
		float alturaNariz = 20.0;
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
		modelPoints.push_back(cvPoint3D32f((3/4.0)*cubeSize, -cubeSize, 0.0f));
		modelPoints.push_back(cvPoint3D32f((1/4.0)*cubeSize, -cubeSize, 0.0f));

		modelPoints.push_back(cvPoint3D32f(40.0f, -40.0f, alturaNariz));
		modelPoints.push_back(cvPoint3D32f(60.0f, -40.0f, alturaNariz));
		modelPoints.push_back(cvPoint3D32f(40.0f, -60.0f, alturaNariz));
		modelPoints.push_back(cvPoint3D32f(60.0f, -60.0f, alturaNariz));
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
		//only one of the criteria is used, so, as EPS could get in configurations that 
		//it wouldnt finish, we have chosen to use the number of iterations
		CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_ITER, 1000, 1.0e-5 );
		FILE* in = fopen("in.txt","r");
		int myFocus;
		fscanf(in,"%d",&myFocus);
		fclose(in);
		printf("Right before posit count %d\n",count);
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
/*
for (int f=0; f<3; f++)
{
        for (int c=0; c<3; c++)
        {
			glPositMatrix[c*4+f] = rotation_matrix[f*3+c];      //transposed
			
                
        }
}*/
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
		cvCircle( image, cvPoint(320+1000*cvmGet(Mr1,0,0)/cvmGet(Mr1,2,0),240+1000*cvmGet(Mr1,1,0)/cvmGet(Mr1,2,0)), w==0?8:3,w%2==0?CV_RGB(128,0,0): (w==7? CV_RGB(0,0,200) : CV_RGB(200,0,0)), -1, 8,0);
		printf("new coords %f %f\n",320+1000*cvmGet(Mr1,0,0)/cvmGet(Mr1,2,0),240+1000*cvmGet(Mr1,1,0)/cvmGet(Mr1,2,0));
		if(w==0) printf("Z(0) = %f\n",cvmGet(Mr1,2,0));
		if(w==7) printf("Z(7) = %f\n",cvmGet(Mr1,2,0));
	}




//	cvReleaseMat(&M);
	CvPoint upperHeadCorner = cvPoint(0,0);
	int headWidth, headHeight;

	getHeadPosition(image, &upperHeadCorner,&headWidth,&headHeight );

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

int main( int argc, char** argv )
{

	

    loadVertices();
    char rawFile[]="head.raw";
    loadRaw(rawFile);

	loadCascade();
	setProjectionMatrix();
    if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
        capture = cvCaptureFromCAM( argc == 2 ? argv[1][0] - '0' : 0 );
    else if( argc == 2 )
        capture = cvCaptureFromAVI( argv[1] );

    if( !capture )
    {
        fprintf(stderr,"Could not initialize capturing...\n");
        return -1;
    }

    cvNamedWindow( "6dofHead", 0 );
    //cvNamedWindow( "Harris", 0 );
    cvSetMouseCallback( "6dofHead", on_mouse, 0 );

    openGLCustomInit(argc,argv);
  
    

    cvReleaseCapture( &capture );
    cvDestroyWindow("6dofHead");

    return 0;
}

