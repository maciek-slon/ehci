#include "ehci.h"
#include <stdio.h>

// This demonstration shows how to use getGlPositMatrix
// to get 3d position of the head

int main(){
		
	double glPositMatrix[16];
	int initialGuess=1;
	int frameCount=0;
	int loopCount = 0;
	
	//this function creates a window to see debug information
	//about capturing image and face detection
	ehciInit();
	
	while(loopCount++ < 100){
		//ehciLoop is the main function of the library
		//it captures a frame and processes it
		//the EHCI6DFACEDETECT parameter tells ehci to 
		//run the 6 degrees of freedom algorithm to detect
		//head translation and rotation
		//the initialGuess parameter tells ehci to
		//initialize, when it is set to zero 
		//and tells ehci to track when it is set to one
		
		ehciLoop(EHCI6DFACEDETECT,initialGuess);
		
		//returns translation and rotation matrix in openGl format
		getGlPositMatrix(glPositMatrix);
		
		//the dimensions from 12 up to 14 store translation data
		//depending on the calibration, measures will be roughly proportional
		//to milimeters. Notice that the center of the camera has coordinates (0,0,0)
		double headX = glPositMatrix[12];
		double headY = glPositMatrix[13];
		double headZ = glPositMatrix[14];
		
		printf("Head Coordinates X: %+7.3lf Y: %+7.3lf Z:%+7.3lf ",
				headX,headY,headZ);
		
		//after 30 frames we will stop the initialization algorithm
		//and start the tracking algorithm
		
		frameCount++;
		if(frameCount >= 30) initialGuess=0;
		else printf("[initializing]");
		
		printf(" (Press Ctrl+C to finish)\n");
	}
	ehciExit();
}