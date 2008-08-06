#include "ehci.h"
#include <stdio.h>

// This demonstration shows how to use the simple getHeadBounds function

int main(){
	
	int upperX,upperY,headWidth,headHeight;
	
	//this function creates a window to see debug information
	//about capturing image and face detection
	ehciInit();
	while(1){
		//this is the main ehciLoop
		//every application should call this function in a realtime loop
		//so that the frame could be grabbed and processed
		//the EHCI2DFACEDETECT parameter tells ehci that only 2d detection will be
		//made
		ehciLoop(EHCI2DFACEDETECT,0);
		getHeadBounds(&upperX,&upperY,&headWidth,&headHeight);
		printf("Head Coordinates %3d %3d head width %3d height %3d (Press Ctrl+C to finish)\n",
				upperX,upperY,headWidth,headHeight);
	}
}