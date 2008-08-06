#include "ehci.h"
#include <stdio.h>


int main(){
	int upperX,upperY,headWidth,headHeight;
	
	ehciInit();
	while(1){
		ehciLoop(EHCI2DFACEDETECT,0);
		getHeadBounds(&upperX,&upperY,&headWidth,&headHeight);
		printf("Head Coordinates %3d %3d head width %3d height %3d (Press Ctrl+C to finish)\n",
				upperX,upperY,headWidth,headHeight);
	}
}