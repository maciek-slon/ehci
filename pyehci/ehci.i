%module ehci
%{
#include "ehci.h"

%}



%inline %{

double *new_mat16() {
   return (double *) malloc(16*sizeof(double));
}
void free_mat16(double* x) {
   free(x);
}
void mat16_set(double* x, int i, double v) {
   x[i] = v;
}
double mat16_get(double* x, int i) {
   return x[i];
}

double (*new_mat44())[4] {
   return (double (*)[4]) malloc(16*sizeof(double));
}
void free_mat44(double (*x)[4]) {
   free(x);
}
void mat44_set(double x[4][4], int i, int j, double v) {
   x[i][j] = v;
}
double mat44_get(double x[4][4], int i, int j) {
   return x[i][j];
}



%}

extern void setGLProjectionMatrix(double projectionMatrix[16]);
extern int ehciLoop(int mode, int initialGuess);
extern void getHeadBounds(int *OUTPUT, int *OUTPUT, int *OUTPUT, int *OUTPUT);
extern void ehciInit();
extern void getGlPositMatrix(double myGlPositMatrix[16]);

%pythoncode %{
def getGlPositMatrix():
	mat = new_mat16()
	_ehci.getGlPositMatrix(mat)
	L=[]
	for i in range(16):
		L.append(mat16_get(mat,i))
	free_mat16(mat)
	return L
%}
