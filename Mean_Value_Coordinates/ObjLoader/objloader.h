
#ifndef OBJLOADER_IS_INCLUDED
#define OBJLOADER_IS_INCLUDED

#include "polygonalmesh.h"
#include <stdio.h>
#include <stdlib.h>






int ParseString(int &nWord, int wordTop[], int wordLength[], char str[]);

void SafeStrCpy(char dst[],char src[],int nLetters,int nLimit);

char *YsFgets(char buf[],unsigned long long int maxSize,FILE *fp);

int GetVertexId(char vtxIdx[]);

bool LoadObjFile(class PolygonalMesh &mesh, const char fn[]);


#endif