// Colouring the mesh

#ifndef __MESH_COLOUR_IS_INCLUDED__
#define __MESH_COLOUR_IS_INCLUDED__


#include <vector>
#include <unordered_map>
#include <unordered_set>

#include <ysclass.h>
#include "ysshellext.h"

std::unordered_map<YSHASHKEY, YsColor> vertexColours;
std::unordered_map<YSHASHKEY, YsColor> oldModelColours;
std::unordered_map<YSHASHKEY, YsColor> interpolatedColours;


void colourModelMesh(const YsShellExt &controlMesh, YsShellExt &modelMesh, const std::vector <std::unordered_map <YSHASHKEY, float>> weights);
void getColours(YsShellExt &modelMesh, YsShellExt &controlMesh_deformed);
void recolourPolygons(YsShellExt &modelMesh);

#endif