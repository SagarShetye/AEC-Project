// Colouring the mesh

#ifndef __MESH_COLOUR_IS_INCLUDED__
#define __MESH_COLOUR_IS_INCLUDED__


#include <vector>
#include <unordered_map>
#include <unordered_set>

#include <ysclass.h>
#include "ysshellext.h"

namespace meshColour {
	static std::unordered_map<YSHASHKEY, YsColor> vertexColours;
	static std::unordered_map<YSHASHKEY, YsColor> oldModelColours;
	static std::unordered_map<YSHASHKEY, YsColor> interpolatedColours;


	void colourModelMesh(YsShellExt &controlMesh, YsShellExt &modelMesh, const std::vector <std::unordered_map <YSHASHKEY, float>> weights);	// Performs interpolation
	void getColours(YsShellExt &modelMesh, YsShellExt &controlMesh_deformed);		//Forms map of the colours (initializer)
	void recolourPolygons(YsShellExt &modelMesh);		// Colours all the polygons after the interpolation
};

#endif