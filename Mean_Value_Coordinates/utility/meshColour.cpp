// Colouring the mesh

#include "meshColour.h"

#include <stdio.h>


// Function to colour the model mesh after slicing
void meshColour::colourModelMesh(YsShellExt &controlMesh, YsShellExt &modelMesh, const std::vector <std::unordered_map <YSHASHKEY, float>> weights) {
	// Forming the colour map
	getColours(modelMesh,controlMesh);

	int i = 0;
	for (auto modelVTHd = modelMesh.NullVertex(); true == modelMesh.MoveToNextVertex(modelVTHd);){
		auto Weights = weights[i];

		float r = 0.0f, g = 0.0f, b = 0.0f;
		float totalW = 0.0f;
		YsColor modelColour = oldModelColours.find(modelMesh.GetSearchKey(modelVTHd))->second;
		for (auto controlVtHd = controlMesh.NullVertex(); true == controlMesh.MoveToNextVertex(controlVtHd);){
			float vertexWeight = Weights.find(controlMesh.GetSearchKey(controlVtHd))->second;
			
			r += vertexWeight*modelColour.Rf();
			g += vertexWeight*modelColour.Gf();
			b += vertexWeight*modelColour.Bf();
			totalW += vertexWeight;
		}

		YsColor newColour(r / totalW, g / totalW, b / totalW);							//Calculate new colour for Model Mesh vertex
		std::pair<YSHASHKEY, YsColor> elem(modelMesh.GetSearchKey(modelVTHd),newColour);
		interpolatedColours.insert(elem);
		i++;
	}

	// Recolouring the polygon based on the interpolation
	recolourPolygons(modelMesh);
}

void meshColour::getColours(YsShellExt &modelMesh, YsShellExt &controlMesh_deformed) {
	// COLOURS
	// Forming the colour map for the vertices
	for (auto vtHd = controlMesh_deformed.NullVertex(); true == controlMesh_deformed.MoveToNextVertex(vtHd);) {
		YsColor colour(0.0, 0.0, 0.0);
		auto neighbourPolygons = controlMesh_deformed.FindPolygonFromVertex(vtHd);				// All polygons that contain the vertex
																								//auto neighbourPolygons = controlMesh_deformed.GetNe
		for (auto polygon : neighbourPolygons) {
			YsColor temp_colour = controlMesh_deformed.GetColor(polygon);
			//std::cout << "X: " << temp_normal.x() << "\tY: " << temp_normal.y() << "\tZ: " << temp_normal.z() << "\n";
			colour.SetFloatRGB(colour.Rf() + temp_colour.Rf(), colour.Gf() + temp_colour.Rf(), colour.Bf() + temp_colour.Rf());
			//normal += controlMesh_deformed.GetNormal(polygon);
		}
		colour.SetFloatRGB(colour.Rf() / (float)neighbourPolygons.size(), colour.Gf() / (float)neighbourPolygons.size(), colour.Bf() / (float)neighbourPolygons.size());
		//std::cout << "X: " << normal.x() << "\tY: " << normal.y() << "\tZ: " << normal.z() << "\n";
		std::pair<YSHASHKEY, YsColor> el(controlMesh_deformed.GetSearchKey(vtHd), colour);
		vertexColours.insert(el);
	}

	// Initial colours of vertices in the model Mesh
	for (auto vtHd = modelMesh.NullVertex(); true == modelMesh.MoveToNextVertex(vtHd);) {
		YsColor colour(1.0, 0.0, 0.0);													// Vertices not shared with the control mesh are red
		auto iter = vertexColours.find(controlMesh_deformed.GetSearchKey(vtHd));
		if (iter != vertexColours.end()) {
			std::pair<YSHASHKEY, YsColor> element(modelMesh.GetSearchKey(vtHd), iter->second);
			oldModelColours.insert(element);
		}
		else {
			std::pair<YSHASHKEY, YsColor> element(modelMesh.GetSearchKey(vtHd), YsColor(1.0, 0.0, 0.0));
			oldModelColours.insert(element);
		}

	}
}

// Function to recolour the polygons of the model mesh
void meshColour::recolourPolygons(YsShellExt &modelMesh) {
	for (auto modelplHd = modelMesh.NullPolygon(); true == modelMesh.MoveToNextPolygon(modelplHd);) {
		auto polygonVertices = modelMesh.GetPolygonVertex(modelplHd);
		float r = 0.0f, g = 0.0f, b = 0.0f;
		for (auto vertex : polygonVertices) {
			YsColor vertexColour = interpolatedColours.find(modelMesh.GetSearchKey(vertex))->second;
			r += vertexColour.Rf();
			g += vertexColour.Gf();
			b += vertexColour.Bf();
		}
		modelMesh.SetPolygonColor(modelplHd,YsColor(r/(float)polygonVertices.size(), g / (float)polygonVertices.size(), b / (float)polygonVertices.size()));
	}
}
