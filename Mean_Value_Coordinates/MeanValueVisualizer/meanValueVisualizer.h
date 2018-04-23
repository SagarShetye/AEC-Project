// Mean Value Coordinate Visualizer

#ifndef __MEAN_VALUE_VISUALIZER_IS_INCLUDED__
#define __MEAN_VALUE_VISUALIZER_IS_INCLUDED__

#include<iostream>
#include<vector>
#include<math.h>
#include<string>

#include "polygonalmesh.h"
#include "ysclass.h"
#include "fslazywindow.h"

#include <ysshellext.h>

#include "objloader.h"

// Class for mesh visualization
class meanValueVis {
private:
	PolygonalMesh modelMesh, controlMesh, controlMesh_deformed;
	float distTolerance, angleTolerance;
	YsMatrix4x4 Rc;
	double d;
	YsVec3 t;
	YsVec3 bbox[2];

	std::vector <float> vtx, nom, col,
		vtx_control, nom_control, col_control;

	// vector for storing picked vertices for deformation:
	std::vector <PolygonalMesh::VertexHandle> vertexVec;

	// function stores vextex Handle selected by user:
	void storeVertex(PolygonalMesh::VertexHandle vtHd);

	// vector for storing the mouse positions for deformation:
	std::vector < std::vector <int>> mouseVec;

	// function deforms model as per mouse input:
	void Deform(std::vector < std::vector <int>> positions);



public:
	meanValueVis();
	void meanValueInterpolateDeformation();
	bool Initialize(const char model_fn[], const char control_fn[]);
	void colourAllPolygons(PolygonalMesh mesh, YsColor colour);										// Adds colour to all the polygons in the mesh
	void draw() const;
	//void RemakeVertexArray();
	static void AddColor(std::vector <float> &col, float r, float g, float b, float a);
	static void AddVertex(std::vector <float> &vtx, float x, float y, float z);
	static void AddNormal(std::vector <float> &nom, float x, float y, float z);


	void RemakeVertexArray(void);
	YsMatrix4x4 GetProjection(void) const;
	YsMatrix4x4 GetModelView(void) const;
	PolygonalMesh::PolygonHandle PickedPlHd(int mx, int my) const;
	PolygonalMesh::VertexHandle PickedVtHd(int mx, int my, int pixRange) const;

	// function to initialize vertices for mesh deformation: 
	void DeformSelection();

	// function stores mouse positions:
	void mousePositions(void);

	// function clears all selected vertices:
	void Clear();


	void move();
	void deformControlMesh();
	bool terminate();
	~meanValueVis() {};
};


#endif
