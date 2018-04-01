// Mean Value Coordinate Visualizer

#ifndef __MEAN_VALUE_VISUALIZER_IS_INCLUDED__
#define __MEAN_VALUE_VISUALIZER_IS_INCLUDED__

#include<iostream>
#include<vector>
#include<math.h>
#include<string>

#include "polygonalmesh.h"
#include "ysclass.h"
#include "fssimplewindow.h"
#include "fslazywindow.h"


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

public:
	meanValueVis() {};
	void meanValueInterpolateDeformation();
	void Initialize(const char model_fn[], const char control_fn[]);
	void draw() const;
	void RemakeVertexArray();
	static void AddColor(std::vector <float> &col, float r, float g, float b, float a);
	static void AddVertex(std::vector <float> &vtx, float x, float y, float z);
	static void AddNormal(std::vector <float> &nom, float x, float y, float z);
	void move();
	void deformControlMesh();
	bool terminate();
	~meanValueVis() {};
};


#endif