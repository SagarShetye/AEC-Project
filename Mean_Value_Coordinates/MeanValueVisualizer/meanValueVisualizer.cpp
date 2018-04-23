// Mean Value Visualizer Implementation file

#include "meanValueVisualizer.h"
# include <iostream>
using namespace std;

// Constructor
meanValueVis::meanValueVis() {
	d = 10.0;
	t = YsVec3::Origin();
}

// Function to interpolate the mean value coordinates for a triangular mesh
// It takes 2 parameters viz. mesh of the model and the control mesh
// It calculates the required deformation value at each vertex
void meanValueVis::meanValueInterpolateDeformation() {
	// Iterating over the vertices of the model mesh
	auto vtxHd = modelMesh.NullVertex();
	modelMesh.MoveToNextVertex(vtxHd);
	for (; true == modelMesh.MoveToNextVertex(vtxHd);) {
		YsVec3 vtx = modelMesh.GetVertexPosition(vtxHd);
		YsVec3 totalF(0.0, 0.0, 0.0);																		// Total property value over the entire control mesh
		float totalW = 0.0;																					// Total weight over the entire control mesh
		bool interpolate = true;
		/*for (auto vHd = controlMesh.NullVertex(); true == controlMesh.MoveToNextVertex(vHd);) {
			float d = (controlMesh.GetVertexPosition(vHd) - vtx).GetLength();
			if (d < distTolerance) {
				modelMesh.SetVertexPosition(vtxHd, controlMesh.GetVertexPosition(vHd));
				break;
			}
		}*/

		// Iterating over the triangles in the control mesh for the interpolation
		auto plgHd_deformed = controlMesh_deformed.NullPolygon();
		controlMesh_deformed.MoveToNextPolygon(plgHd_deformed);
		auto plgHd = controlMesh.NullPolygon();
		controlMesh.MoveToNextPolygon(plgHd);
		for (; true == controlMesh.MoveToNextPolygon(plgHd) && true == controlMesh_deformed.MoveToNextPolygon(plgHd_deformed);) {
			std::vector<PolygonalMesh::VertexHandle> vtxHandles = controlMesh.GetPolygonVertex(plgHd);		// Vertices in the undeformed control mesh polygon

			std::vector<PolygonalMesh::VertexHandle> vtxHandles_deformed = controlMesh_deformed.GetPolygonVertex(plgHd_deformed);		// Vertices in the deformed control mesh polygon
			// vertices of the deformed control mesh
			YsVec3 p0 = controlMesh_deformed.GetVertexPosition(vtxHandles_deformed[0]),
				p1 = controlMesh_deformed.GetVertexPosition(vtxHandles_deformed[1]),
				p2 = controlMesh_deformed.GetVertexPosition(vtxHandles_deformed[2]);

			//std::cout << "Deformed size: " << vtxHandles_deformed.size() << "\n";
			float d0 = (controlMesh.GetVertexPosition(vtxHandles[0]) - vtx).GetLength(),
				d1 = (controlMesh.GetVertexPosition(vtxHandles[1]) - vtx).GetLength(),
				d2 = (controlMesh.GetVertexPosition(vtxHandles[2]) - vtx).GetLength();						// Distance of vtx from the vertices of the polygon

																											// If the vtx is very close to any vertex of the control mesh, approximate the values to be equal at both the points
			if (d0 < distTolerance) {
				modelMesh.SetVertexPosition(vtxHd, controlMesh_deformed.GetVertexPosition(vtxHandles_deformed[0]));
				// TODO:Move to next vertex in the model mesh
				interpolate = false;
				break;
				//continue;
			}
			if (d1 < distTolerance) {
				modelMesh.SetVertexPosition(vtxHd, controlMesh_deformed.GetVertexPosition(vtxHandles_deformed[1]));
				// TODO:Move to next vertex in the model mesh
				interpolate = false;
				break;
				//continue;
			}
			if (d2 < distTolerance) {
				modelMesh.SetVertexPosition(vtxHd, controlMesh_deformed.GetVertexPosition(vtxHandles_deformed[2]));
				// TODO:Move to next vertex in the model mesh
				interpolate = false;
				break;
				//continue;
			}

			// Moving to the next triangle in the control mesh if vtx is too far from it
			float dimX = this->bbox[1].x() - this->bbox[0].x(), dimY = this->bbox[1].y() - this->bbox[0].y(),
				dimZ = this->bbox[1].z() - this->bbox[0].z();
			if (d0 > (1 - distTolerance)*dimX || d0 > (1 - distTolerance)*dimY || d0 > (1 - distTolerance)*dimZ)
				continue;
			if (d1 > (1 - distTolerance)*dimX || d2 > (1 - distTolerance)*dimY || d1 > (1 - distTolerance)*dimZ)
				continue;
			if (d2 > (1 - distTolerance)*dimX || d2 > (1 - distTolerance)*dimY || d2 > (1 - distTolerance)*dimZ)
				continue;

			//std::cout << "No distTolerance\n";
			// Unit vectors along direction from vtx to vertices of the polygon
			YsVec3 u0 = (controlMesh.GetVertexPosition(vtxHandles[0]) - vtx).UnitVector(controlMesh.GetVertexPosition(vtxHandles[0]) - vtx),
				u1 = (controlMesh.GetVertexPosition(vtxHandles[1]) - vtx).UnitVector(controlMesh.GetVertexPosition(vtxHandles[1]) - vtx),
				u2 = (controlMesh.GetVertexPosition(vtxHandles[2]) - vtx).UnitVector(controlMesh.GetVertexPosition(vtxHandles[2]) - vtx);

			// Calculating the length of edges projected on the sphere centered at vtx
			float L0 = (u1 - u2).GetLength(), L1 = (u2 - u0).GetLength(), L2 = (u0 - u1).GetLength();
			float theta_0 = 2 * asin(L0 / 2.0), theta_1 = 2 * asin(L1 / 2.0), theta_2 = 2 * asin(L2 / 2.0);
			float h = (theta_0 + theta_1 + theta_2) / 2.0;

			// vtx lies in the polygon, normal Barycentric coordinates used
			if (YsPi - h < angleTolerance) {
				float w0, w1, w2;											// Weights for Barycentric interpolation
				w0 = sin(theta_0)*L1*L2;
				w1 = sin(theta_1)*L0*L2;
				w2 = sin(theta_2)*L1*L0;

				//YsVec3 newPosition = (w0*controlMesh_deformed.GetVertexPosition(vtxHandles_deformed[0]) + w1*controlMesh_deformed.GetVertexPosition(vtxHandles_deformed[1]) +
				//	w2*controlMesh_deformed.GetVertexPosition(vtxHandles_deformed[2])) / (w0 + w1 + w2);
				YsVec3 newPosition = (w0*p0 + w1*p1 + w2*p2) / (w0 + w1 + w2);
				modelMesh.SetVertexPosition(vtxHd, newPosition);
				// TODO:Move to next vertex in the model mesh
				interpolate = false;
				break;
				//continue;
			}

			// vtx doesn't lie in the plane of the polygon, have to consider the spherical triangle
			float c0 = 2 * sin(h)*sin(h - theta_0) / (sin(theta_1)*sin(theta_2)) - 1,
				c1 = 2 * sin(h)*sin(h - theta_1) / (sin(theta_0)*sin(theta_2)) - 1,
				c2 = 2 * sin(h)*sin(h - theta_2) / (sin(theta_0)*sin(theta_1)) - 1;
			float det = u0.x()*(u1.y()*u2.z() - u2.y()*u1.z()) - u1.x()*(u0.y()*u2.z() - u2.y()*u0.z())
				+ u2.x()*(u0.y()*u1.z() - u0.z()*u1.y());
			float s0 = (det / abs(det))*sqrt(1 - c0*c0),
				s1 = (det / abs(det))*sqrt(1 - c1*c1),
				s2 = (det / abs(det))*sqrt(1 - c2*c2);

			// vtx lies in plane of the polygon but outide it
			if (abs(s0) <= angleTolerance || abs(s1) <= angleTolerance || abs(s2) <= angleTolerance)
				continue;

			// Calculating the weights
			float w0, w1, w2;
			//w0 = (theta_0 - c1*theta_2 - c2*theta_1) / (d0*sin(theta_1)*s2);
			//w1 = (theta_1 - c0*theta_2 - c2*theta_0) / (d1*sin(theta_2)*s0);
			//w2 = (theta_2 - c1*theta_0 - c0*theta_1) / (d2*sin(theta_0)*s1);

			w0 = (theta_0 - c1*theta_2 - c2*theta_1) / (2*d0*sin(theta_2)*sqrt(1 - c1*c1));
			w1 = (theta_1 - c0*theta_2 - c2*theta_0) / (2*d1*sin(theta_0)*sqrt(1 - c2*c2));
			w2 = (theta_2 - c1*theta_0 - c0*theta_1) / (2*d2*sin(theta_1)*sqrt(1 - c0*c0));


			//totalF += w0*controlMesh_deformed.GetVertexPosition(vtxHandles_deformed[0]) + w1*controlMesh_deformed.GetVertexPosition(vtxHandles_deformed[1])
			//	+ w2*controlMesh_deformed.GetVertexPosition(vtxHandles_deformed[2]);
			totalF += w0*p0 + w1*p1 + w2*p2;
			totalW += w0 + w1 + w2;

			//// Moving to the next polygon of the deformed control mesh
			//controlMesh_deformed.MoveToNextPolygon(plgHd_deformed);										
		}

		// Updating the vertex position of the model based on the interpolated value from the control mesh
		if (interpolate) {
			YsVec3 newPos = totalF / totalW;
			modelMesh.SetVertexPosition(vtxHd, newPos);
		}
		//std::cout << "newX: " << newPos.x() << "\tnewY: " << newPos.y() << "\tnewZ: " << newPos.z() << "\n";
	}
	std::cout << "DONE\n";
}


// Adding colour to all polygons of a mesh
void meanValueVis::colourAllPolygons(PolygonalMesh controlMesh_deformed, YsColor colour) {
	for (auto plgHd = controlMesh_deformed.NullPolygon(); controlMesh_deformed.MoveToNextPolygon(plgHd) == true;) {
		controlMesh_deformed.SetPolygonColor(plgHd,colour);
	}
}

// Initialize (read the mesh files)
bool meanValueVis::Initialize(const char model_fn[], const char control_fn[]) {
	
	// Reading the .obj file
	if (!(LoadObjFile(modelMesh, model_fn) && LoadObjFile(controlMesh, control_fn) &&
		LoadObjFile(controlMesh_deformed, control_fn)))
		return false;

	// Colouring the model mesh
	colourAllPolygons(modelMesh,YsBlue());

	// Setting the tolerances
	distTolerance = 0.01f;
	angleTolerance = 0.01f;
	// Bounding box of the model
	//modelMesh.GetBoundingBox(bbox[0],bbox[1]);
	//cout << "1" << endl;
	RemakeVertexArray();
	//cout << "re" << endl;
	return true;
}

// Setting up the arrays that would be used for drawing the mesh
void meanValueVis::RemakeVertexArray() {
	vtx.clear();
	col.clear();
	nom.clear();

	for (auto plHd = modelMesh.NullPolygon(); true == modelMesh.MoveToNextPolygon(plHd); )
	{
		auto plVtHd = modelMesh.GetPolygonVertex(plHd);
		auto plCol = modelMesh.GetColor(plHd);
		auto plNom = modelMesh.GetNormal(plHd);
		
		// Let's assume every polygon is a triangle for now.
		if (3 == plVtHd.size())
		{
			for (int i = 0; i<3; ++i)
			{
				auto vtPos = modelMesh.GetVertexPosition(plVtHd[i]);
				vtx.push_back(vtPos.xf());
				vtx.push_back(vtPos.yf());
				vtx.push_back(vtPos.zf());
				nom.push_back(plNom.xf());
				nom.push_back(plNom.yf());
				nom.push_back(plNom.zf());
				col.push_back(plCol.Rf());
				col.push_back(plCol.Gf());
				col.push_back(plCol.Bf());
				col.push_back(plCol.Af());
			}
		}
	}

	// Control mesh
	for (auto plHd = controlMesh_deformed.NullPolygon(); true == controlMesh_deformed.MoveToNextPolygon(plHd); )
	{
		auto plVtHd = controlMesh_deformed.GetPolygonVertex(plHd);
		auto plCol = controlMesh_deformed.GetColor(plHd);
		auto plNom = controlMesh_deformed.GetNormal(plHd);

		// Let's assume every polygon is a triangle for now.
		if (3 == plVtHd.size())
		{
			for (int i = 0; i<3; ++i)
			{
				auto vtPos = controlMesh_deformed.GetVertexPosition(plVtHd[i]);
				vtx_control.push_back(vtPos.xf());
				vtx_control.push_back(vtPos.yf());
				vtx_control.push_back(vtPos.zf());
				nom_control.push_back(plNom.xf());
				nom_control.push_back(plNom.yf());
				nom_control.push_back(plNom.zf());
				col_control.push_back(plCol.Rf());
				col_control.push_back(plCol.Gf());
				col_control.push_back(plCol.Bf());
				col_control.push_back(plCol.Af());
			}
		}
	}
	//std::cout << "# vertices: " << vtx.size() << "\n";
}

YsVec2i ViewPortToWindow(int winWid, int winHei, const YsVec3 &vp)
{
	double s = (vp.x() + 1.0) / 2.0;
	double t = (1.0 - vp.y()) / 2.0;
	double x = (double)winWid*s;
	double y = (double)winHei*t;
	return YsVec2i((int)x, (int)y);
}

YsVec3 WindowToViewPort(int winWid, int winHei, int x, int y)
{
	double vx = 2.0*((double)x / (double)winWid) - 1.0;
	double vy = 1.0 - 2.0*((double)y / (double)winHei);
	return YsVec3(vx, vy, 0.0);
}

YsMatrix4x4 MakePerspective(const double fovy, const double aspect, const double nearz, const double farz)
{
	float mat[16];

	// Based on the formula listed in www.opengl.org
	const float f = (float)(1.0 / tan(fovy / 2.0));
	mat[0] = (float)(f / aspect); mat[4] = 0;        mat[8] = 0;                                  mat[12] = 0;
	mat[1] = 0;                 mat[5] = (float)f; mat[9] = 0;                                  mat[13] = 0;
	mat[2] = 0;                 mat[6] = 0;        mat[10] = (float)((farz + nearz) / (nearz - farz)); mat[14] = (float)((2.0*farz*nearz) / (nearz - farz));
	mat[3] = 0;                 mat[7] = 0;        mat[11] = -1;                                 mat[15] = 0;

	YsMatrix4x4 tfm;
	tfm.CreateFromOpenGlCompatibleMatrix(mat);
	return tfm;
}

YsMatrix4x4 MakeOrthogonal(
	const double left, const double right, const double bottom, const double top, const double nearz, const double farz)
{
	float mat[16];

	// Based on the formula listed in www.opengl.org
	const double tx = -(right + left) / (right - left);
	const double ty = -(top + bottom) / (top - bottom);
	const double tz = -(farz + nearz) / (farz - nearz);
	mat[0] = (float)(2.0 / (right - left)); mat[4] = 0;                         mat[8] = 0;                          mat[12] = (float)tx;
	mat[1] = 0;                         mat[5] = (float)(2.0 / (top - bottom)); mat[9] = 0;                          mat[13] = (float)ty;
	mat[2] = 0;                         mat[6] = 0;                         mat[10] = (float)(-2.0 / (farz - nearz)); mat[14] = (float)tz;
	mat[3] = 0;                         mat[7] = 0;                         mat[11] = 0;                          mat[15] = 1;

	YsMatrix4x4 tfm;
	tfm.CreateFromOpenGlCompatibleMatrix(mat);
	return tfm;
}


YsMatrix4x4 meanValueVis::GetProjection(void) const
{
	int wid, hei;
	FsGetWindowSize(wid, hei);
	auto aspect = (double)wid / (double)hei;
	return MakePerspective(45.0, aspect, d / 10.0, d*2.0);
}

YsMatrix4x4 meanValueVis::GetModelView(void) const
{
	YsMatrix4x4 globalToCamera = Rc;
	globalToCamera.Invert();

	YsMatrix4x4 modelView;
	modelView.Translate(0, 0, -d);
	modelView *= globalToCamera;
	modelView.Translate(-t);
	return modelView;
}

PolygonalMesh::PolygonHandle meanValueVis::PickedPlHd(int mx, int my) const
{
	int wid, hei;
	FsGetWindowSize(wid, hei);
	auto vp = WindowToViewPort(wid, hei, mx, my);

	auto perspective = GetProjection();
	auto modelView = GetModelView();

	YsMatrix4x4 all = perspective*modelView;
	all.Invert();

	auto nearPos = vp, farPos = vp;
	nearPos.SetZ(-1.0);
	farPos.SetZ(1.0);

	YsVec3 ln[2];
	all.Mul(ln[0], nearPos, 1.0);
	all.Mul(ln[1], farPos, 1.0);

	PolygonalMesh::PolygonHandle pickedPlHd = controlMesh_deformed.NullPolygon();
	double pickedZ = 0.0;
	for (auto plHd = controlMesh_deformed.NullPolygon(); true == controlMesh_deformed.MoveToNextPolygon(plHd); )
	{
		std::vector <YsVec3> plVtPos;
		for (auto vtHd : controlMesh_deformed.GetPolygonVertex(plHd))
		{
			plVtPos.push_back(controlMesh_deformed.GetVertexPosition(vtHd));
		}
		YsPlane pln;
		YsVec3 crs;
		if (YSOK == pln.MakeBestFitPlane(plVtPos) &&
			YSOK == pln.GetPenetration(crs, ln[0], ln[1]) &&
			YSINSIDE == YsCheckInsidePolygon3(crs, plVtPos))
		{
			crs = modelView*crs;
			if (controlMesh_deformed.NullPolygon() == pickedPlHd || crs.z()>pickedZ)
			{
				pickedPlHd = plHd;
				pickedZ = crs.z();
			}
		}
	}

	return pickedPlHd;
}

PolygonalMesh::VertexHandle meanValueVis::PickedVtHd(int mx, int my, int pixRange) const
{
	int wid, hei;
	FsGetWindowSize(wid, hei);
	auto vp = WindowToViewPort(wid, hei, mx, my);

	auto projection = GetProjection();
	auto modelView = GetModelView();

	double pickedZ = 0.0;
	auto pickedVtHd = controlMesh_deformed.NullVertex();
	for (auto vtHd = controlMesh_deformed.NullVertex(); true == controlMesh_deformed.MoveToNextVertex(vtHd); )
	{
		auto vtPos = controlMesh_deformed.GetVertexPosition(vtHd);
		vtPos = projection*modelView*vtPos;
		auto winPos = ViewPortToWindow(wid, hei, vtPos);
		int dx = (mx - winPos.x()), dy = (my - winPos.y());
		if (-pixRange <= dx && dx <= pixRange && -pixRange <= dy && dy <= pixRange)
		{
			if (controlMesh_deformed.NullVertex() == pickedVtHd || vtPos.z()<pickedZ)
			{
				pickedVtHd = vtHd;
				pickedZ = vtPos.z();
			}
		}
	}

	return pickedVtHd;
}

// Deforming the control mesh
void meanValueVis::deformControlMesh() {
	//controlMesh_deformed = controlMesh;
	// INITIAL TEST - shifting all the vertices by a fixed amount
	//for (auto vtxHd = controlMesh_deformed.NullVertex(); true == controlMesh_deformed.MoveToNextVertex(vtxHd); ){
	//	// Translating each vertex by some amount in x-direction
	//	controlMesh_deformed.SetVertexPosition(vtxHd, controlMesh_deformed.GetVertexPosition(vtxHd) + YsVec3(1.0,0.0,0.0));
	//}

	// Calculating the mean value coordinate interpolation
	meanValueInterpolateDeformation();

	//// Updating the control mesh
	//for (PolygonalMesh::VertexHandle vtxHd = controlMesh.NullVertex(), vtxHd_deformed = controlMesh_deformed.NullVertex(); 
	//	true == controlMesh.MoveToNextVertex(vtxHd) && true == controlMesh_deformed.MoveToNextVertex(vtxHd_deformed); ) {
	//	// Translating each vertex by some amount in x-direction
	//	controlMesh.SetVertexPosition(vtxHd, controlMesh_deformed.GetVertexPosition(vtxHd_deformed));
	//}
	RemakeVertexArray();
}

void meanValueVis::AddColor(std::vector <float> &col, float r, float g, float b, float a)
{
	col.push_back(r);
	col.push_back(g);
	col.push_back(b);
	col.push_back(a);
}

void meanValueVis::AddVertex(std::vector <float> &vtx, float x, float y, float z)
{
	vtx.push_back(x);
	vtx.push_back(y);
	vtx.push_back(z);
}

void meanValueVis::AddNormal(std::vector <float> &nom, float x, float y, float z)
{
	nom.push_back(x);
	nom.push_back(y);
	nom.push_back(z);
}

void meanValueVis::DeformSelection()
// function to initialize vertices selection for deformation
{
	int mx, my, lb, mb, rb;
	int evt = FsGetMouseEvent(lb, mb, rb, mx, my);
	int wid, hei;
	FsGetWindowSize(wid, hei);

	// mouse left button for selection:
	if (evt == FSMOUSEEVENT_LBUTTONDOWN)
	{
		// store vertex handles for picked vertices:
		auto vtHd = PickedVtHd(mx, my, 20);
		storeVertex(vtHd);
		RemakeVertexArray();
	}

}
 
void meanValueVis::storeVertex(PolygonalMesh::VertexHandle vtHd)
// function stores vextex Handle selected by user
{
	vertexVec.push_back(vtHd);
}

void meanValueVis::mousePositions()
// function deforms the model as per mouse movement
{
	int mx, my, lb, mb, rb;
	int evt = FsGetMouseEvent(lb, mb, rb, mx, my);
	int wid, hei;
	FsGetWindowSize(wid, hei);

	// mouse left button for selection:
	if (evt == FSMOUSEEVENT_LBUTTONDOWN)
	{
		// store vertex handles for picked vertices:
		mouseVec.push_back({ mx,my });
		Deform(mouseVec);
		RemakeVertexArray();
	}
}

inline void meanValueVis::Deform(std::vector < std::vector <int>> positions)
{
	// calculate translation:
	int end = positions.size();
	auto translateX = positions[0][0] - positions[end - 1][0];
	auto translateY = positions[0][1] - positions[end - 1][1];

	for (auto VtHd : vertexVec)
	{
		auto pos = controlMesh_deformed.GetVertexPosition(VtHd);
		cout << "before:\t" << pos[0] << endl;
		
		pos[0] = pos[0] + 0.01*translateX;
		pos[1] = pos[1] + 0.01*translateY;

		cout << "after:\t" << pos[0] << endl;

		controlMesh_deformed.SetVertexPosition(VtHd, pos);
	}
}

void meanValueVis::Clear()
// function clears the storage vector of selected vertices
{
	cout << "Storage array size before clearing:\t" << vertexVec.size() << endl;
	vertexVec.clear();
	mouseVec.clear();
	cout << "Storage size after clearing:\t" << vertexVec.size() << endl;
}


bool meanValueVis::terminate() {
	if (FsGetKeyState(FSKEY_ESC))
	{
		return true;
	}
	return false;
}


// Moving the model
void meanValueVis::move() {
	if (FsGetKeyState(FSKEY_LEFT))
	{
		Rc.RotateXZ(YsPi / 60.0);
	}
	if (FsGetKeyState(FSKEY_RIGHT))
	{
		Rc.RotateXZ(-YsPi / 60.0);
	}
	if (FsGetKeyState(FSKEY_UP))
	{
		Rc.RotateYZ(YsPi / 60.0);
	}
	if (FsGetKeyState(FSKEY_DOWN))
	{
		Rc.RotateYZ(-YsPi / 60.0);
	}
	if (FsGetKeyState(FSKEY_PLUS)) {
		d -= 0.1;
	}
	if (FsGetKeyState(FSKEY_MINUS)) {
		d += 0.1;
	}

}

// Drawing the model mesh
void meanValueVis::draw() const {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	int wid, hei;
	FsGetWindowSize(wid, hei);
	auto aspect = (double)wid / (double)hei;
	glViewport(0, 0, wid, hei);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, aspect, d / 10.0, d*2.0);

	YsMatrix4x4 globalToCamera = Rc;
	globalToCamera.Invert();

	YsMatrix4x4 modelView;  // need #include ysclass.h
	modelView.Translate(0, 0, -d);
	modelView *= globalToCamera;
	modelView.Translate(-t);

	GLfloat modelViewGl[16];
	modelView.GetOpenGlCompatibleMatrix(modelViewGl);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//GLfloat lightDir[]={0.0f,1.0f/(float)sqrt(2.0f),1.0f/(float)sqrt(2.0f),0.0f};
	//glLightfv(GL_LIGHT0,GL_POSITION,lightDir);
	//glEnable(GL_COLOR_MATERIAL);
	//glEnable(GL_LIGHTING);
	//glEnable(GL_LIGHT0);

	glMultMatrixf(modelViewGl);

	// Model mesh
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glColorPointer(4, GL_FLOAT, 0, col.data());
	glNormalPointer(GL_FLOAT, 0, nom.data());
	glVertexPointer(3, GL_FLOAT, 0, vtx.data());
	glDrawArrays(GL_TRIANGLES, 0, vtx.size() / 3);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	// Control Mesh
	for (int idx = 0; idx < vtx_control.size()/3; idx += 3) {
		glBegin(GL_LINE_LOOP);
		glVertex3d(vtx_control[3*idx], vtx_control[3*idx + 1], vtx_control[3*idx + 2]);
		glVertex3d(vtx_control[3 * (idx + 1)], vtx_control[3 * (idx + 1) + 1], vtx_control[3 * (idx + 1) + 2]);
		glVertex3d(vtx_control[3 * (idx + 2)], vtx_control[3 * (idx + 2) + 1], vtx_control[3 * (idx + 2)+ 2]);
		glEnd();
	}

}
