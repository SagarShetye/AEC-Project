#include <vector>

#include <ysclass.h>
#include <glutil.h>
#include "meshDeform.h"
#include "meshColour.h"

#include <fslazywindow.h>

#include "ysshellext.h"
#include "objloader.h"
#include "meanValueInterpolation.h"

#include <unordered_set>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include<iostream>

#define INF_D (std::numeric_limits<float>::infinity())

// TEST
void meanValueInterpolateDeformation(YsShellExt &modelMesh, YsShellExt &controlMesh, YsShellExt &controlMesh_deformed,
	float angleTolerance, float distTolerance, YsVec3 bbox[2]);

class FsLazyWindowApplication : public FsLazyWindowApplicationBase
{
protected:
	YsShellExt controlMeshUndeformed;
	bool needRedraw;

	YsMatrix4x4 Rc;
	double d;
	YsVec3 t;
	std::unordered_set <YSHASHKEY> PickedVertices; //unordered set of vertices picked by the mouse to move to ensure no vertex is added twice
	bool moveVertex; //flag to indicate whether to move vertices of control mesh or not
	bool moveCluster; //flag for moving the clustered vertex of control mesh ---k means clustering
	std::unordered_map <int,YsVec3> K_Points; //kmean clustering - the points
	int PickedPoint; //Cpoint from the kmean cluster being picked;
	int group_kmean; //no of clusters to form;
	std::unordered_map <YSHASHKEY,int> K_Groups; //kmean clustering  - the groups
	bool dispMesh; //flag for not displaying model mesh while moving clustered  k group points

	YsShellExt Model_Mesh; //The Model mesh
	YsShellExt Control_Mesh; //The control mesh
	std::vector <std::unordered_map <YSHASHKEY,float>> Weights_Map; //The weights map for every vertices of Model_Mesh

	std::vector <float> vtx,nom,col; //For model mesh
	std::vector <float> vtx_control, nom_control, col_control; //For Control Mesh
	std::vector <float> vtx_highlight,col_highlight; //for picked vertices
	std::vector <float> vtx_k,col_k; //for points in k groups

	YsVec3 bbx[2];//bounding box of the mesh

	
	static void AddColor(std::vector <float> &col,float r,float g,float b,float a);
	static void AddVertex(std::vector <float> &vtx,float x,float y,float z);
	static void AddNormal(std::vector <float> &nom,float x,float y,float z);


	YsMatrix4x4 GetProjection(void) const;
	YsMatrix4x4 GetModelView(void) const;
	YsShellExt::PolygonHandle PickedPlHd(int mx,int my) const; //Function for picking the polygon of the mesh
	YsShellExt::VertexHandle PickedVtHd(int mx,int my,int pixRange) const; //Function for picking vertex of the mesh
	int PickedClusterPoint(int mx,int my, std::unordered_map <int,YsVec3> K_Points, int pixRange) const; //Function for picking the point for kmeans cluster

	void RemakeVertexArray(void);

public:
	FsLazyWindowApplication();
	virtual void BeforeEverything(int argc,char *argv[]);
	virtual void GetOpenWindowOption(FsOpenWindowOption &OPT) const;
	virtual void Initialize(int argc,char *argv[]);
	virtual void Interval(void);
	virtual void BeforeTerminate(void);
	virtual void Draw(void);
	virtual bool UserWantToCloseProgram(void);
	virtual bool MustTerminate(void) const;
	virtual long long int GetMinimumSleepPerInterval(void) const;
	virtual bool NeedRedraw(void) const;
};

/* static */ void FsLazyWindowApplication::AddColor(std::vector <float> &col,float r,float g,float b,float a)
{
	col.push_back(r);
	col.push_back(g);
	col.push_back(b);
	col.push_back(a);
}
/* static */ void FsLazyWindowApplication::AddVertex(std::vector <float> &vtx,float x,float y,float z)
{
	vtx.push_back(x);
	vtx.push_back(y);
	vtx.push_back(z);
}
/* static */ void FsLazyWindowApplication::AddNormal(std::vector <float> &nom,float x,float y,float z)
{
	nom.push_back(x);
	nom.push_back(y);
	nom.push_back(z);
}

void FsLazyWindowApplication::RemakeVertexArray(void)
{
	vtx.clear();
	col.clear();
	nom.clear();

	vtx_control.clear();
	col_control.clear();
	nom_control.clear();

	vtx_highlight.clear();
	col_highlight.clear();

	vtx_k.clear();
	col_k.clear();

	//Model Mesh
	for(auto plHd=Model_Mesh.NullPolygon(); true==Model_Mesh.MoveToNextPolygon(plHd); )
	{
		auto plVtHd=Model_Mesh.GetPolygonVertex(plHd);
		auto plCol=Model_Mesh.GetColor(plHd);
		auto plNom=Model_Mesh.GetNormal(plHd);

		
		for(int i=0; i<plVtHd.size(); ++i)
		{
			auto vtPos=Model_Mesh.GetVertexPosition(plVtHd[i]);
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

	// Control mesh
	for (auto plHd = Control_Mesh.NullPolygon(); true == Control_Mesh.MoveToNextPolygon(plHd); )
	{
		auto plVtHd = Control_Mesh.GetPolygonVertex(plHd);
		auto plCol = Control_Mesh.GetColor(plHd);
		auto plNom = Control_Mesh.GetNormal(plHd);

		// Let's assume every polygon is a triangle for now.
		if (3 == plVtHd.size())
		{
			for (int i = 0; i<plVtHd.size(); ++i)
			{
				auto vtPos = Control_Mesh.GetVertexPosition(plVtHd[i]);
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

	if (moveVertex)
	{
		//for highlighting picked vertices
		Control_Mesh.EnableSearch();	
		for (auto &p: PickedVertices)
		{
			auto vtHd = Control_Mesh.FindVertex(p);
			auto vtPos = Control_Mesh.GetVertexPosition(vtHd);
			vtx_highlight.push_back(vtPos.xf());
			vtx_highlight.push_back(vtPos.yf());
			vtx_highlight.push_back(vtPos.zf());
			col_highlight.push_back(1.0);
			col_highlight.push_back(0.0);
			col_highlight.push_back(0.0);
			col_highlight.push_back(1.0);
		}

	}

	if (moveCluster)
	{
		if (PickedPoint != -1)
		{	
			auto vtPos = K_Points.find(PickedPoint)->second;
			vtx_highlight.push_back(vtPos.xf());
			vtx_highlight.push_back(vtPos.yf());
			vtx_highlight.push_back(vtPos.zf());
			col_highlight.push_back(1.0);
			col_highlight.push_back(0.0);
			col_highlight.push_back(0.0);
			col_highlight.push_back(1.0);
		}

	}
	
	//For K Points
	for (auto &k : K_Points)
	{
		auto vtPos  = k.second;
		vtx_k.push_back(vtPos.xf());
		vtx_k.push_back(vtPos.yf());
		vtx_k.push_back(vtPos.zf());
		col_k.push_back(0.0);
		col_k.push_back(1.0);
		col_k.push_back(0.0);
		col_k.push_back(1.0);

	}

	
}

YsMatrix4x4 FsLazyWindowApplication::GetProjection(void) const
{
	int wid,hei;
	FsGetWindowSize(wid,hei);
	auto aspect=(double)wid/(double)hei;
	return MakePerspective(45.0,aspect,d/10.0,d*2.0);
}

YsMatrix4x4 FsLazyWindowApplication::GetModelView(void) const
{
	YsMatrix4x4 globalToCamera=Rc;
	globalToCamera.Invert();

	YsMatrix4x4 modelView;
	modelView.Translate(0,0,-d);
	modelView*=globalToCamera;
	modelView.Translate(-t);
	return modelView;
}

//Get the picked polygon handle of control mesh
YsShellExt::PolygonHandle FsLazyWindowApplication::PickedPlHd(int mx,int my) const
{
	YsVec3 mos[2];

	int wid,hei;
	FsGetWindowSize(wid,hei);
	auto p=WindowToViewPort(wid,hei,mx,my);
	mos[0]=p;
	mos[1]=p;
	mos[0].SetZ(-1.0);
	mos[1].SetZ( 1.0);

	auto pers=GetProjection();
	pers.MulInverse(mos[0],mos[0],1.0);
	pers.MulInverse(mos[1],mos[1],1.0);

	auto modelView=GetModelView();
	modelView.MulInverse(mos[0],mos[0],1.0);
	modelView.MulInverse(mos[1],mos[1],1.0);

	double maxZ=0.0;
	YsShellExt::PolygonHandle plHd=nullptr,pickedPlHd=nullptr;
	while(true==Control_Mesh.MoveToNextPolygon(plHd))
	{
		auto plVtHd=Control_Mesh.GetPolygonVertex(plHd);
		std::vector <YsVec3> plVtPos;
		plVtPos.resize(plVtHd.size());
		for(int i=0; i<plVtHd.size(); ++i)
		{
			plVtPos[i]=Control_Mesh.GetVertexPosition(plVtHd[i]);
		}

		YsPlane pln;
		if(YSOK==pln.MakeBestFitPlane(plVtPos))
		{
			YsVec3 itsc;
			if(YSTRUE==pln.GetPenetration(itsc,mos[0],mos[1]))
			{
				auto side=YsCheckInsidePolygon3(itsc,plVtPos);
				if(YSINSIDE==side || YSBOUNDARY==side)
				{
					auto itscInView=modelView*itsc;
					if(nullptr==pickedPlHd || itscInView.z()>maxZ)
					{
						maxZ=itscInView.z();
						pickedPlHd=plHd;
					}
				}
			}
		}
	}

	return pickedPlHd;
}


//Get the vertex handle of the of the vertex of the control mesh picked by mouse
YsShellExt::VertexHandle FsLazyWindowApplication::PickedVtHd(int mx,int my,int pixRange) const
{
	int wid,hei;
	FsGetWindowSize(wid,hei);


	//printf("distance from camera %lf\n", d);
	//printf("mouse positions: %d %d\n",mx,my);


	//auto vp=WindowToViewPort(wid,hei,mx,my);


	auto projection=GetProjection();
	auto modelView=GetModelView();

	double pickedZ=0.0;

	
	//auto pickedVtHd=Control_Mesh.NullVertex();
	//for(auto vtHd=Control_Mesh.NullVertex(); true==Control_Mesh.MoveToNextVertex(vtHd); )
	//{
	//	auto vtPos=Control_Mesh.GetVertexPosition(vtHd);
	//	vtPos=projection*modelView*vtPos;
	//	auto winPos=ViewPortToWindow(wid,hei,vtPos); //here ViewPort means the clip coordinates (i guess)
	//	

	//	//printf("%d winPos: %lf %lf %lf\n",Control_Mesh.GetSearchKey(vtHd), winPos.xf(),winPos.yf(),vtPos.z());

	//	int dx=(mx-winPos.x()),dy=(my-winPos.y());
	//	//double d = sqrt(dx*dx + dy*dy);
	//	if(-pixRange<=dx && dx<=pixRange && -pixRange<=dy && dy<=pixRange)
	//	{
	//		if(Control_Mesh.NullVertex()==pickedVtHd || vtPos.z()<pickedZ)
	//		{
	//			pickedVtHd=vtHd;
	//			pickedZ=vtPos.z();
	//		}
	//	}
	//}
	
	
	// TEST
	YsShellExt::PolygonHandle pickedPolygon = PickedPlHd(mx,my);
	float distance = INF_D;
	auto vertices = Control_Mesh.GetPolygonVertex(pickedPolygon);
	YsShellExt::VertexHandle pickedVtHd = Control_Mesh.NullVertex();;
	for (auto vertex : vertices) 
	{
		auto vtPos = Control_Mesh.GetVertexPosition(vertex);
		vtPos = projection*modelView*vtPos;
		auto winPos = ViewPortToWindow(wid, hei, vtPos);
		int dx = (mx - winPos.x()), dy = (my - winPos.y());

		printf("%d distance: %lf\n",  Control_Mesh.GetSearchKey(vertex), sqrt(dx*dx + dy*dy));
		if (sqrt(dx*dx + dy*dy) < distance)
		{
			pickedVtHd = vertex;
			distance = sqrt(dx*dx + dy*dy);
		}
			
	}
	

	return pickedVtHd;

}

//This function returns the Points from K_Points picked by the mouse
int FsLazyWindowApplication::PickedClusterPoint(int mx,int my, std::unordered_map <int,YsVec3> K_Points, int pixRange) const
{


	//printf("mouse positions: %d %d\n",mx,my);

	int wid, hei;
	FsGetWindowSize(wid,hei);
	auto projection = GetProjection(); //Get projection matrix
	auto modelView = GetModelView(); //Get model view matrix

	double pickedZ = INF_D;


	int PickedPoint = -1;
	for (auto &k : K_Points)
	{
		auto Point_Position = k.second;
		Point_Position = projection*modelView*Point_Position;
		auto winPos=ViewPortToWindow(wid,hei,Point_Position); //here ViewPort means the clip coordinates (i guess)


		//printf("%d winPos: %lf %lf %lf\n", k.first , winPos.xf(),winPos.yf(),Point_Position.z());

		int dx=(mx-winPos.x()),dy=(my-winPos.y());

		//dist = sqrt(dx*dx + dy*dy);
		if(-pixRange<=dx && dx<=pixRange && -pixRange<=dy && dy<=pixRange)
		{
			if(Point_Position.z()<pickedZ)
			{
				PickedPoint = k.first;
				pickedZ = Point_Position.z();
			}
		}
	}

	return PickedPoint;

}

///////////////////////////////////////////////////////////////////
FsLazyWindowApplication::FsLazyWindowApplication()
{
	needRedraw=false;
	//d=35;
	t=YsVec3::Origin();
	moveVertex = false; //by default move vertex is false;
	moveCluster = false;
	dispMesh = true; //by default dispMesh is true;
	PickedPoint = -1; //id of cluster point picked
	group_kmean = 2;//by default no oof cluster points is 2;

}



/* virtual */ void FsLazyWindowApplication::BeforeEverything(int argc,char *argv[])
{
}
/* virtual */ void FsLazyWindowApplication::GetOpenWindowOption(FsOpenWindowOption &opt) const
{
	opt.x0=0;
	opt.y0=0;
	opt.wid=1200;
	opt.hei=800;
}
/* virtual */ void FsLazyWindowApplication::Initialize(int argc,char *argv[])
{
	//if (2<=argc && mesh.LoadBinStl(argv[1]))
	if(3<=argc)
	{
	
		LoadObjFile(Model_Mesh,argv[1]); //load the  model mesh
		LoadObjFile(Control_Mesh,argv[2]); //Load  the control  mesh

		LoadObjFile(controlMeshUndeformed, argv[2]); //Load  the undeformed control  mesh

		Weights_Map = GetMeanValueCoordinates(Model_Mesh,Control_Mesh); //Calculate the weights
		
		RemakeVertexArray();
		Control_Mesh.GetBoundingBox(bbx[0],bbx[1]);
		
		t=(bbx[0]+bbx[1])/2.0;
		d=(bbx[1]-bbx[0]).GetLength()*1.2;

		//printf("Target %s\n",t.Txt());
		//printf("Diagonal %lf\n",d);
		

	}
}
/* virtual */ void FsLazyWindowApplication::Interval(void)
{
	auto key=FsInkey();
	if(FSKEY_ESC==key)
	{
		SetMustTerminate(true);
	}

	if(FsGetKeyState(FSKEY_LEFT)) //rotate left
	{		
		Rc.RotateXZ(YsPi/60.0);		
	}
	if(FsGetKeyState(FSKEY_RIGHT)) //rotate right
	{		
		Rc.RotateXZ(-YsPi/60.0);	
	}
	if(FsGetKeyState(FSKEY_UP))  //rotate up
	{
		Rc.RotateYZ(YsPi/60.0);
	}
	if(FsGetKeyState(FSKEY_DOWN)) //rotate down
	{
		Rc.RotateYZ(-YsPi/60.0);
	}
	if (FsGetKeyState(FSKEY_PLUS)) //zoom in
	{
		d -= 0.1;		
	}
	if (FsGetKeyState(FSKEY_MINUS)) //zoom out
	{
		d += 0.1;	
	}

	if(FsGetKeyState(FSKEY_TEN6)) //move vertices in x
	{
		if (moveVertex)
		{
			MoveControlMesh_vertex(Control_Mesh,PickedVertices,YsVec3(0.05,0.00,0.00));
			RemakeVertexArray();
		}
		if (moveCluster)
		{
			MoveControlMesh_cluster(Control_Mesh, K_Points, K_Groups, PickedPoint, YsVec3(0.05,0.00,0.00));
			RemakeVertexArray();
		}
	}
	if(FsGetKeyState(FSKEY_TEN4)) //move vertices in -x
	{
		if (moveVertex)
		{
			MoveControlMesh_vertex(Control_Mesh,PickedVertices,YsVec3(-0.05,0.00,0.00));
			RemakeVertexArray();			
		}
		if (moveCluster)
		{
			MoveControlMesh_cluster(Control_Mesh, K_Points, K_Groups, PickedPoint, YsVec3(-0.05,0.00,0.00));
			RemakeVertexArray();
		}
		
	}	
	if(FsGetKeyState(FSKEY_TEN8)) //move vertiecs in +y
	{
		if (moveVertex)
		{
			MoveControlMesh_vertex(Control_Mesh,PickedVertices,YsVec3(0.00,0.05,0.00));
			RemakeVertexArray();
		}
		if (moveCluster)
		{
			MoveControlMesh_cluster(Control_Mesh, K_Points, K_Groups, PickedPoint, YsVec3(0.00,0.05,0.00));
			RemakeVertexArray();
		}		
	}
	if(FsGetKeyState(FSKEY_TEN2)) //move vertices in -y
	{
		if (moveVertex)
		{
			MoveControlMesh_vertex(Control_Mesh,PickedVertices,YsVec3(0.00,-0.05,0.00));
			RemakeVertexArray();
		}
		if (moveCluster)
		{
			MoveControlMesh_cluster(Control_Mesh, K_Points, K_Groups, PickedPoint, YsVec3(0.00,-0.05,0.00));
			RemakeVertexArray();
		}
	}
	if (FsGetKeyState(FSKEY_TENPLUS)) //move vertices in +z
	{
		if (moveVertex)
		{
			MoveControlMesh_vertex(Control_Mesh,PickedVertices,YsVec3(0.00,0.00,0.05));
			RemakeVertexArray();
		}
		if (moveCluster)
		{
			MoveControlMesh_cluster(Control_Mesh, K_Points, K_Groups, PickedPoint, YsVec3(0.00,0.00,0.05));
			RemakeVertexArray();
		}	
	}
	if (FsGetKeyState(FSKEY_TENMINUS)) //move vertices in -z
	{
		if (moveVertex)
		{
			MoveControlMesh_vertex(Control_Mesh,PickedVertices,YsVec3(0.00,0.00,-0.05));
			RemakeVertexArray();
		}
		if (moveCluster)
		{
			MoveControlMesh_cluster(Control_Mesh, K_Points, K_Groups, PickedPoint, YsVec3(0.00,0.00,-0.05));
			RemakeVertexArray();
		}		
	}



	if (FsGetKeyState(FSKEY_V)) //enable move vertices
	{
		moveVertex = true;
		moveCluster = false;
		PickedVertices.clear();
		RemakeVertexArray();	
	}
	if(FsGetKeyState(FSKEY_K)) //enable k means clustering
	{
		moveCluster = true;
		dispMesh = false;
		moveVertex = false;
		PickedPoint = -1;
		K_Points.clear();
		K_Groups.clear();
		K_Means(K_Points,K_Groups,Control_Mesh,group_kmean);
		RemakeVertexArray();
	}
	if (FsGetKeyState(FSKEY_D)) //disable move vertices and k means clustering
	{		
		moveVertex = false;
		moveCluster = false;
		PickedVertices.clear();
		PickedPoint = -1;
		K_Points.clear();
		K_Groups.clear();
		dispMesh = true;
		RemakeVertexArray();		
	}

	if (FsGetKeyState(FSKEY_B)) //scale up
	{
		ScaleUp(Control_Mesh);
		RemakeVertexArray();
	}
	if (FsGetKeyState(FSKEY_S)) //scale  down
	{
		ScaleDown(Control_Mesh);
		RemakeVertexArray();
	}

	
	//Move Model Mesh
	if (FsGetKeyState(FSKEY_T))
	{

		MoveModelMesh(Control_Mesh, Model_Mesh,Weights_Map);

		//// TEST
		//meanValueInterpolateDeformation(Model_Mesh, controlMeshUndeformed,Control_Mesh,0.01f,0.01f,bbx);
		RemakeVertexArray();
	}
	//Save the mesh
	if (FsGetKeyState(FSKEY_ENTER))
	{
		SaveObj(Model_Mesh,"save.obj");
	}

	int lb,mb,rb,mx,my;
	auto evt=FsGetMouseEvent(lb,mb,rb,mx,my);
	if(evt==FSMOUSEEVENT_LBUTTONDOWN)
	{

		if (moveVertex)
		{
			auto pickedVtHd=PickedVtHd(mx,my,30);
			if(nullptr!=pickedVtHd)
			{
				printf("%d \n", Control_Mesh.GetSearchKey(pickedVtHd));
				PickedVertices.insert(Control_Mesh.GetSearchKey(pickedVtHd));
				RemakeVertexArray();
			}
		}

		if (moveCluster)
		{
			PickedPoint = PickedClusterPoint(mx,my,K_Points,30);
			RemakeVertexArray();
		}

	}



	needRedraw=true;
}
/* virtual */ void FsLazyWindowApplication::Draw(void)
{
	needRedraw=false;

	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	int wid,hei;
	FsGetWindowSize(wid,hei);
	auto aspect=(double)wid/(double)hei;
	glViewport(0,0,wid,hei);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0,aspect,d/10.0,d*2.0);

	YsMatrix4x4 globalToCamera=Rc;
	globalToCamera.Invert();

	YsMatrix4x4 modelView;  // need #include ysclass.h
	modelView.Translate(0,0,-d);
	modelView*=globalToCamera;
	modelView.Translate(-t);

	GLfloat modelViewGl[16];
	modelView.GetOpenGlCompatibleMatrix(modelViewGl);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	

	GLfloat lightDir[]={0.0f,1.0f/(float)sqrt(2.0f),1.0f/(float)sqrt(2.0f),0.0f};
	glLightfv(GL_LIGHT0,GL_POSITION,lightDir);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glMultMatrixf(modelViewGl);

	//Draw Model Mesh
	if (dispMesh == true)
	{	
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);
		glColorPointer(4,GL_FLOAT,0,col.data());
		glNormalPointer(GL_FLOAT,0,nom.data());
		glVertexPointer(3,GL_FLOAT,0,vtx.data());
		glDrawArrays(GL_TRIANGLES,0,vtx.size()/3);
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
		glDisableClientState(GL_COLOR_ARRAY);
	}
	

	// Draw Control Mesh
	for (int idx = 0; idx < vtx_control.size()/3; idx += 3) {
		glBegin(GL_LINE_LOOP);
		glVertex3d(vtx_control[3*idx], vtx_control[3*idx + 1], vtx_control[3*idx + 2]);
		glVertex3d(vtx_control[3 * (idx + 1)], vtx_control[3 * (idx + 1) + 1], vtx_control[3 * (idx + 1) + 2]);
		glVertex3d(vtx_control[3 * (idx + 2)], vtx_control[3 * (idx + 2) + 1], vtx_control[3 * (idx + 2)+ 2]);
		glEnd();
	}


	//Draw Control Nodes	
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glPointSize(10.0f);
	glVertexPointer(3,GL_FLOAT,0,vtx_highlight.data());
	glColorPointer(4,GL_FLOAT,0,col_highlight.data());
	glDrawArrays(GL_POINTS,0,vtx_highlight.size()/3);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
	

	//Draw K Points	
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glPointSize(10.0f);
	glVertexPointer(3,GL_FLOAT,0,vtx_k.data());
	glColorPointer(4,GL_FLOAT,0,col_k.data());
	glDrawArrays(GL_POINTS,0,vtx_k.size()/3);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
	

	FsSwapBuffers();
}
/* virtual */ bool FsLazyWindowApplication::UserWantToCloseProgram(void)
{
	return true; // Returning true will just close the program.
}
/* virtual */ bool FsLazyWindowApplication::MustTerminate(void) const
{
	return FsLazyWindowApplicationBase::MustTerminate();
}
/* virtual */ long long int FsLazyWindowApplication::GetMinimumSleepPerInterval(void) const
{
	return 10;
}
/* virtual */ void FsLazyWindowApplication::BeforeTerminate(void)
{
}
/* virtual */ bool FsLazyWindowApplication::NeedRedraw(void) const
{
	return needRedraw;
}


static FsLazyWindowApplication *appPtr=nullptr;

/* static */ FsLazyWindowApplicationBase *FsLazyWindowApplicationBase::GetApplication(void)
{
	if(nullptr==appPtr)
	{
		appPtr=new FsLazyWindowApplication;
	}
	return appPtr;
}




// TEST (Mean Value Deformation)
// Function to interpolate the mean value coordinates for a triangular mesh
// It takes 2 parameters viz. mesh of the model and the control mesh
// It calculates the required deformation value at each vertex
void meanValueInterpolateDeformation(YsShellExt &modelMesh, YsShellExt &controlMesh, YsShellExt &controlMesh_deformed,
							float angleTolerance, float distTolerance, YsVec3 bbox[2]) {
	distTolerance = 0.001*(bbox[1] - bbox[0]).GetLength();
	// Iterating over the vertices of the model mesh
	auto vtxHd = modelMesh.NullVertex();
	modelMesh.MoveToNextVertex(vtxHd);
	for (; true == modelMesh.MoveToNextVertex(vtxHd);) {
		//std::cout << "VERTEX LOOP\n";
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
			auto vtxHandles = controlMesh.GetPolygonVertex(plgHd);		// Vertices in the undeformed control mesh polygon

			auto vtxHandles_deformed = controlMesh_deformed.GetPolygonVertex(plgHd_deformed);		// Vertices in the deformed control mesh polygon
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
			float dimX = bbox[1].x() - bbox[0].x(), dimY = bbox[1].y() - bbox[0].y(),
				dimZ = bbox[1].z() - bbox[0].z();
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

			w0 = (theta_0 - c1*theta_2 - c2*theta_1) / (2 * d0*sin(theta_2)*sqrt(1 - c1*c1));
			w1 = (theta_1 - c0*theta_2 - c2*theta_0) / (2 * d1*sin(theta_0)*sqrt(1 - c2*c2));
			w2 = (theta_2 - c1*theta_0 - c0*theta_1) / (2 * d2*sin(theta_1)*sqrt(1 - c0*c0));


			//totalF += w0*controlMesh_deformed.GetVertexPosition(vtxHandles_deformed[0]) + w1*controlMesh_deformed.GetVertexPosition(vtxHandles_deformed[1])
			//	+ w2*controlMesh_deformed.GetVertexPosition(vtxHandles_deformed[2]);
			totalF += w0*p0 + w1*p1 + w2*p2;
			float w = w0 + w1 + w2;
			//std::cout << "w: " << w << "\n";
			//w = w > 1.0 ? 1.0 : w;			// clamping between [0,1]
			totalW += w;

			//// Moving to the next polygon of the deformed control mesh
			//controlMesh_deformed.MoveToNextPolygon(plgHd_deformed);										
		}
		if (totalW == 0)
			std::cout << "total weight = 0\n";

		// Updating the vertex position of the model based on the interpolated value from the control mesh
		if (interpolate) {
			YsVec3 newPos = totalF / totalW;
			//std::cout << "X: " << totalF.x() << "\tY: " << totalF.y() << "\tZ: " << totalF.z() << "\n";
			modelMesh.SetVertexPosition(vtxHd, newPos);
		}
		//std::cout << "newX: " << newPos.x() << "\tnewY: " << newPos.y() << "\tnewZ: " << newPos.z() << "\n";
	}

}