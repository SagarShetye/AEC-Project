#include <vector>

#include <ysclass.h>

#include <fslazywindow.h>

#include "polygonalmesh.h"
#include "objloader.h"
#include "meanValueVisualizer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string>
using namespace std;


class FsLazyWindowApplication : public FsLazyWindowApplicationBase
{
protected:
	bool needRedraw;

	YsMatrix4x4 Rc;
	double d;
	YsVec3 t;

	PolygonalMesh mesh;
	std::vector <float> vtx,nom,col;
	YsVec3 bbx[2];

	meanValueVis visualizer;
	
	bool checkSelection = false, deformation = false;
	
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


FsLazyWindowApplication::FsLazyWindowApplication()
{
	needRedraw=false;
	d=35;
	t=YsVec3::Origin();
}

/* virtual */ void FsLazyWindowApplication::BeforeEverything(int argc,char *argv[])
{
}
/* virtual */ void FsLazyWindowApplication::GetOpenWindowOption(FsOpenWindowOption &opt) const
{
	opt.x0=0;
	opt.y0=0;
	opt.wid=600;
	opt.hei=400;
}
/* virtual */ void FsLazyWindowApplication::Initialize(int argc,char *argv[])
{
	//if (2<=argc && mesh.LoadBinStl(argv[1]))
	if(2<=argc && visualizer.Initialize(argv[1],argv[2]))
	{
		
		std::cout << "MESH LOADED\n";
		mesh.GetBoundingBox(bbx[0],bbx[1]);
		
		t=(bbx[0]+bbx[1])/2.0;
		d=(bbx[1]-bbx[0]).GetLength()*1.2;

		printf("Target %s\n",t.Txt());
		printf("Diagonal %lf\n",d);
		

	}
}
/* virtual */ void FsLazyWindowApplication::Interval(void)
{
	auto key=FsInkey();
	if(FSKEY_ESC==key)
	{
		SetMustTerminate(true);
	}
	// Deforming the control mesh
	if (FSKEY_SPACE == key)
	{
		std::cout << "INTERPOLATION STARTED\n";
		//visualizer.deformControlMesh();
		visualizer.meanValueInterpolateDeformation();
		visualizer.RemakeVertexArray();
	}

	// selection of vertices for deformation:
	if (FSKEY_D == key)
	{
		cout << "Vertices selection for deformation started!" << endl;
		checkSelection = true;
		deformation = false;
	}
	if (checkSelection)
	{
		visualizer.DeformSelection();
	}

	// deformation of the model:
	if (FSKEY_F == key)
	{
		cout << "Selection done!" << endl;
		cout << "Onwards to deformation-->" << endl;
		checkSelection = false;
		deformation = true;
	}

	if (deformation == true && checkSelection == false)
	{
		visualizer.mousePositions();
	}

	if (FSKEY_E == key)
	{
		checkSelection = false;
		deformation = false;
	}

	if (FSKEY_C == key)
	{
		visualizer.Clear();
	}

	visualizer.move();
	needRedraw=true;
}
/* virtual */ void FsLazyWindowApplication::Draw(void)
{
	needRedraw=false;

visualizer.draw();	

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
	return 25;
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
