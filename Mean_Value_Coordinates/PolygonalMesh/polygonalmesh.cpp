#include "polygonalmesh.h"

PolygonalMesh::PolygonalMesh()
{
	searchKeySeed=1;
}

PolygonalMesh::VertexHandle PolygonalMesh::AddVertex(const YsVec3 &pos)
{
	Vertex vtx;
	vtx.pos=pos;
	vtx.searchKey=searchKeySeed;
	vtxList.push_back((Vertex &&)vtx);
	VertexHandle vtHd;
	vtHd.vtxPtr=vtxList.end();
	vtHd.vtxPtr--;
	vtxSearch[searchKeySeed]=vtHd;
	++searchKeySeed;
	return vtHd;
}

bool PolygonalMesh::SetVertexPosition(VertexHandle vtHd,const YsVec3 &newPos)
{
	if(NullVertex()!=vtHd)
	{
		vtHd.vtxPtr->pos=newPos;
		return true;
	}
	return false;
}

YsVec3 PolygonalMesh::GetVertexPosition(VertexHandle vtHd) const
{
	if(NullVertex()==vtHd)
	{
		return YsVec3::Origin();
	}
	return vtHd.vtxPtr->pos;
}

long long int PolygonalMesh::GetNumVertex(void) const
{
	return vtxList.size();
}
bool PolygonalMesh::MoveToNextVertex(VertexHandle &vtHd) const
{
	if(NullVertex()==vtHd)
	{
		vtHd.vtxPtr=vtxList.begin();
	}
	else
	{
		++vtHd.vtxPtr;
	}
	if(NullVertex()!=vtHd)
	{
		return true;
	}
	return false;
}
unsigned int PolygonalMesh::GetSearchKey(VertexHandle vtHd) const
{
	if(NullVertex()==vtHd)
	{
		return 0xffffffff;
	}
	return vtHd.vtxPtr->searchKey;
}

PolygonalMesh::PolygonHandle PolygonalMesh::AddPolygon(int nPlVt,const VertexHandle plVtHd[])
{
	Polygon plg;
	plg.vtHd.resize(nPlVt);
	for(int i=0; i<nPlVt; ++i)
	{
		plg.vtHd[i]=plVtHd[i];
	}
	plgList.push_back((Polygon &&)plg);
	PolygonHandle plHd;
	plHd.plgPtr=plgList.end();
	--plHd.plgPtr;
	for(int i=0; i<nPlVt; ++i)
	{
		vtKeyToPlg[GetSearchKey(plVtHd[i])].push_back(plHd);
	}
	return plHd;
}
PolygonalMesh::PolygonHandle PolygonalMesh::AddPolygon(const std::vector <VertexHandle> &plVtHd)
{
	return AddPolygon((int)plVtHd.size(),plVtHd.data());
}
const std::vector <PolygonalMesh::VertexHandle> PolygonalMesh::GetPolygonVertex(PolygonHandle plHd) const
{
	if(NullPolygon()!=plHd)
	{
		return plHd.plgPtr->vtHd;
	}
	std::vector <PolygonalMesh::VertexHandle> plVtHd;
	return plVtHd;
}
long long int PolygonalMesh::GetNumPolygon(void) const
{
	return plgList.size();
}
bool PolygonalMesh::MoveToNextPolygon(PolygonHandle &plHd) const
{
	if(NullPolygon()==plHd)
	{
		plHd.plgPtr=plgList.begin();
	}
	else
	{
		++plHd.plgPtr;
	}
	if(NullPolygon()!=plHd)
	{
		return true;
	}
	return false;
}

std::vector <PolygonalMesh::PolygonHandle> PolygonalMesh::FindPolygonFromVertex(VertexHandle vtHd) const
{
	auto found=vtKeyToPlg.find(GetSearchKey(vtHd));
	if(vtKeyToPlg.end()!=found)
	{
		return found->second;
	}
	std::vector <PolygonHandle> vtPlHd;
	return vtPlHd;
}

bool PolygonalMesh::SetPolygonColor(PolygonHandle plHd,YsColor col)
{
	if(NullPolygon()!=plHd)
	{
		plHd.plgPtr->col=col;
		return true;
	}
	return false;
}
YsColor PolygonalMesh::GetColor(PolygonHandle plHd) const
{
	if(NullPolygon()!=plHd)
	{
		return plHd.plgPtr->col;
	}
	return YsBlack();
}
bool PolygonalMesh::SetPolygonNormal(PolygonHandle plHd,const YsVec3 &nom)
{
	if(NullPolygon()!=plHd)
	{
		plHd.plgPtr->nom=nom;
		return true;
	}
	return false;
}
YsVec3 PolygonalMesh::GetNormal(PolygonHandle plHd) const
{
	if(NullPolygon()!=plHd)
	{
		return plHd.plgPtr->nom;
	}
	return YsVec3::Origin();
}

void PolygonalMesh::GetBoundingBox(YsVec3 &min,YsVec3 &max) const
{
	YsBoundingBoxMaker3 mkBbx;
	for(auto vtHd=NullVertex(); true==MoveToNextVertex(vtHd); )
	{
		mkBbx.Add(GetVertexPosition(vtHd));
	}
	mkBbx.Get(min,max);
}
void PolygonalMesh::GetBoundingBox(YsVec3 bbx[2]) const
{
	YsBoundingBoxMaker3 mkBbx;
	for(auto vtHd=NullVertex(); true==MoveToNextVertex(vtHd); )
	{
		mkBbx.Add(GetVertexPosition(vtHd));
	}
	mkBbx.Get(bbx);
}
