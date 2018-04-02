#ifndef POLYGONALMESH_IS_INCLUDED
#define POLYGONALMESH_IS_INCLUDED

#include <list>
#include <unordered_map>
#include <ysclass.h>

class PolygonalMesh
{
protected:
	unsigned int searchKeySeed;
public:
	PolygonalMesh();

protected:
	class Vertex
	{
	public:
		YsVec3 pos;
		unsigned int searchKey;
	};
private:
	mutable std::list <Vertex> vtxList;
public:
	class VertexHandle
	{
	friend class PolygonalMesh;
	private:
		std::list <Vertex>::iterator vtxPtr;
	public:
		VertexHandle(){};  // C++11 VertexHandle()=default;
		inline bool operator==(const VertexHandle &vtHd) const
		{
			return vtxPtr==vtHd.vtxPtr;
		}
		inline bool operator!=(const VertexHandle &vtHd) const
		{
			return vtxPtr!=vtHd.vtxPtr;
		}
	};
protected:
	std::unordered_map <unsigned int,VertexHandle> vtxSearch;
public:
	VertexHandle AddVertex(const YsVec3 &pos);
	inline VertexHandle NullVertex(void) const
	{
		VertexHandle vtHd;
		vtHd.vtxPtr=vtxList.end();
		return vtHd;
	}
	YsVec3 GetVertexPosition(VertexHandle vtHd) const;
	bool SetVertexPosition(VertexHandle vtHd,const YsVec3 &newPos);
	long long int GetNumVertex(void) const;
	bool MoveToNextVertex(VertexHandle &vtHd) const;
	unsigned int GetSearchKey(VertexHandle vtHd) const;

protected:
	class Polygon
	{
	public:
		std::vector <VertexHandle> vtHd;
		YsVec3 nom;
		YsColor col;
	};
private:
	mutable std::list <Polygon> plgList;
public:
	class PolygonHandle
	{
	friend class PolygonalMesh;
	private:
		std::list <Polygon>::iterator plgPtr;
	public:
		PolygonHandle(){};  // C++11 PolygonHandle()=default;
		inline bool operator==(const PolygonHandle &plHd) const
		{
			return plgPtr==plHd.plgPtr;
		}
		inline bool operator!=(const PolygonHandle &plHd) const
		{
			return plgPtr!=plHd.plgPtr;
		}
	};
protected:
	std::unordered_map <unsigned int,std::vector <PolygonHandle> > vtKeyToPlg;
public:
	PolygonHandle AddPolygon(int nPlVt,const VertexHandle plVtHd[]);
	PolygonHandle AddPolygon(const std::vector <VertexHandle> &plVtHd);
	inline PolygonHandle NullPolygon(void) const
	{
		PolygonHandle plHd;
		plHd.plgPtr=plgList.end();
		return plHd;
	}
	const std::vector <VertexHandle> GetPolygonVertex(PolygonHandle plHd) const;

	long long int GetNumPolygon(void) const;
	bool MoveToNextPolygon(PolygonHandle &plHd) const;

	std::vector <PolygonHandle> FindPolygonFromVertex(VertexHandle vtHd) const;
	bool SetPolygonColor(PolygonHandle plHd,YsColor col);
	YsColor GetColor(PolygonHandle plHd) const;
	bool SetPolygonNormal(PolygonHandle plHd,const YsVec3 &nom);
	YsVec3 GetNormal(PolygonHandle plHd) const;

	bool SaveBinStl(const char fn[]) const;

	bool LoadBinStl(const char fn[]);
private:
	bool CPUisLittleEndian(void);
	int BinaryToInt(const unsigned char dw[4]);
	float BinaryToFloat(const unsigned char dw[4]);
	void AddBinaryStlTriangle(const unsigned char buf[50]);

public:
	void GetBoundingBox(YsVec3 &min,YsVec3 &max) const;
	void GetBoundingBox(YsVec3 bbx[2]) const;
};

#endif
