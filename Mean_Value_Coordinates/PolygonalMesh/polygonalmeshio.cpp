#include <vector>
#include <stdio.h>
#include "polygonalmesh.h"

bool PolygonalMesh::LoadBinStl(const char fn[])
{
    FILE *fp=fopen(fn,"rb");
    if(nullptr!=fp)
    {
        unsigned char title[80];
        fread(title,1,80,fp); // Skip title

        unsigned char dw[4];
        fread(dw,4,1,fp);  // Read 4 bytes
        auto nTri=BinaryToInt(dw);
        printf("%d triangles\n",nTri);

        int nTriActual=0;
        for(int i=0; i<nTri; ++i)
        {
            unsigned char buf[50];  // 50 bytes per triangle
            if(50==fread(buf,1,50,fp))
            {
                AddBinaryStlTriangle(buf);
                ++nTriActual;
            }
            else
            {
                break;
            }
        }
        printf("Actually read %d\n",nTriActual);

        fclose(fp);
		return true;
    }
	return false;
}

bool PolygonalMesh::CPUisLittleEndian(void)
{
    unsigned int one=1;
    auto *dat=(const unsigned char *)&one;
    if(1==dat[0])
    {
        return true;
    }
    return false;
}
int PolygonalMesh::BinaryToInt(const unsigned char dw[4])
{
    int b0=(int)dw[0];
    int b1=(int)dw[1];
    int b2=(int)dw[2];
    int b3=(int)dw[3];
    return b0+b1*0x100+b2*0x10000+b3*0x1000000;
}
float PolygonalMesh::BinaryToFloat(const unsigned char dw[4])
{
    if(true==CPUisLittleEndian())
    {
        const float *fPtr=(const float *)dw;
        return *fPtr;
    }
    else
    {
        float value;
        auto *valuePtr=(unsigned char *)&value;
        valuePtr[0]=dw[3];
        valuePtr[1]=dw[2];
        valuePtr[2]=dw[1];
        valuePtr[3]=dw[0];
        return value;
    }
}

void PolygonalMesh::AddBinaryStlTriangle(const unsigned char buf[50])
{
    float nx=BinaryToFloat(buf),ny=BinaryToFloat(buf+4),nz=BinaryToFloat(buf+8);
    const YsVec3 nom(nx,ny,nz);

	const YsVec3 tri[3]=
	{
	    YsVec3(BinaryToFloat(buf+12),BinaryToFloat(buf+16),BinaryToFloat(buf+20)),
	    YsVec3(BinaryToFloat(buf+24),BinaryToFloat(buf+28),BinaryToFloat(buf+32)),
	    YsVec3(BinaryToFloat(buf+36),BinaryToFloat(buf+40),BinaryToFloat(buf+44)),
	};

	const VertexHandle triVtHd[3]=
	{
		AddVertex(tri[0]),
		AddVertex(tri[1]),
		AddVertex(tri[2]),
	};

	auto plHd=AddPolygon(3,triVtHd);
	SetPolygonColor(plHd,YsBlue());
	SetPolygonNormal(plHd,nom);
}

//Save polygonal mesh data to binary stl file
bool PolygonalMesh::SaveBinStl(const char fn[]) const
{

    FILE *bin_fp = fopen(fn,"wb");

    if (nullptr!=bin_fp)
    {

        //Write comments
        unsigned char comments[80] = {0};
        fwrite(comments,1,80,bin_fp); //First 80 bytes are comments

        //Write num of polygon data
        int NumPolygon = GetNumPolygon();
        fwrite((unsigned char*)&NumPolygon,1,4,bin_fp); //next 4 bytes are the number of polygons

        int count = 0;
        for(auto plHd=NullPolygon(); true==MoveToNextPolygon(plHd); )
        {

            auto plVtHd=GetPolygonVertex(plHd);
            auto plNom=GetNormal(plHd);

            //Write Normal data
            float nom_x = plNom.xf(), nom_y =  plNom.yf(), nom_z =  plNom.zf();
            fwrite((unsigned char*)&nom_x,1,4,bin_fp); //x coord for normal
            fwrite((unsigned char*)&nom_y,1,4,bin_fp); //y coord for normal
            fwrite((unsigned char*)&nom_z,1,4,bin_fp); //z coord for normal

            //Write vertex data
            for (int i = 0; i < 3; i++)
            {
                auto vtPos=GetVertexPosition(plVtHd[i]);
                float v_x = vtPos.xf() , v_y =vtPos.yf(), v_z =vtPos.zf() ;
                fwrite((unsigned char*)&v_x,1,4,bin_fp); //x coordd for vertex
                fwrite((unsigned char*)&v_y,1,4,bin_fp); //y coord for vertex
                fwrite((unsigned char*)&v_z,1,4,bin_fp); //z coord for vertex
            }

            //Write element id data
            char vid[2] = {count};
            fwrite(vid,1,2,bin_fp);

            count++;
        }

        fclose(bin_fp);
        return true;
    }
	return false;
}
