/* ////////////////////////////////////////////////////////////

File Name: ysglparticlemanager.cpp
Copyright (c) 2017 Soji Yamakawa.  All rights reserved.
http://www.ysflight.com

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation 
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//////////////////////////////////////////////////////////// */

#include <ysmergesort.h>
#include <ysclass11.h>

#include "ysglparticlemanager.h"


YsGLParticleManager::YsGLParticleManager()
{
	CleanUp();
}
YsGLParticleManager::~YsGLParticleManager()
{
}
void YsGLParticleManager::CleanUp(void)
{
	vtxBuf.CleanUp();
	texCoordBuf.CleanUp();
	colBuf.CleanUp();
	psBuf.CleanUp();
	particle.CleanUp();
	particleOrder.CleanUp();
	particleDist.CleanUp();
}
YSSIZE_T YsGLParticleManager::GetNumParticle(void) const
{
	return particle.GetN();
}

void YsGLParticleManager::CalculateDotProd(YSSIZE_T i0,YSSIZE_T i1,const YsVec3 &viewPos,const YsVec3 & viewDir)
{
	for(auto idx=i0; idx<i1; ++idx)
	{
		particleOrder[idx]=idx;
		particleDist[idx]=-(particle[idx].pos-viewPos)*viewDir;
	}
}

void YsGLParticleManager::Sort(const YsVec3 &viewPos,const YsVec3 &viewDir)
{
	particleOrder.Set(particle.GetN(),nullptr);
	particleDist.Set(particle.GetN(),nullptr);

	CalculateDotProd(0,particle.size(),viewPos,viewDir);

	YsSimpleMergeSort <double,YSSIZE_T> (particleDist.GetN(),particleDist,particleOrder);
}
void YsGLParticleManager::Sort(const YsVec3 &viewPos,const YsVec3 &viewDir,class YsThreadPool &thrPool)
{
	particleOrder.Set(particle.GetN(),nullptr);
	particleDist.Set(particle.GetN(),nullptr);

	if(particle.size()<thrPool.size()*2 || thrPool.size()<2)
	{
		CalculateDotProd(0,particle.size(),viewPos,viewDir);
	}
	else
	{
		std::vector <std::function<void()> > task;
		task.resize(thrPool.size());
		for(YSSIZE_T i=0; i<thrPool.size(); ++i)
		{
			auto i0=i*particle.size()/thrPool.size();
			auto i1=(i+1)*particle.size()/thrPool.size();
			task[i]=std::bind(&YsGLParticleManager::CalculateDotProd,this,i0,i1,viewPos,viewDir);
		}
		thrPool.Run(task.size(),task.data());
	}

	YsSimpleParallelMergeSort <double,YSSIZE_T> (particleDist.GetN(),particleDist,particleOrder,thrPool);
}

void YsGLParticleManager::MakeBufferForPointSprite(void)
{
	vtxBuf.CleanUp();
	texCoordBuf.CleanUp();
	colBuf.CleanUp();
	psBuf.CleanUp();

	for(auto idx : particleOrder)
	{
		auto &p=particle[idx];
		vtxBuf.Add(p.pos);
		texCoordBuf.Add(p.texCoord[0],p.texCoord[1]);
		colBuf.Add(p.col);
		psBuf.Add(p.dimension);
	}
}

void YsGLParticleManager::MakeBufferForTriangle(const YsVec3 &viewDir,const float spriteTexCoordRange)
{
	vtxBuf.CleanUp();
	texCoordBuf.CleanUp();
	colBuf.CleanUp();
	psBuf.CleanUp();

	auto xVec=YsUnitVector(viewDir.GetArbitraryPerpendicularVector());
	auto yVec=YsUnitVector(viewDir^xVec);
	auto p00Vec=(-xVec-yVec);
	auto p10Vec=( xVec-yVec);
	auto p01Vec=(-xVec+yVec);
	auto p11Vec=( xVec+yVec);

	for(auto idx : particleOrder)
	{
		auto &p=particle[idx];
		vtxBuf.Add(p.pos+p00Vec*p.dimension/2.0);
		vtxBuf.Add(p.pos+p10Vec*p.dimension/2.0);
		vtxBuf.Add(p.pos+p11Vec*p.dimension/2.0);

		vtxBuf.Add(p.pos+p00Vec*p.dimension/2.0);
		vtxBuf.Add(p.pos+p11Vec*p.dimension/2.0);
		vtxBuf.Add(p.pos+p01Vec*p.dimension/2.0);


		texCoordBuf.Add(p.texCoord[0],p.texCoord[1]);
		texCoordBuf.Add<float>(p.texCoord[0]+spriteTexCoordRange,p.texCoord[1]);
		texCoordBuf.Add<float>(p.texCoord[0]+spriteTexCoordRange,p.texCoord[1]+spriteTexCoordRange);

		texCoordBuf.Add(p.texCoord[0],p.texCoord[1]);
		texCoordBuf.Add<float>(p.texCoord[0]+spriteTexCoordRange,p.texCoord[1]+spriteTexCoordRange);
		texCoordBuf.Add<float>(p.texCoord[0],p.texCoord[1]+spriteTexCoordRange);

		colBuf.Add(p.col);
		colBuf.Add(p.col);
		colBuf.Add(p.col);
		colBuf.Add(p.col);
		colBuf.Add(p.col);
		colBuf.Add(p.col);
	}
}
