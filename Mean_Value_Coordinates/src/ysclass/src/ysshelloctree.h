/* ////////////////////////////////////////////////////////////

File Name: ysshelloctree.h
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

#ifndef YSSHELLOCTREE_IS_INCLUDED
#define YSSHELLOCTREE_IS_INCLUDED
/* { */


// Body of YsShellOctree is written in ysshelllattice.cpp

class YsShellOctree : public YsOctree <YSSIDE>
{
public:
	YsShellOctree();

	YSSIDE CheckInside(const YsVec3 &pos) const;

	int nInsideOutsideComputed;
	int nDerivedFromNeighbor;
	const YsShell *shl;
protected:
	virtual YSRESULT TraverseFunc(YsOctreeNode <YSSIDE> *node,const YsVec3 &min,const YsVec3 &max,int param1,int param2);
	// param1 -> Command (Use only negative number for toolkit functions)
	//   -1...Building Inside/Outside Node

	YSRESULT SpreadInsideOutside(YsOctreeNode <YSSIDE> *node,int vx,int vy,int vz,YSSIDE side,int depthLimit);
	YSRESULT SpreadInsideOutsideInverse(YsOctreeNode <YSSIDE> *node,int vx,int vy,int vz,YSSIDE side,int depthLimit);
};



/* } */
#endif
