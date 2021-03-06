/* ////////////////////////////////////////////////////////////

File Name: ysglbuffermanager_gl2.h
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

#ifndef YSGLBUFFERMANAGER_GL2_IS_INCLUDED
#define YSGLBUFFERMANAGER_GL2_IS_INCLUDED
/* { */

#include <ysgl.h>
#include <ysclass.h>
#include "ysglbuffermanager.h"

class YsGLBufferManager::ActualBuffer
{
public:
	enum
	{
		MAX_NUM_VERTEX_PER_VBO=99999
	};

	class BufferSet
	{
	public:
		YSSIZE_T nVtx;
		GLuint vboIdent;
		GLuint vtxPtr,nomPtr,colPtr,pointSizePtr;

		BufferSet()
		{
			vboIdent=0;
			nVtx=0;
			vtxPtr=0;
			nomPtr=0;
			colPtr=0;
			pointSizePtr=0;
		}
	};
	YsArray <BufferSet> bufSet;

	void PrepareVbo(YSSIZE_T nVbo);
	void DrawPrimitiveVtx(YsGLSL3DRenderer *renderer,GLenum primType) const;
	void DrawPrimitiveVtxNom(YsGLSL3DRenderer *renderer,GLenum primType) const;
	void DrawPrimitiveVtxCol(YsGLSL3DRenderer *renderer,GLenum primType) const;
	void DrawPrimitiveVtxNomCol(YsGLSL3DRenderer *renderer,GLenum primType) const;
	void DrawPrimitiveVtxColPointSize(YsGLSL3DRenderer *renderer,GLenum primType) const;
};

/* } */
#endif
