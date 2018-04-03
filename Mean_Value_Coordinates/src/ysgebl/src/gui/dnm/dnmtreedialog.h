/* ////////////////////////////////////////////////////////////

File Name: dnmtreedialog.h
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

#ifndef DNMTREEDIALOG_IS_INCLUDED
#define DNMTREEDIALOG_IS_INCLUDED
/* { */

#include <fsgui.h>
#include <ysshellextedit.h>
#include <ysshelldnmtemplate.h>

class PolyCreDnmTreeDialog : public FsGuiDialog
{
private:
	class GeblGuiEditorBase *canvas;
	FsGuiTreeControl *dnmTree;
	FsGuiButton *okBtn,*cancelBtn;
	YSBOOL treeEdited;
public:
	YSBOOL runningModal;

	// YsGLVertexBuffer lineBuf,triBuf; as required

public:
	PolyCreDnmTreeDialog();
	~PolyCreDnmTreeDialog();
	static PolyCreDnmTreeDialog *Create(void);
	static void Delete(PolyCreDnmTreeDialog *ptr);

	YSRESULT Make(class GeblGuiEditorBase *canvas);
	void EnableTreeEdit(void);  // If enable, call this function after creating the dialog.
private:
	void PopulateTreeControl(const FsGuiTreeControlNode *parent,YsShellDnmContainer <YsShellExtEditGui>::Node *dnmNode);

public:
	void RemakeDrawingBuffer(void);

	virtual void OnButtonClick(FsGuiButton *btn);
	virtual void OnDropListSelChange(FsGuiDropList *drp,int);
	virtual void OnTextBoxChange(FsGuiTextBox *txt);
	virtual void OnSliderPositionChange(FsGuiSlider *slider,const double &prevPos,const double &prevValue);
	virtual void OnTreeControlSelChange(FsGuiTreeControl *tree);
	virtual void OnTreeControlNodeMoved(FsGuiTreeControl *tree,int nNode,const FsGuiTreeControlNode * const node[]);
};

/* } */
#endif
