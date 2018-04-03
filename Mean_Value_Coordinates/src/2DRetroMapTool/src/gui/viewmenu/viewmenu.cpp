#include <ysclass.h>
#include <yscompilerwarning.h>
#include <ysgl.h>
#include <ysport.h>

#include "../fsgui3dapp.h"
#include "../textresource.h"



void FsGui3DMainCanvas::View_ShowPositionReferenceMarker(FsGuiPopUpMenuItem *menuPtr)
{
	if(nullptr!=menuPtr)
	{
		auto check=menuPtr->GetCheck();
		YsFlip(check);
		menuPtr->SetCheck(check);
	}
}
void FsGui3DMainCanvas::View_MovePositionReferenceMarker(FsGuiPopUpMenuItem *)
{
	Edit_ClearUIMode(nullptr);
	DisableDefaultAction();

	view_showPositionReferenceMarker->SetCheck(YSTRUE);

	YsVec3 cursorPos(position_reference_marker.x(),position_reference_marker.y(),0);
	threeDInterface.BeginInputPoint1(cursorPos);
	threeDInterface.SetPoint1BasePlane(0);
	threeDInterface.DisablePoint1BasePlaneButton();
	threeDInterface.BindCallBack(&THISCLASS::View_MovePositionReferenceMarker_3DInterfaceCallBack,this);
}

void FsGui3DMainCanvas::View_MovePositionReferenceMarker_3DInterfaceCallBack(FsGui3DInterface *itfc)
{
	auto pos=itfc->GetPoint();
	position_reference_marker.Set((int)pos.x(),(int)pos.y());
}

void FsGui3DMainCanvas::View_Fit(FsGuiPopUpMenuItem *)
{
	auto fieldPtr=world.GetField(GetCurrentField());
	if(nullptr!=fieldPtr)
	{
		auto bbx=fieldPtr->GetBoundingBox();
		drawEnv.SetViewTarget(bbx.GetCenter());
		drawEnv.SetTargetBoundingBox(bbx.Min(),bbx.Max());
		SetNeedRedraw(YSTRUE);
	}
}

void FsGui3DMainCanvas::View_RestoreViewWhenSwitchingField(FsGuiPopUpMenuItem *menuPtr)
{
	if(nullptr!=menuPtr)
	{
		auto check=menuPtr->GetCheck();
		YsFlip(check);
		menuPtr->SetCheck(check);
	}
}

void FsGui3DMainCanvas::View_ShowMarkUp(FsGuiPopUpMenuItem *menuPtr)
{
	if(nullptr!=menuPtr)
	{
		auto check=menuPtr->GetCheck();
		YsFlip(check);
		menuPtr->SetCheck(check);
	}
}
