/* ////////////////////////////////////////////////////////////

File Name: main.cpp
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

#include <fslazywindow.h>
#include <ysgl.h>
#include <ysbitmap.h>
#include "opencv2/opencv.hpp"

class FsLazyWindowApplication : public FsLazyWindowApplicationBase
{
protected:
	enum 
	{
		// iOS seems to be able to deal with only one.
		MAX_NUM_VIDEO=1
	};

	bool needRedraw;
	cv::VideoCapture *capPtr[MAX_NUM_VIDEO];
	cv::Mat frame[MAX_NUM_VIDEO];
	YsBitmap rgba[MAX_NUM_VIDEO];

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
	for(auto &ptr : capPtr)
	{
		ptr=nullptr;
	}
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
	for(int i=0; i<MAX_NUM_VIDEO; ++i)
	{
		capPtr[i]=new cv::VideoCapture(i);
	}
	YsGLSLCreateSharedRenderer();
}
/* virtual */ void FsLazyWindowApplication::Interval(void)
{
    auto key=FsInkey();
	if(FSKEY_ESC==key)
	{
		SetMustTerminate(true);
	}

	for(int i=0; i<MAX_NUM_VIDEO; ++i)
	{
        printf("xyz %d\n",i);
		if(capPtr[i]->isOpened())
		{
			(*capPtr[i])>>frame[i];
			printf("Yes! [%d]\n",i);
			printf("%d x %d\n",frame[i].size[0],frame[i].size[1]);

			int bx=1,by=1;
			while(bx<frame[i].size[1])
			{
				bx*=2;
			}
			while(by<frame[i].size[0])
			{
				by*=2;
			}

			rgba[i].PrepareBitmap(bx,by);
			rgba[i].Clear(0,0,0,0);
			for(int y=0; y<frame[i].size[0]; ++y)
			{
				for(int x=0; x<frame[i].size[1]; ++x)
				{
					cv::Vec3b pix=frame[i].at<cv::Vec3b>(y,x);
                    auto pixPtr=rgba[i].GetEditableRGBAPixelPointer(x,by-1-y);
					pixPtr[0]=pix[2];
					pixPtr[1]=pix[1];
					pixPtr[2]=pix[0];
					pixPtr[3]=255;
				}
			}
		}
		else
		{
			printf("Moo! [%d]\n",i);
		}
	}

	needRedraw=true;
}
/* virtual */ void FsLazyWindowApplication::Draw(void)
{
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	int wid,hei;
	FsGetWindowSize(wid,hei);

	auto renderer=YsGLSLSharedBitmapRenderer();
	YsGLSLUseBitmapRenderer(renderer);
	for(int i=0; i<MAX_NUM_VIDEO; ++i)
	{
		int x=(wid/2)*(i%2);
		int y=(hei/2)*(i/2);

		if(0<rgba[i].GetWidth() && rgba[i].GetHeight())
		{
			YsGLSLRenderRGBABitmap2D(renderer,x,y,YSGLSL_HALIGN_LEFT,YSGLSL_VALIGN_TOP,
                                     rgba[i].GetWidth(),rgba[i].GetHeight(),rgba[i].GetRGBABitmapPointer());
		}
	}
	YsGLSLEndUseBitmapRenderer(renderer);

	FsSwapBuffers();

	needRedraw=false;
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
	for(auto &ptr : capPtr)
	{
		delete ptr;
		ptr=nullptr;
	}
	YsGLSLDeleteSharedRenderer();
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
