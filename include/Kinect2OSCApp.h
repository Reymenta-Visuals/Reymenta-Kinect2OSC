/*
*
* Copyright (c) 2014, Ban the Rewind, Wieden+Kennedy
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or
* without modification, are permitted provided that the following
* conditions are met:
*
* Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
*
* Neither the name of the Ban the Rewind nor the names of its
* contributors may be used to endorse or promote products
* derived from this software without specific prior written
* permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "cinder/app/AppNative.h"
#include "cinder/Camera.h"
#include "cinder/gl/Fbo.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Vbo.h"
#include "cinder/params/Params.h"
#include "cinder/ImageIo.h"
#include "cinder/Rand.h"
#include "cinder/Utilities.h"
#include "Resources.h"
#include "Kinect.h"
// logger
#include "Logger.h"
// parameters
#include "ParameterBag.h"
// shaders
#include "Shaders.h"
// textures
#include "Textures.h"
// OSC
#include "OSCWrapper.h"
// Utils
#include "Batchass.h"
// spout
#include "Spout.h"

#define COUNT 400

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace Reymenta;

class Kinect2OSCApp : public AppNative {
public:
	void 						draw();
	void 						prepareSettings(ci::app::AppBasic::Settings* settings);
	void						resize();
	void 						setup();
	void						update();
	void						shutdown();
	//void						keyDown(KeyEvent event);
	//void						keyUp(KeyEvent event);
	//void						fileDrop(FileDropEvent event);
	void						updateWindowTitle();

private:
	// windows
	WindowRef					mMainWindow;
	void						windowManagement();
	void						getWindowsResolution();
	bool						mIsShutDown;
	bool						mCursorVisible;

	// kinect
	MsKinect::DeviceRef			mDevice;
	MsKinect::Frame				mFrame;

	MsKinect::Face				mFace;
	MsKinect::FaceTrackerRef	mFaceTracker;

	void						onFrame(MsKinect::Frame frame);

	ci::gl::Fbo					mFbo[2];
	ci::gl::GlslProgRef			mShaderDraw;
	ci::gl::GlslProgRef			mShaderGpGpu;
	ci::gl::VboMeshRef			mMesh;
	ci::gl::TextureRef			mTextureDepth;
	ci::gl::TextureRef			mTextureColor;
	ci::gl::TextureRef			mTextureInfrared;
	// particles
	float						mParticleDampen;
	float						mParticleSpeed;
	float						mParticleTrails;
	float						mPointCloudDepth;

	ci::CameraPersp				mCamera;
	ci::Vec3f					mEyePoint;
	ci::Vec3f					mLookAt;

	ci::Vec3f					mParticleCenter;

	bool						mDrawParams;
	bool						mDrawTextures;
	bool						mFullScreen;
	bool						mFullScreenPrev;
	// Logger
	LoggerRef					log;
	// Shaders
	ShadersRef					mShaders;
	// Textures
	TexturesRef					mTextures;
	// parameters
	ParameterBagRef				mParameterBag;
	// utils
	BatchassRef					mBatchass;
	// osc
	OSCRef						mOSC;
	//fbo
	gl::Fbo						mSpoutFbo;

	// spout
	SpoutSender					spoutsender;                    // Create a Spout sender object
	bool						bInitialized;                          // true if a sender initializes OK
	bool						bMemoryMode;                           // tells us if texture share compatible
	unsigned int				g_Width, g_Height;             // size of the texture being sent out
	char						SenderName[256];                       // sender name 
	gl::Texture					spoutTexture;                   // Local Cinder texture used for sharing
	// default vertex shader
	std::string					vs;
	// default pixel shader
	std::string					fs;
	bool						setGLSLString(string pixelFrag);
};