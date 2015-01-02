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

#include "Kinect2OSCApp.h"

void Kinect2OSCApp::prepareSettings(Settings* settings)
{
	// instanciate the logger class
	log = Logger::create("Kinect2OSClog.txt");
	log->logTimedString("start");
	// parameters
	mParameterBag = ParameterBag::create();

	settings->setFrameRate(60.0f);
	settings->setWindowSize(mParameterBag->mMainWindowWidth, mParameterBag->mMainWindowHeight);
	settings->setResizable(true);
	settings->setWindowPos(Vec2i(mParameterBag->mMainWindowX, mParameterBag->mMainWindowY));
}

void Kinect2OSCApp::setup()
{

	log->logTimedString("setup");
	glLineWidth(2.0f);
	mIsShutDown = false;
	mCursorVisible = true;

	// instanciate the Shaders class, must not be in prepareSettings
	mShaders = Shaders::create(mParameterBag);

	// instanciate the textures class
	mTextures = Textures::create(mParameterBag, mShaders);
	// utils
	mBatchass = Batchass::create(mParameterBag);
	mBatchass->getWindowsResolution();
	getWindow()->setTitle("Reymenta Kinect2OSC");
	// instanciate the OSC class
	mOSC = OSC::create(mParameterBag);

	try {
		mShaderDraw = gl::GlslProg::create(loadAsset("draw_vert.glsl"), loadAsset("draw_frag.glsl"));
	}
	catch (gl::GlslProgCompileExc ex) {
		console() << "Unable to load draw shader: " << ex.what() << endl;
		quit();
		return;
	}
	vs = loadString(loadAsset("gpgpu.vert"));
	fs = loadString(loadAsset("gpgpu.frag"));

	try {
		//mShaderGpGpu = gl::GlslProg::create(loadResource(RES_GLSL_GPGPU_VERT), loadResource(RES_GLSL_GPGPU_FRAG));
		setGLSLString(fs);
	}
	catch (gl::GlslProgCompileExc ex) {
		console() << "Unable to load GPGPU shader: " << ex.what() << endl;
		quit();
		return;
	}

	////////////////////////////////////////////////////////////////
	// Define properties

	mDrawParams = false;
	mDrawTextures = false;
	mEyePoint = Vec3f(0.0f, 0.0f, 3000.0f);
	mFullScreen = isFullScreen();
	mFullScreenPrev = mFullScreen;
	mLookAt = Vec3f(0.0f, -500.0f, 0.0f);

	////////////////////////////////////////////////////////////////
	// Set up Kinect

	MsKinect::DeviceOptions deviceOptions;
	deviceOptions.enableColor(true);
	deviceOptions.enableNearMode();
	deviceOptions.enableFaceTracking();
	deviceOptions.setDepthResolution(MsKinect::ImageResolution::NUI_IMAGE_RESOLUTION_640x480);
	mDevice = MsKinect::Device::create();
	mDevice->connectEventHandler(&Kinect2OSCApp::onFrame, this);
	try {
		mDevice->start(deviceOptions);
		mDevice->getFaceTracker()->enableCalcMesh(false);
		mDevice->getFaceTracker()->enableCalcMesh2d();
	}
	catch (MsKinect::Device::ExcDeviceCreate ex) {
		console() << ex.what() << endl;
	}
	catch (MsKinect::Device::ExcDeviceInit ex) {
		console() << ex.what() << endl;
	}
	catch (MsKinect::Device::ExcDeviceInvalid ex) {
		console() << ex.what() << endl;
	}
	catch (MsKinect::Device::ExcGetCoordinateMapper ex) {
		console() << ex.what() << endl;
	}
	catch (MsKinect::Device::ExcOpenStreamColor ex) {
		console() << ex.what() << endl;
	}
	catch (MsKinect::Device::ExcOpenStreamDepth ex) {
		console() << ex.what() << endl;
	}
	catch (MsKinect::Device::ExcStreamStart ex) {
		console() << ex.what() << endl;
	}
	catch (MsKinect::Device::ExcUserTrackingEnable ex) {
		console() << ex.what() << endl;
	}

	////////////////////////////////////////////////////////////////
	// Set up particles
	// particles
	mParticleDampen = 0.96f;
	mParticleSpeed = 0.05f;
	mParticleTrails = 0.92f;
	mPointCloudDepth = 3000.0f;

	glPointSize(1.0f);

	gl::Fbo::Format format;
	format.enableColorBuffer(true, 4);
	format.setColorInternalFormat(GL_RGBA32F);

	for (size_t i = 0; i < 2; ++i) {
		mFbo[i] = gl::Fbo(deviceOptions.getDepthSize().x, deviceOptions.getDepthSize().y, format);
		mFbo[i].bindFramebuffer();
		const GLenum buffers[2] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1 };
		glDrawBuffers(2, buffers);
		gl::setViewport(mFbo[i].getBounds());
		gl::setMatricesWindow(mFbo[i].getSize());
		gl::clear();
		mFbo[i].unbindFramebuffer();
	}

	vector<uint32_t> indices;
	vector<Vec3f> positions;
	vector<Vec2f> texCoords;
	int32_t h = mFbo[0].getHeight();
	int32_t w = mFbo[0].getWidth();
	for (int32_t x = 0; x < w; ++x) {
		for (int32_t y = 0; y < h; ++y) {
			indices.push_back((uint32_t)(x * h + y));
			texCoords.push_back(Vec2f((float)x / (float)(w - 1), (float)y / (float)(h - 1)));
			positions.push_back(Vec3f(
				(texCoords.rbegin()->x * 2.0f - 1.0f) * (float)h,
				(texCoords.rbegin()->y * 2.0f - 1.0f) * (float)w,
				0.0f));
		}
	}
	gl::VboMesh::Layout layout;
	layout.setStaticIndices();
	layout.setStaticPositions();
	layout.setStaticTexCoords2d();
	layout.setDynamicColorsRGB();

	mMesh = gl::VboMesh::create(positions.size(), indices.size(), layout, GL_POINTS);
	mMesh->bufferIndices(indices);
	mMesh->bufferPositions(positions);
	mMesh->bufferTexCoords2d(0, texCoords);
	mMesh->unbindBuffers();

	// spout
	g_Width = 640;
	g_Height = 480;
	// Set up the texture we will use to send out
	// We grab the screen so it has to be the same size
	spoutTexture = gl::Texture(g_Width, g_Height);
	strcpy_s(SenderName, "Reymenta Kinect Sender"); // we have to set a sender name first

	// Initialize a sender
	bInitialized = spoutsender.CreateSender(SenderName, g_Width, g_Height);

	/*gl::enableDepthRead();
	gl::enableDepthWrite();*/

	mSpoutFbo = gl::Fbo(g_Width, g_Height);

	mParameterBag->mSendToOutput = true;
	resize();
}
void Kinect2OSCApp::resize()
{
	glPointSize(0.25f);
	gl::enable(GL_POINT_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

	mCamera = CameraPersp(getWindowWidth(), getWindowHeight(), 60.0f, 1.0f, 50000.0f);
	mCamera.setWorldUp(-Vec3f::yAxis());
}
bool Kinect2OSCApp::setGLSLString(string pixelFrag)
{
	try
	{
		mShaderGpGpu = gl::GlslProg::create(vs.c_str(), pixelFrag.c_str());
	}
	catch (gl::GlslProgCompileExc &exc)
	{
		log->logTimedString("setGLSLString error: " + string(exc.what()));
	}
	return true;
}
void Kinect2OSCApp::update()
{
	mParameterBag->iGlobalTime = getElapsedSeconds();

	mParameterBag->iFps = getAverageFps();
	updateWindowTitle();

	//mFrameRate = getAverageFps();
	mCamera.lookAt(mEyePoint, mLookAt);

	if (mFullScreenPrev != mFullScreen) {
		setFullScreen(mFullScreen);
		mFullScreenPrev = mFullScreen;
	}
	float time = (float)getElapsedSeconds();
	const float deltaAng = 1.5f * M_PI / 100;
	gl::VboMesh::VertexIter iter = mMesh->mapVertexBuffer();

	for (int i = 0; i < COUNT; ++i) {
		iter.setPosition(Vec3f(math<float>::cos(time + i * deltaAng), math<float>::sin(time + i * deltaAng), .1f + 149.9f * i / COUNT));
		//use this line to see closest first 
		//iter.setPosition(Vec3f(math<float>::cos(time + i *		deltaAng), math<float>::sin(time + i * deltaAng), 150.f -			149.9f * i / COUNT));
		iter.setColorRGB(Color(CM_HSV, Vec3f(fract(time + i *	deltaAng), 1.f, 1.f)));
		++iter;
	}

	mTextures->update();
	mOSC->update();
}
void Kinect2OSCApp::updateWindowTitle()
{
	getWindow()->setTitle("(" + toString(floor(getAverageFps())) + " fps) Reymenta Kinect2OSC");
}
void Kinect2OSCApp::onFrame(MsKinect::Frame frame)
{
	mFrame = frame;
	mFace = frame.getFace();

	if (frame.getDepthChannel())
	{
		Surface16u depth = MsKinect::depthChannelToSurface(frame.getDepthChannel(), MsKinect::DepthProcessOptions().enableRemoveBackground());
		mTextureDepth = gl::Texture::create(depth);
		//mTextures->setKinectTexture(3, gl::Texture(depth));
	}
	if (frame.getColorSurface())
	{
		mTextureColor = gl::Texture::create(frame.getColorSurface());
		//mTextures->setKinectTexture(2, frame.getColorSurface());
	}
	//else 
	/*if (frame.getInfraredChannel())
	{
	Surface16u infraredDepth = MsKinect::
	mTextureInfrared = gl::Texture::create(frame.getInfraredChannel());
	mTextures->setKinectTexture(1, frame.getInfraredChannel());
	}*/

}
void Kinect2OSCApp::draw()
{
	gl::clear();
	// draw the fbos
	mTextures->draw();
	unsigned int width, height;

	// -------- SPOUT -------------
	if (bInitialized && mParameterBag->mSendToOutput) {
		width = g_Width;
		height = g_Height;

		if (width != mParameterBag->mOutputResolution.x || height != mParameterBag->mOutputResolution.y) {
			// The sender dimensions have changed, update the global width and height
			g_Width = mParameterBag->mOutputResolution.x;
			g_Height = mParameterBag->mOutputResolution.y;
			// Update the local texture to receive the new dimensions
			mSpoutFbo = gl::Fbo(g_Width, g_Height);

			return; // quit for next round
		}
	}

	if (mFrame.getDepthChannel())
	{
		gl::pushMatrices();

		int skeletonIndex = 0;
		int jointIndex = 0;

		for (const auto& skeleton : mFrame.getSkeletons()) {
			for (const auto& joint : skeleton) {
				const MsKinect::Bone& bone = joint.second;

				Vec2i v0 = mDevice->mapSkeletonCoordToDepth(bone.getPosition());
				Vec2i v1 = mDevice->mapSkeletonCoordToDepth(skeleton.at(bone.getStartJoint()).getPosition());
				//OK mOSC->sendOSCIntMessage("/joint", skeletonIndex, jointIndex, v0.x, v0.y, v1.x, v1.y);
				gl::drawLine(v0, v1);
				gl::drawSolidCircle(v0, 5.0f, 16);
				jointIndex++;
			}
			skeletonIndex++;
		}
		//OK mOSC->sendOSCIntMessage("/jointcount", jointIndex / 20, 0, 0, 0, 0, 0);

		if (mFace.getMesh2d().getNumVertices() > 0) {
			gl::pushMatrices();
			gl::scale(0.5f, 0.5f);
			gl::color(ColorAf::white());
			gl::enableWireframe();
			gl::draw(mFace.getMesh2d());
			gl::disableWireframe();
			gl::popMatrices();
		}

		gl::popMatrices();
	}
	if (mTextureDepth)
	{
		gl::enable(GL_TEXTURE_2D);

		////////////////////////////////////////////////////////////////
		// GPGPU

		int32_t ping = getElapsedFrames() % 2;
		int32_t pong = (ping + 1) % 2;
		{
			mFbo[pong].bindFramebuffer();
			const GLenum buffers[2] = {
				GL_COLOR_ATTACHMENT0,
				GL_COLOR_ATTACHMENT1
			};
			glDrawBuffers(2, buffers);
			gl::setViewport(mFbo[pong].getBounds());
			gl::setMatricesWindow(mFbo[pong].getSize(), false);
			for (int32_t i = 0; i < 2; ++i) {
				mFbo[ping].bindTexture(i, i);
			}
			mTextureDepth->bind(2);

			mShaderGpGpu->bind();
			mShaderGpGpu->uniform("uCenter", mParticleCenter);
			mShaderGpGpu->uniform("uDampen", mParticleDampen);
			mShaderGpGpu->uniform("uSpeed", mParticleSpeed);
			mShaderGpGpu->uniform("iInvert", (int)mParameterBag->controlValues[48]);
			mShaderGpGpu->uniform("uTextureKinect", 2);
			mShaderGpGpu->uniform("uTexturePosition", 0);
			mShaderGpGpu->uniform("uTextureVelocity", 1);

			gl::drawSolidRect(mFbo[pong].getBounds(), false);

			mShaderGpGpu->unbind();
			mTextureDepth->unbind();
			mFbo[ping].unbindTexture();
			mFbo[pong].unbindFramebuffer();
			//mTextures->setKinectTexture(0, mFbo[pong].getTexture());
			//mTextures->setKinectTexture(1, *mTextureDepth);
		}

		////////////////////////////////////////////////////////////////
		// Draw particles

		gl::setViewport(getWindowBounds());
		if (mParameterBag->mPreviewEnabled)
		{
			if (bInitialized && mParameterBag->mSendToOutput)
			{
				mSpoutFbo.bindFramebuffer();

				gl::setViewport(mSpoutFbo.getBounds());

				// clear the FBO
				gl::clear();

				gl::setMatricesWindow(mParameterBag->mOutputResolution.x, mParameterBag->mOutputResolution.y, mParameterBag->mOriginUpperLeft);
			}
			gl::setMatricesWindow(getWindowSize());
			gl::enableAlphaBlending();
			gl::disable(GL_TEXTURE_2D);
			gl::color(ColorAf(Colorf::black(), 1.0f - mParticleTrails));
			gl::drawSolidRect(getWindowBounds());
			gl::disableAlphaBlending();

			gl::setMatrices(mCamera);
			gl::enableAdditiveBlending();
			gl::enable(GL_TEXTURE_2D);
			gl::color(ColorAf::white());

			mFbo[pong].bindTexture();

			mShaderDraw->bind();
			mShaderDraw->uniform("uDepth", mPointCloudDepth);
			mShaderDraw->uniform("uPositions", 0);
			mShaderDraw->uniform("uTime", (float)getElapsedSeconds());

			gl::draw(mMesh);

			mShaderDraw->unbind();
			mFbo[pong].unbindTexture();
			//mTextures->setKinectTexture(4, mFbo[pong].getTexture());
			gl::disableAlphaBlending();
			if (bInitialized && mParameterBag->mSendToOutput)
			{
				// stop drawing into the FBO
				mSpoutFbo.unbindFramebuffer();
				spoutsender.SendTexture(mSpoutFbo.getTexture().getId(), mSpoutFbo.getTexture().getTarget(), mSpoutFbo.getWidth(), mSpoutFbo.getHeight(), false);
			}
		}
		else
		{
			if (bInitialized && mParameterBag->mSendToOutput)
			{

				spoutsender.SendTexture(mTextures->getTexture(mParameterBag->currentSelectedIndex).getId(), mTextures->getTexture(mParameterBag->currentSelectedIndex).getTarget(), g_Width, g_Height, false);
			}
		}
	}
	gl::disableAlphaBlending();
}
void Kinect2OSCApp::shutdown()
{
	if (!mIsShutDown)
	{
		mIsShutDown = true;
		log->logTimedString("shutdown");
		spoutsender.ReleaseSender();
		quit();
	}
}
CINDER_APP_NATIVE( Kinect2OSCApp, RendererGl )
