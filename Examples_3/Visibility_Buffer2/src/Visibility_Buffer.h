/*
 * Copyright (c) 2017-2024 The Forge Interactive Inc.
 *
 * This file is part of The-Forge
 * (see https://github.com/ConfettiFX/The-Forge).
 *
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include "../../../Common_3/Application/Interfaces/IApp.h"
#include "../../../Common_3/Application/Interfaces/ICameraController.h"
#include "../../../Common_3/Application/Interfaces/IFont.h"
#include "../../../Common_3/OS/Interfaces/IInput.h"
#include "../../../Common_3/Application/Interfaces/IProfiler.h"
#include "../../../Common_3/Application/Interfaces/IScreenshot.h"
#include "../../../Common_3/Application/Interfaces/IUI.h"
#include "../../../Common_3/Game/Interfaces/IScripting.h"
#include "../../../Common_3/Graphics/Interfaces/IGraphics.h"
#include "../../../Common_3/Renderer/Interfaces/IVisibilityBuffer2.h"
#include "../../../Common_3/Utilities/Interfaces/IFileSystem.h"
#include "../../../Common_3/Utilities/Interfaces/ILog.h"
#include "../../../Common_3/Utilities/Interfaces/IThread.h"
#include "../../../Common_3/Utilities/Interfaces/ITime.h"

#include "../../../Common_3/Utilities/RingBuffer.h"
#include "../../../Common_3/Utilities/Threading/ThreadSystem.h"

#include "SanMiguel.h"
#define NO_FSL_DEFINITIONS
#include "Shaders/FSL/shader_defs.h.fsl"
#include "Shaders/FSL/triangle_binning.h.fsl"

#if defined(XBOX)
#include "../../../Xbox/Common_3/Graphics/Direct3D12/Direct3D12X.h"
#include "../../../Xbox/Common_3/Graphics/IESRAMManager.h"
#define BEGINALLOCATION(X, O) esramBeginAllocation(pRenderer->mDx.pESRAMManager, X, O)
#define ALLOCATIONOFFSET()    esramGetCurrentOffset(pRenderer->mDx.pESRAMManager)
#define ENDALLOCATION(X)      esramEndAllocation(pRenderer->mDx.pESRAMManager)
#else
#define BEGINALLOCATION(X, O)
#define ALLOCATIONOFFSET() 0u
#define ENDALLOCATION(X)
#endif

#include "../../../Common_3/Utilities/Interfaces/IMemory.h"

#define FOREACH_SETTING(X)  \
    X(BindlessSupported, 1) \
    X(DisableAO, 0)         \
    X(DisableGodRays, 0)    \
    X(AddGeometryPassThrough, 0)

#define GENERATE_ENUM(x, y)   x,
#define GENERATE_STRING(x, y) #x,
#define GENERATE_STRUCT(x, y) uint32_t m##x = y;
#define GENERATE_VALUE(x, y)  y,
#define INIT_STRUCT(s)        s = { FOREACH_SETTING(GENERATE_VALUE) }

typedef enum ESettings
{
    FOREACH_SETTING(GENERATE_ENUM) Count
} ESettings;

const char* gSettingNames[] = { FOREACH_SETTING(GENERATE_STRING) };

// Useful for using names directly instead of subscripting an array
struct ConfigSettings
{
    FOREACH_SETTING(GENERATE_STRUCT)
} gGpuSettings;

static ThreadSystem gThreadSystem;

#define SCENE_SCALE 50.0f

typedef enum OutputMode
{
    OUTPUT_MODE_SDR = 0,
    OUTPUT_MODE_P2020,
    OUTPUT_MODE_COUNT
} OutputMode;

struct SCurveInfo
{
    float C1;
    float C2;
    float C3;
    float UseSCurve;

    float ScurveSlope;
    float ScurveScale;
    float linearScale;
    float pad0;

    uint outputMode;
};

SCurveInfo gSCurveInfomation;

struct GodRayConstant
{
    float mScatterFactor;
};

GodRayConstant gGodRayConstant{ 0.5f };

struct GodRayBlurConstant
{
    uint32_t mBlurPassType; // Horizontal or Vertical pass
    uint32_t mFilterRadius;
};

#define MAX_BLUR_KERNEL_SIZE 8

struct BlurWeights
{
    float mBlurWeights[MAX_BLUR_KERNEL_SIZE];
};

GodRayBlurConstant gGodRayBlurConstant;
BlurWeights        gBlurWeightsUniform;
float              gGaussianBlurSigma[2] = { 1.0f, 1.0f };

static float gaussian(float x, float sigma)
{
    x = abs(x) / sigma;
    x *= x;
    return exp(-0.5f * x);
}

typedef enum CurveConversionMode
{
    CurveConversion_LinearScale = 0,
    CurveConversion_SCurve = 1
} CurveConversionMode;

enum DisplayColorRange
{
    ColorRange_RGB = 0,
    ColorRange_YCbCr422 = 1,
    ColorRange_YCbCr444 = 2
};

enum DisplaySignalRange
{
    Display_SIGNAL_RANGE_FULL = 0,
    Display_SIGNAL_RANGE_NARROW = 1
};

enum DisplayColorSpace
{
    ColorSpace_Rec709 = 0,
    ColorSpace_Rec2020 = 1,
    ColorSpace_P3D65 = 2
};

struct DisplayChromacities
{
    float RedX;
    float RedY;
    float GreenX;
    float GreenY;
    float BlueX;
    float BlueY;
    float WhiteX;
    float WhiteY;
};

enum BlurPassType
{
    BLUR_PASS_TYPE_HORIZONTAL,
    BLUR_PASS_TYPE_VERTICAL,
    BLUR_PASS_TYPE_COUNT
};

// Camera Walking
static float gCameraWalkingTime = 0.0f;
float3*      gCameraPathData;

uint  gCameraPoints;
float gTotalElpasedTime;
/************************************************************************/
// GUI CONTROLS
/************************************************************************/
#if defined(ANDROID)
#define DEFAULT_ASYNC_COMPUTE false
#else
#define DEFAULT_ASYNC_COMPUTE true
#endif

typedef struct AppSettings
{
    OutputMode mOutputMode = OUTPUT_MODE_SDR;

    bool mSmallScaleRaster = true;

    bool mAsyncCompute = DEFAULT_ASYNC_COMPUTE;
    // toggle rendering of local point lights
    bool mRenderLocalLights = false;

    bool mDrawDebugTargets = false;

    bool mLargeBinRasterGroups = true;

    float nearPlane = 10.0f;
    float farPlane = 3000.0f;

    // adjust directional sunlight angle
    float2 mSunControl = { -2.1f, 0.164f };

    float mSunSize = 300.0f;

    float4 mLightColor = { 1.0f, 0.8627f, 0.78f, 2.5f };

    DynamicUIWidgets mDynamicUIWidgetsGR;
    bool             mEnableGodray = true;
    uint32_t         mFilterRadius = 3;

    float mEsmControl = 200.0f;

    bool mVisualizeBinOccupancy = false;

    // AO data
    DynamicUIWidgets mDynamicUIWidgetsAO;
    bool             mEnableAO = true;
    bool             mVisualizeAO = false;
    float            mAOIntensity = 3.0f;
    int              mAOQuality = 2;

    DynamicUIWidgets mSCurve;
    float            SCurveScaleFactor = 2.6f;
    float            SCurveSMin = 0.00f;
    float            SCurveSMid = 12.0f;
    float            SCurveSMax = 99.0f;
    float            SCurveTMin = 0.0f;
    float            SCurveTMid = 11.0f;
    float            SCurveTMax = 400.0f;
    float            SCurveSlopeFactor = 1.475f;

    DynamicUIWidgets mLinearScale;
    float            LinearScale = 260.0f;

    // HDR10
    DynamicUIWidgets mDisplaySetting;

    DisplayColorSpace  mCurrentSwapChainColorSpace = ColorSpace_Rec2020;
    DisplayColorRange  mDisplayColorRange = ColorRange_RGB;
    DisplaySignalRange mDisplaySignalRange = Display_SIGNAL_RANGE_FULL;

    CurveConversionMode mCurveConversionMode = CurveConversion_LinearScale;

    // Camera Walking
    bool  cameraWalking = false;
    float cameraWalkingSpeed = 1.0f;

} AppSettings;

/************************************************************************/
// Constants
/************************************************************************/

// #NOTE: Two sets of resources (one in flight and one being used on CPU)
const uint32_t gDataBufferCount = 2;

// Constants
const uint32_t gShadowMapSize = 1024;

struct UniformDataSkybox
{
    mat4 mProjectView;
    vec3 mCamPos;
};

int gGodrayScale = 2;

// Define different geometry sets (opaque and alpha tested geometry)
const uint32_t gNumGeomSets = NUM_GEOMETRY_SETS;

/************************************************************************/
// Per frame staging data
/************************************************************************/
struct PerFrameData
{
    // Stores the camera/eye position in object space for cluster culling
    vec3                    gEyeObjectSpace[NUM_CULLING_VIEWPORTS] = {};
    PerFrameConstantsData   gPerFrameUniformData = {};
    PerFrameVBConstantsData gPerFrameVBUniformData = {};
    UniformDataSkybox       gUniformDataSky;

    // These are just used for statistical information
    uint32_t gTotalClusters = 0;
    uint32_t gCulledClusters = 0;
    uint32_t gDrawCount[gNumGeomSets] = {};
    uint32_t gTotalDrawCount = {};
};

// Push constant data we provide to rasterizer compute pipeline.
// View is either VIEW_SHADOW or VIEW_CAMERA
// Width and height are the dimensions of the render target
struct RasterizerPushConstantData
{
    uint32_t view;
    int      width;
    int      height;
};

uint32_t gIndexCount = 0;

/************************************************************************/
// Scene
/************************************************************************/
Scene*            gScene = NULL;
/************************************************************************/
// Settings
/************************************************************************/
AppSettings       gAppSettings;
/************************************************************************/
// Profiling
/************************************************************************/
ProfileToken      gGraphicsProfileToken;
ProfileToken      gComputeProfileToken;
/************************************************************************/
// Rendering data
/************************************************************************/
Renderer*         pRenderer = NULL;
VisibilityBuffer* pVisibilityBuffer = NULL;
/************************************************************************/
// Queues and Command buffers
/************************************************************************/
Queue*            pGraphicsQueue = NULL;
GpuCmdRing        gGraphicsCmdRing = {};

Queue*         pComputeQueue = NULL;
GpuCmdRing     gComputeCmdRing = {};
/************************************************************************/
// Swapchain
/************************************************************************/
SwapChain*     pSwapChain = NULL;
Semaphore*     pImageAcquiredSemaphore = NULL;
Semaphore*     pPresentSemaphore = NULL;
/************************************************************************/
// Clear buffers pipeline
/************************************************************************/
Shader*        pShaderClearBuffers = nullptr;
Pipeline*      pPipelineClearBuffers = nullptr;
RootSignature* pRootSignatureClearBuffers = NULL;
DescriptorSet* pDescriptorSetClearBuffers = NULL;
/************************************************************************/
// Clear VisibilityBuffer pipeline
/************************************************************************/
RootSignature* pRootSignatureClearRenderTarget = NULL;
Shader*        pShaderClearRenderTarget = NULL;
Pipeline*      pPipelineClearRenderTarget = NULL;
DescriptorSet* pDescriptorSetClearDepthTarget = NULL;
uint32_t       gClearRenderTargetRootConstantIndex;
/************************************************************************/
// Triangle filtering pipeline
/************************************************************************/
Shader*        pShaderTriangleFiltering = nullptr;
Pipeline*      pPipelineTriangleFiltering = nullptr;
RootSignature* pRootSignatureTriangleFiltering = nullptr;
DescriptorSet* pDescriptorSetTriangleFiltering[2] = { NULL };
/************************************************************************/
// Clear light clusters pipeline
/************************************************************************/
Shader*        pShaderClearLightClusters = nullptr;
Pipeline*      pPipelineClearLightClusters = nullptr;
RootSignature* pRootSignatureLightClusters = nullptr;
DescriptorSet* pDescriptorSetLightClusters[2] = { NULL };
/************************************************************************/
// Compute light clusters pipeline
/************************************************************************/
Shader*        pShaderClusterLights = nullptr;
Pipeline*      pPipelineClusterLights = nullptr;
/************************************************************************/
// Shadow pass pipeline
/************************************************************************/
DescriptorSet* pDescriptorSetShadowRasterizationPass[2] = { NULL };
/************************************************************************/
// GPU Driven Visibility Buffer Filling
/************************************************************************/
// Below are the GPU resources created for compute based visibility buffer filling.
// Unfortunately, to be able to stay cross platform, we need Root Signatures per each compute shader.
// This is due to Prospero requiring exact same bindings in each shader that shares the same root signature.
// Shaders below do not share the exact bindings with each other.
/************************************************************************/
// GPU Driven Visibility Buffer Filling - Depth/Shadow
/************************************************************************/
// Small triangles
RootSignature* pRootSignatureVisibilityBufferDepthRaster = nullptr;
uint32_t       gVisibilityBufferDepthRasterRootConstantIndex = 0;
Shader*        pShaderVisibilityBufferDepthRaster = nullptr;
Pipeline*      pPipelineVisibilityBufferDepthRaster = nullptr;
DescriptorSet* pDescriptorSetVBDepthRaster[2] = { NULL };
/************************************************************************/
// VB Blit Depth
/************************************************************************/
RootSignature* pRootSignatureBlitDepth = nullptr;
Shader*        pShaderBlitDepth = nullptr;
Pipeline*      pPipelineBlitDepth = nullptr;
DescriptorSet* pDescriptorSetBlitDepth = nullptr;
uint32_t       gBlitDepthRootConstantIndex = 0;
/************************************************************************/
// VB shade pipeline
/************************************************************************/
Shader*        pShaderVisibilityBufferShade[2] = { nullptr };
Pipeline*      pPipelineVisibilityBufferShadeSrgb[2] = { nullptr };
RootSignature* pRootSignatureVBShade = nullptr;
DescriptorSet* pDescriptorSetVBShade[2] = { NULL };
/************************************************************************/
// Skybox pipeline
/************************************************************************/
Shader*        pShaderSkybox = nullptr;
Pipeline*      pSkyboxPipeline = nullptr;
RootSignature* pRootSingatureSkybox = nullptr;

Buffer*        pSkyboxVertexBuffer = NULL;
Texture*       pSkybox = NULL;
DescriptorSet* pDescriptorSetSkybox[2] = { NULL };
/************************************************************************/
// Godray pipeline
/************************************************************************/
Shader*        pGodRayPass = nullptr;
Pipeline*      pPipelineGodRayPass = nullptr;
RootSignature* pRootSigGodRayPass = nullptr;
DescriptorSet* pDescriptorSetGodRayPass = NULL;
DescriptorSet* pDescriptorSetGodRayPassPerFrame = NULL;
Buffer*        pBufferGodRayConstant = nullptr;
uint32_t       gGodRayConstantIndex = 0;

Shader*        pShaderGodRayBlurPass = nullptr;
Pipeline*      pPipelineGodRayBlurPass = nullptr;
RootSignature* pRootSignatureGodRayBlurPass = nullptr;
DescriptorSet* pDescriptorSetGodRayBlurPass = nullptr;
Buffer*        pBufferBlurWeights = nullptr;
Buffer*        pBufferGodRayBlurConstant = nullptr;
uint32_t       gGodRayBlurConstantIndex = 0;
/************************************************************************/
// Curve Conversion pipeline
/************************************************************************/
Shader*        pShaderCurveConversion = nullptr;
Pipeline*      pPipelineCurveConversionPass = nullptr;
RootSignature* pRootSigCurveConversionPass = nullptr;
DescriptorSet* pDescriptorSetCurveConversionPass = NULL;

OutputMode         gWasOutputMode = gAppSettings.mOutputMode;
DisplayColorSpace  gWasColorSpace = gAppSettings.mCurrentSwapChainColorSpace;
DisplayColorRange  gWasDisplayColorRange = gAppSettings.mDisplayColorRange;
DisplaySignalRange gWasDisplaySignalRange = gAppSettings.mDisplaySignalRange;

/************************************************************************/
// Present pipeline
/************************************************************************/
Shader*        pShaderPresentPass = nullptr;
Pipeline*      pPipelinePresentPass = nullptr;
RootSignature* pRootSigPresentPass = nullptr;
DescriptorSet* pDescriptorSetPresentPass = { NULL };
uint32_t       gSCurveRootConstantIndex = 0;
/************************************************************************/
// Render targets
/************************************************************************/
RenderTarget*  pDepthBuffer = NULL;
Buffer*        pVBDepthBuffer[2] = { NULL };
RenderTarget*  pRenderTargetVBPass = NULL;
RenderTarget*  pRenderTargetShadow = NULL;
RenderTarget*  pIntermediateRenderTarget = NULL;
RenderTarget*  pRenderTargetGodRay[2] = { NULL };
RenderTarget*  pCurveConversionRenderTarget = NULL;
/************************************************************************/
// Samplers
/************************************************************************/
Sampler*       pSamplerTrilinearAniso = NULL;
Sampler*       pSamplerBilinear = NULL;
Sampler*       pSamplerPointClamp = NULL;
Sampler*       pSamplerBilinearClamp = NULL;
/************************************************************************/
// Bindless texture array
/************************************************************************/
Texture**      gDiffuseMapsStorage = NULL;
Texture**      gNormalMapsStorage = NULL;
Texture**      gSpecularMapsStorage = NULL;
/************************************************************************/
// Vertex buffers for the scene
/************************************************************************/
Geometry*      pGeom = NULL;
/************************************************************************/
// Indirect buffers
/************************************************************************/
Buffer*        pMaterialPropertyBuffer = NULL;
Buffer*        pPerFrameUniformBuffers[gDataBufferCount] = { NULL };
enum
{
    VB_UB_COMPUTE = 0,
    VB_UB_GRAPHICS,
    VB_UB_COUNT
};
Buffer*  pPerFrameVBUniformBuffers[VB_UB_COUNT][gDataBufferCount] = {};
// Buffers containing all indirect draw commands per geometry set (no culling)
uint32_t gDrawCountAll[gNumGeomSets] = {};
Buffer*  pMeshConstantsBuffer = NULL;

/************************************************************************/
// Other buffers for lighting, point lights,...
/************************************************************************/
Buffer*          pLightsBuffer = NULL;
Buffer**         gPerBatchUniformBuffers = NULL;
Buffer*          pVertexBufferCube = NULL;
Buffer*          pIndexBufferCube = NULL;
Buffer*          pLightClustersCount[gDataBufferCount] = { NULL };
Buffer*          pLightClusters[gDataBufferCount] = { NULL };
Buffer*          pUniformBufferSky[gDataBufferCount] = { NULL };
uint64_t         gFrameCount = 0;
FilterContainer* pFilterContainers = NULL;
uint32_t         gMeshCount = 0;
uint32_t         gMaterialCount = 0;
UIComponent*     pGuiWindow = NULL;
UIComponent*     pDebugTexturesWindow = NULL;
UIWidget*        pOutputSupportsHDRWidget = NULL;
FontDrawDesc     gFrameTimeDraw;
uint32_t         gFontID = 0;

/************************************************************************/
ICameraController* pCameraController = NULL;
/************************************************************************/
// CPU staging data
/************************************************************************/
// CPU buffers for light data
LightData          gLightData[LIGHT_COUNT] = {};

PerFrameData  gPerFrame[gDataBufferCount] = {};
RenderTarget* pScreenRenderTarget = NULL;
/************************************************************************/
// Screen resolution UI data
/************************************************************************/
#if defined(_WINDOWS)
struct ResolutionData
{
    // Buffer for all res name strings
    char*        mResNameContainer;
    // Array of const char*
    const char** mResNamePointers;
};

static ResolutionData gGuiResolution = { NULL, NULL };
#endif

const char*    pPipelineCacheName = "PipelineCache.cache";
PipelineCache* pPipelineCache = NULL;

/************************************************************************/
// App implementation
/************************************************************************/
void SetupDebugTexturesWindow()
{
    float  scale = 0.15f;
    float2 screenSize = { (float)pRenderTargetVBPass->mWidth, (float)pRenderTargetVBPass->mHeight };
    float2 texSize = screenSize * scale;

    if (!pDebugTexturesWindow)
    {
        UIComponentDesc UIComponentDesc = {};
        UIComponentDesc.mStartSize = vec2(UIComponentDesc.mStartSize.getX(), UIComponentDesc.mStartSize.getY());
        UIComponentDesc.mStartPosition.setY(screenSize.getY() - texSize.getY() - 50.f);
        uiAddComponent("DEBUG RTs", &UIComponentDesc, &pDebugTexturesWindow);

        DebugTexturesWidget widget;
        luaRegisterWidget(uiAddComponentWidget(pDebugTexturesWindow, "Debug RTs", &widget, WIDGET_TYPE_DEBUG_TEXTURES));
    }

    static const Texture* pVBRTs[6];
    uint32_t              textureCount = 0;

    pVBRTs[textureCount++] = pRenderTargetVBPass->pTexture;
    pVBRTs[textureCount++] = pRenderTargetShadow->pTexture;
    pVBRTs[textureCount++] = pDepthBuffer->pTexture;

    if (pDebugTexturesWindow)
    {
        ((DebugTexturesWidget*)pDebugTexturesWindow->mWidgets[0]->pWidget)->pTextures = pVBRTs;
        ((DebugTexturesWidget*)pDebugTexturesWindow->mWidgets[0]->pWidget)->mTexturesCount = textureCount;
        ((DebugTexturesWidget*)pDebugTexturesWindow->mWidgets[0]->pWidget)->mTextureDisplaySize = texSize;
    }
}

class Visibility_Buffer: public IApp
{
public:
    Visibility_Buffer() { SetContentScaleFactor(2.0f); }
    bool Init(); 
    void Exit();

    // Setup the render targets used in this demo.
    // The only render target that is being currently used stores the results of the Visibility Buffer pass.
    // As described earlier, this render target uses 32 bit per pixel to store draw / triangle IDs that will be
    // loaded later by the shade step to reconstruct interpolated triangle data per pixel.
    bool Load(ReloadDesc* pReloadDesc);

    void Unload(ReloadDesc* pReloadDesc);

    void Update(float deltaTime);

    void Draw();

    const char* GetName() { return "Visibility_Buffer2"; }

    bool addDescriptorSets();

    void removeDescriptorSets();

    void prepareDescriptorSets();
    /************************************************************************/
    // Add render targets
    /************************************************************************/
    bool addSwapChain();
    void addRenderTargets();
    void removeRenderTargets();
    /************************************************************************/
    // Load all the shaders needed for the demo
    /************************************************************************/
    void addRootSignatures();
    void removeRootSignatures();
    void addShaders();
    void removeShaders();
    void addPipelines();
    void removePipelines();

    // This method sets the contents of the buffers to indicate the rendering pass that
    // the whole scene triangles must be rendered (no cluster / triangle filtering).
    // This is useful for testing purposes to compare visual / performance results.
    void addTriangleFilteringBuffers(Scene* pScene);
    void removeTriangleFilteringBuffers();

    /************************************************************************/
    // Scene update
    /************************************************************************/
    // Updates uniform data for the given frame index.
    // This includes transform matrices, render target resolution and global information about the scene.
    void updateUniformData(uint currentFrameIdx);

    /************************************************************************/
    // UI
    /************************************************************************/
    void updateDynamicUIElements();

    /************************************************************************/
    // Rendering
    /************************************************************************/
    void clearVisibilityBuffer(Cmd* cmd, ProfileToken profileToken, uint32_t frameIdx);

    // Render the shadow mapping pass. This pass updates the shadow map texture.
    void drawShadowMapPass(Cmd* cmd, ProfileToken pGpuProfiler, uint32_t frameIdx);

    // Render the scene to perform the Visibility Buffer pass. In this pass the (filtered) scene geometry is rendered
    // into a 32-bit per pixel render target. This contains triangle information (batch Id and triangle Id) that allows
    // to reconstruct all triangle attributes per pixel. This is faster than a typical Deferred Shading pass, because
    // less memory bandwidth is used.
    void drawVisibilityBufferPass(Cmd* cmd, ProfileToken pGpuProfiler, uint32_t frameIdx);

    void blitVisibilityBufferDepthPass(Cmd* cmd, ProfileToken pGpuProfiler, uint32_t view, uint32_t frameIdx, RenderTarget* depthTarget);

    // Render a fullscreen triangle to evaluate shading for every pixel. This render step uses the render target generated by
    // DrawVisibilityBufferPass to get the draw / triangle IDs to reconstruct and interpolate vertex attributes per pixel. This method
    // doesn't set any vertex/index buffer because the triangle positions are calculated internally using vertex_id.
    void drawVisibilityBufferShade(Cmd* cmd, uint32_t frameIdx);

    // Executes a compute shader to clear (reset) the the light clusters on the GPU
    void clearLightClusters(Cmd* cmd, uint32_t frameIdx);

    // Executes a compute shader that computes the light clusters on the GPU
    void computeLightClusters(Cmd* cmd, uint32_t frameIdx);

    // This is the main scene rendering function. It shows the different steps / rendering passes.
    void drawScene(Cmd* cmd, uint32_t frameIdx);

    void drawSkybox(Cmd* cmd, int frameIdx);

    void drawGodray(Cmd* cmd, uint frameIdx);

    void blurGodRay(Cmd* cmd, uint frameIdx);

    void drawColorconversion(Cmd* cmd);

    static void presentImage(Cmd* const cmd, RenderTarget* pSrc, RenderTarget* pDstCol);

    // Draw GUI / 2D elements
    void drawGUI(Cmd* cmd, uint32_t frameIdx);
};

DEFINE_APPLICATION_MAIN(Visibility_Buffer)
