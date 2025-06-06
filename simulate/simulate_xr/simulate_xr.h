// Copyright 2024 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SIMULATE_XR_H_
#define SIMULATE_XR_H_

#define GLEW_STATIC

// the code only works with Windows, because support for XR on Linux
// and Mac is limited or nonexistent
#ifdef _WIN32

// Windows is needed but without minmax
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#include <unknwn.h>

// for proper init of openxr
#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_OPENGL

#endif // WIN32

// openxr with render by opengl
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

// to link to windows?
#include <mujoco/mujoco.h>

#include <vector>
#include <unordered_map>
#include <algorithm>


class SimulateXr {
 public:
  SimulateXr();
  ~SimulateXr();

  int32_t width = 0;
  int32_t height = 0;
  int32_t width_render = 0;

  // 0 no text except warnings and errors
  // 1 some success messages
  // 2 more information
  // 4 frame-by-frame info
  int verbose = 2;

  void init();
  void deinit();

  void set_scn_params(mjvScene *scn);
  void set_vis_params(mjModel *m);

  bool before_render(mjvScene *scn, mjModel *m);

  void after_render(mjrContext *con);

  bool is_initialized();

 private:
  bool m_initialized = false;

  const float nearZ = 0.05f;
  const float farZ = 50.0f;  // todo switch to 100?

  std::vector<XrView> m_views;

  XrInstance m_xrInstance = XR_NULL_HANDLE;
  std::vector<const char *> m_activeAPILayers = {};
  std::vector<const char *> m_activeInstanceExtensions = {};
  std::vector<std::string> m_apiLayers = {};
  std::vector<std::string> m_instanceExtensions = {};

  XrFormFactor m_formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
  XrSystemId m_systemID = {};
  XrSystemProperties m_systemProperties = {XR_TYPE_SYSTEM_PROPERTIES};
  std::vector<XrViewConfigurationType> m_applicationViewConfigurations = {
      XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
      XR_VIEW_CONFIGURATION_TYPE_PRIMARY_MONO};
  std::vector<XrViewConfigurationType> m_viewConfigurations;
  std::vector<XrViewConfigurationView> m_viewConfigurationViews;
  XrViewConfigurationType m_viewConfiguration =
      XR_VIEW_CONFIGURATION_TYPE_MAX_ENUM;

  std::vector<XrEnvironmentBlendMode> m_applicationEnvironmentBlendModes = {
      XR_ENVIRONMENT_BLEND_MODE_OPAQUE, XR_ENVIRONMENT_BLEND_MODE_ADDITIVE};
  std::vector<XrEnvironmentBlendMode> m_environmentBlendModes = {};
  XrEnvironmentBlendMode m_environmentBlendMode =
      XR_ENVIRONMENT_BLEND_MODE_MAX_ENUM;

  XrSpace m_localSpace = XR_NULL_HANDLE;
  struct RenderLayerInfo {
    XrTime predictedDisplayTime = 0;
    std::vector<XrCompositionLayerBaseHeader *> layers;
    XrCompositionLayerProjection layerProjection = {
        XR_TYPE_COMPOSITION_LAYER_PROJECTION};
    std::vector<XrCompositionLayerProjectionView> layerProjectionViews;
  };

  struct SwapchainInfo {
    XrSwapchain swapchain = XR_NULL_HANDLE;
    int64_t swapchainFormat = 0;
    std::vector<void *> imageViews;
  };
  std::vector<SwapchainInfo> m_colorSwapchainInfos = {};
  SwapchainInfo m_colorSwapchainInfo;
  enum class SwapchainType : uint8_t { COLOR, DEPTH };  // only using color
  std::unordered_map<
      XrSwapchain,
      std::pair<SwapchainType, std::vector<XrSwapchainImageOpenGLKHR>>>
      swapchainImagesMap{};
  // from m_graphicsAPI
  XrSwapchainImageBaseHeader* AllocateSwapchainImageData(XrSwapchain swapchain,
                                                        SwapchainType type,
                                                         uint32_t count);
  void* GetSwapchainImage(XrSwapchain swapchain, uint32_t index) {
    return (void*)(uint64_t)swapchainImagesMap[swapchain].second[index].image;
  }
  struct ImageViewCreateInfo {
    void* image;
    enum class Type : uint8_t { RTV, DSV, SRV, UAV } type;
    enum class View : uint8_t {
      TYPE_1D,
      TYPE_2D,
      TYPE_3D,
      TYPE_CUBE,
      TYPE_1D_ARRAY,
      TYPE_2D_ARRAY,
      TYPE_CUBE_ARRAY,
    } view;
    int64_t format;
    enum class Aspect : uint8_t {
      COLOR_BIT = 0x01,
      DEPTH_BIT = 0x02,
      STENCIL_BIT = 0x04
    } aspect;
    uint32_t baseMipLevel;
    uint32_t levelCount;
    uint32_t baseArrayLayer;
    uint32_t layerCount;
  };
  // should be GLuint, hack
  std::unordered_map<unsigned int, ImageViewCreateInfo> imageViews{};
  void *CreateImageView(const ImageViewCreateInfo &imageViewCI);
  int64_t SelectColorSwapchainFormat(const std::vector<int64_t> &formats);

  PFN_xrGetOpenGLGraphicsRequirementsKHR xrGetOpenGLGraphicsRequirementsKHR =
      nullptr;
  XrGraphicsBindingOpenGLWin32KHR graphicsBinding{};

  XrSession m_session = XR_NULL_HANDLE;
  XrSessionState m_sessionState = XR_SESSION_STATE_UNKNOWN;

  void _view_to_cam(mjvGLCamera &cam, const XrView &view);

  void _fill_layer_proj_views(XrCompositionLayerProjectionView &xr_lpv,
                              const XrView &view, const int32_t offset);

  int _create_instance();
  void _destroy_instance();

  void _get_instance_properties();

  int _get_system_id();

  int _get_view_configuration_views();

  int _get_environment_blend_modes();

  int _create_session();
  void _destroy_session();

  int _create_reference_space();
  void _destroy_reference_space();

  int _create_swapchain();
  void _destroy_swapchain();

  bool m_sessionRunning = false;
  void _poll_events();

  // carried b/w calls before and after render
  // see before_render and after_render functions
  XrFrameState frameState{XR_TYPE_FRAME_STATE};
  RenderLayerInfo renderLayerInfo;
  bool rendered = false;

  void _blit_to_mujoco();
};

#endif  // SIMULATE_XR_H_
