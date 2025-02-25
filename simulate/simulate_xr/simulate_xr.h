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

// Windows is needed but without minmax
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#include <unknwn.h>

// TODO future setup for Android build on devices
// for proper init of openxr
#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_OPENGL

// openxr with render by opengl
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

// to link to windows
#include <mujoco/mujoco.h>

#include <vector>
#include <unordered_map>
#include <algorithm>

struct SimulateXrController_ {
  bool is_active = false;
  float pos[3] = {0};
  float rot_quat[4] = {0};
};
typedef struct SimulateXrController_ SimulateXrController;


class SimulateXrControllers {
 public:
  SimulateXrControllers();
  ~SimulateXrControllers();

  int init(XrInstance &xrInstance);

  int init_session(XrInstance &xrInstance, XrSession &session);

  void poll_actions(XrTime predictedTime, XrSession &session,
                    XrSpace &localSpace);
  void process_actions();

  int get_controller_position_left(XrPosef &handPose);
  int get_controller_position_right(XrPosef &handPose);

 private:
  XrActionSet m_actionSet;
  // An action for grabbing blocks, and an action to change the color of a
  // block.
  XrAction m_grabCubeAction, m_spawnCubeAction, m_changeColorAction;
  // The realtime states of these actions.
  XrActionStateFloat m_grabState[2] = {{XR_TYPE_ACTION_STATE_FLOAT},
                                       {XR_TYPE_ACTION_STATE_FLOAT}};
  XrActionStateBoolean m_changeColorState[2] = {{XR_TYPE_ACTION_STATE_BOOLEAN},
                                                {XR_TYPE_ACTION_STATE_BOOLEAN}};
  XrActionStateBoolean m_spawnCubeState = {XR_TYPE_ACTION_STATE_BOOLEAN};
  // The haptic output action for grabbing cubes.
  XrAction m_buzzAction;
  // The current haptic output value for each controller.
  float m_buzz[2] = {0, 0};
  // The action for getting the hand or controller position and orientation.
  XrAction m_palmPoseAction;
  // The XrPaths for left and right hand hands or controllers.
  XrPath m_handPaths[2] = {0, 0};
  // The spaces that represents the two hand poses.
  XrSpace m_handPoseSpace[2];
  XrActionStatePose m_handPoseState[2] = {{XR_TYPE_ACTION_STATE_POSE},
                                          {XR_TYPE_ACTION_STATE_POSE}};
  // In STAGE space, viewHeightM should be 0. In LOCAL space, it should be
  // offset downwards, below the viewer's initial position.
  float m_viewHeightM = 1.5f;
  // The current poses obtained from the XrSpaces.
  XrPosef m_handPose[2] = {
      {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -m_viewHeightM}},
      {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -m_viewHeightM}}};

  XrPath CreateXrPath(const char *path_string, XrInstance &xrInstance);

  //// Init 1
  // used to be lambda
  void create_action(XrAction &xrAction, const char *name,
                    XrActionType xrActionType, XrInstance &xrInstance,
                    std::vector<const char *> subaction_paths = {});
  int create_action_set(XrInstance &xrInstance);

  // used to be lambda
  bool suggest_single_binding(
      const char *profile_path,
      std::vector<XrActionSuggestedBinding> bindings, 
      XrInstance &xrInstance);
  int suggest_bindings(XrInstance &xrInstance);

  //// Init 2 session
  // used to be lambda
  XrSpace create_action_pose_space(XrSession session, XrAction xrAction,
                                   XrInstance &xrInstance,
                                   const char *subaction_path = nullptr);
  int create_action_poses(XrInstance &xrInstance, XrSession &m_session);

  int attach_action_set(XrSession &m_session);

  void copy_xr_posef(XrPosef &from, XrPosef &to);

};


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

  void after_render(mjrContext *con, int window_width, int window_height);

  bool is_initialized();
  bool is_controllers_initialized();

  SimulateXrController simxr_controllers[2];

 private:
  bool m_initialized = false;
  bool m_controllers_initialized = false;

  const float nearZ = 0.05f;
  const float farZ = 50.0f;  // todo switch to 100?

  std::vector<XrView> m_views;

  SimulateXrControllers _sim_xr_controllers;

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

  void _blit_to_mujoco(int dst_width, int dst_height);

  void _before_render_controllers();

  void _hand_to_mujoco_controller(XrPosef &m_handPose,
                                  SimulateXrController &simxr_controllers);
};

#endif  // SIMULATE_XR_H_
