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

#ifndef SIMULATE_XR_CONTROLLERS_H_
#define SIMULATE_XR_CONTROLLERS_H_

#include "simulate_xr_imports.h"

enum class SIMXR_CONTROLLER_GRAB_ACTIONS {
  TRANSLATE = 0,
  ROTATE,
  BOTH,

  NUM_ACTIONS,
};


class SimulateXrControllers {
 public:
  SimulateXrControllers();
  ~SimulateXrControllers();

  // 0 no text except warnings and errors
  // 1 some success messages
  // 2 more information
  // 4 frame-by-frame info
  int verbose = 1;

  int init(XrInstance &xrInstance);

  int init_session(XrInstance &xrInstance, XrSession &session);

  void poll_actions(XrTime predictedTime, XrSession &session,
                    XrSpace &localSpace);
  void process_actions();

  int get_controller_position_left(XrPosef &handPose);
  int get_controller_position_right(XrPosef &handPose);

  bool is_left_controller_grabbing();
  bool is_right_controller_grabbing();

  SIMXR_CONTROLLER_GRAB_ACTIONS get_controller_grab_action();

 private:
  XrActionSet m_actionSet;
  // An action for grabbing (bodies)
  XrAction m_grabAction;
  // switching the type of grabbing
  XrAction m_switchGrabAction;
  // The realtime states of these actions.
  XrActionStateFloat m_grabState[2] = {{XR_TYPE_ACTION_STATE_FLOAT},
                                       {XR_TYPE_ACTION_STATE_FLOAT}};
  XrActionStateBoolean m_switchGrabState = {XR_TYPE_ACTION_STATE_BOOLEAN};
  SIMXR_CONTROLLER_GRAB_ACTIONS m_switchGrabActionState =
      SIMXR_CONTROLLER_GRAB_ACTIONS::TRANSLATE;
  // TODO:
  // The haptic output action for grabbing or switching.
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
  bool suggest_single_binding(const char *profile_path,
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


#endif  // SIMULATE_XR_CONTROLLERS_H_