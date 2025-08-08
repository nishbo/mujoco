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

// include order is VERY important:
// header, glad, glfw3, glfw3native
// autoformat will try to screw with this
#include "simulate_xr_controllers.h"

SimulateXrControllers::SimulateXrControllers() {}

SimulateXrControllers::~SimulateXrControllers() {}

int SimulateXrControllers::init(XrInstance &xrInstance) {
  if (create_action_set(xrInstance)) return -1;
  if (suggest_bindings(xrInstance)) return -2;
  return 0;
}

int SimulateXrControllers::init_session(XrInstance &xrInstance,
                                        XrSession &session) {
  if (create_action_poses(xrInstance, session) < 0) return -1;

  if (attach_action_set(session) < 0) return -2;

  return 0;
}

void SimulateXrControllers::poll_actions(XrTime predictedTime,
                                         XrSession &session,
                                         XrSpace &localSpace) {
  // Update our action set with up-to-date input data.
  // First, we specify the actionSet we are polling.
  XrActiveActionSet activeActionSet{};
  activeActionSet.actionSet = m_actionSet;
  activeActionSet.subactionPath = XR_NULL_PATH;
  // Now we sync the Actions to make sure they have current data.
  XrActionsSyncInfo actionsSyncInfo{XR_TYPE_ACTIONS_SYNC_INFO};
  actionsSyncInfo.countActiveActionSets = 1;
  actionsSyncInfo.activeActionSets = &activeActionSet;
  if (xrSyncActions(session, &actionsSyncInfo) < 0) {
    mju_warning("Failed to sync Actions.");
  } else {
    // printf("Got sync Actions.\n");
  }
  XrActionStateGetInfo actionStateGetInfo{XR_TYPE_ACTION_STATE_GET_INFO};

  // We pose a single Action, twice - once for each subAction Path.
  actionStateGetInfo.action = m_palmPoseAction;
  // For each hand, get the pose state if possible.
  for (int i = 0; i < 2; i++) {
    // Specify the subAction Path.
    actionStateGetInfo.subactionPath = m_handPaths[i];
    if (xrGetActionStatePose(session, &actionStateGetInfo,
                             &m_handPoseState[i]) < 0) {
      mju_warning("Failed to get Pose State.");
    } else {
      if (verbose > 3) printf_s("Got Pose State. ");
    }

    if (m_handPoseState[i].isActive) {
      XrSpaceLocation spaceLocation{XR_TYPE_SPACE_LOCATION};
      XrResult res = xrLocateSpace(m_handPoseSpace[i], localSpace,
                                   predictedTime, &spaceLocation);
      if (XR_UNQUALIFIED_SUCCESS(res) &&
          (spaceLocation.locationFlags &
           XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 &&
          (spaceLocation.locationFlags &
           XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0) {
        m_handPose[i] = spaceLocation.pose;
      } else {
        m_handPoseState[i].isActive = false;
      }
    }
  }
  for (int i = 0; i < 2; i++) {
    actionStateGetInfo.action = m_grabAction;
    actionStateGetInfo.subactionPath = m_handPaths[i];
    if (xrGetActionStateFloat(session, &actionStateGetInfo, &m_grabState[i]) <
        0) {
      // mju_warning("Failed to get Float State of Grab action.");
    }
  }
  {
    actionStateGetInfo.action = m_switchGrabAction;
    actionStateGetInfo.subactionPath = 0;
    if (xrGetActionStateBoolean(session, &actionStateGetInfo,
                                &m_switchGrabState) < 0) {
      // mju_warning("Failed to get Boolean State of Switch Grab action.");
    }
  }
  for (int i = 0; i < 2; i++) {
    m_buzz[i] *= 0.5f;
    if (m_buzz[i] < 0.01f) m_buzz[i] = 0.0f;
    XrHapticVibration vibration{XR_TYPE_HAPTIC_VIBRATION};
    vibration.amplitude = m_buzz[i];
    vibration.duration = XR_MIN_HAPTIC_DURATION;
    vibration.frequency = XR_FREQUENCY_UNSPECIFIED;

    XrHapticActionInfo hapticActionInfo{XR_TYPE_HAPTIC_ACTION_INFO};
    hapticActionInfo.action = m_buzzAction;
    hapticActionInfo.subactionPath = m_handPaths[i];
    if (xrApplyHapticFeedback(session, &hapticActionInfo,
                              (XrHapticBaseHeader *)&vibration) < 0) {
      // mju_warning("Failed to apply haptic feedback.");
    }
  }

  // process button presses - on liftoff
  if (m_switchGrabState.isActive == XR_TRUE &&
      m_switchGrabState.currentState == XR_FALSE &&
      m_switchGrabState.changedSinceLastSync == XR_TRUE) {
    m_switchGrabActionState = static_cast<SIMXR_CONTROLLER_GRAB_ACTIONS>(
        (static_cast<int>(m_switchGrabActionState) + 1) %
        static_cast<int>(SIMXR_CONTROLLER_GRAB_ACTIONS::NUM_ACTIONS));
    if (verbose > 2) {
      printf_s(" Switched grab action state to %d.",
               static_cast<int>(m_switchGrabActionState));
    }
  }

  // print some stuff
  if (verbose > 3) {
    if (m_handPoseState[0].isActive == XR_TRUE) {
      printf_s(" controller 1: %.6f", m_handPose[0].position.x);
    }
    if (m_handPoseState[1].isActive == XR_TRUE) {
      printf_s(" controller 2: %.6f", m_handPose[1].position.x);
    }

    printf_s("\n");
  }
}

void SimulateXrControllers::process_actions() {
  // not implemented
}

int SimulateXrControllers::get_controller_position_left(XrPosef &handPose) {
  if (m_handPoseState[0].isActive) {
    copy_xr_posef(m_handPose[0], handPose);
    return 0;
  }
  return -1;
}

int SimulateXrControllers::get_controller_position_right(XrPosef &handPose) {
  if (m_handPoseState[1].isActive) {
    copy_xr_posef(m_handPose[1], handPose);
    return 0;
  }
  return -1;
}

bool SimulateXrControllers::is_left_controller_grabbing() {
  if (m_handPoseState[0].isActive) {
    return m_grabState[0].currentState > 0.5f;
  }
  return false;
}

bool SimulateXrControllers::is_right_controller_grabbing() {
  if (m_handPoseState[1].isActive) {
    return m_grabState[1].currentState > 0.5f;
  }
  return false;
}

SIMXR_CONTROLLER_GRAB_ACTIONS
SimulateXrControllers::get_controller_grab_action() {
  return m_switchGrabActionState;
}

XrPath SimulateXrControllers::CreateXrPath(const char *path_string,
                                           XrInstance &xrInstance) {
  XrPath xrPath;
  if (xrStringToPath(xrInstance, path_string, &xrPath) < 0) {
    mju_warning("Failed to create XrPath from string: ", path_string, ".");
  }
  return xrPath;
}

void SimulateXrControllers::create_action(
    XrAction &xrAction, const char *name, XrActionType xrActionType,
    XrInstance &xrInstance, std::vector<const char *> subaction_paths) {
  XrActionCreateInfo actionCI{XR_TYPE_ACTION_CREATE_INFO};
  // The type of action: float input, pose, haptic output etc.
  actionCI.actionType = xrActionType;
  // Subaction paths, e.g. left and right hand. To distinguish the same action
  // performed on different devices.
  std::vector<XrPath> subaction_xrpaths;
  for (auto p : subaction_paths) {
    subaction_xrpaths.push_back(CreateXrPath(p, xrInstance));
  }
  actionCI.countSubactionPaths = (uint32_t)subaction_xrpaths.size();
  actionCI.subactionPaths = subaction_xrpaths.data();
  // The internal name the runtime uses for this Action.
  strncpy(actionCI.actionName, name, XR_MAX_ACTION_NAME_SIZE);
  // Localized names are required so there is a human-readable action name to
  // show the user if they are rebinding the Action in an options screen.
  strncpy(actionCI.localizedActionName, name,
          XR_MAX_LOCALIZED_ACTION_NAME_SIZE);
  if (xrCreateAction(m_actionSet, &actionCI, &xrAction) < 0)
    mju_warning("Failed to create Action.");
}

int SimulateXrControllers::create_action_set(XrInstance &xrInstance) {
  XrActionSetCreateInfo actionSetCI{XR_TYPE_ACTION_SET_CREATE_INFO};
  // The internal name the runtime uses for this Action Set.
  strncpy(actionSetCI.actionSetName, "pulling-actionset",
          XR_MAX_ACTION_SET_NAME_SIZE);
  // Localized names are required so there is a human-readable action name to
  // show the user if they are rebinding Actions in an options screen.
  strncpy(actionSetCI.localizedActionSetName, "Pulling ActionSet",
          XR_MAX_LOCALIZED_ACTION_SET_NAME_SIZE);
  // Set a priority: this comes into play when we have multiple Action Sets, and
  // determines which Action takes priority in binding to a specific input.
  actionSetCI.priority = 0;

  if (xrCreateActionSet(xrInstance, &actionSetCI, &m_actionSet) < 0) {
    mju_warning("ERROR: Failed to create ActionSet.");
    return -1;
  }

  // An Action for grabbing.
  create_action(m_grabAction, "grab", XR_ACTION_TYPE_FLOAT_INPUT, xrInstance,
                {"/user/hand/left", "/user/hand/right"});
  // An Action for switching grabbing actions.
  create_action(m_switchGrabAction, "switch-grab", XR_ACTION_TYPE_BOOLEAN_INPUT,
                xrInstance);
  // An Action for the position of the palm of the user's hand - appropriate for
  // the location of a grabbing Actions.
  create_action(m_palmPoseAction, "palm-pose", XR_ACTION_TYPE_POSE_INPUT,
                xrInstance, {"/user/hand/left", "/user/hand/right"});
  // An Action for a vibration output on one or other hand.
  create_action(m_buzzAction, "buzz", XR_ACTION_TYPE_VIBRATION_OUTPUT,
                xrInstance, {"/user/hand/left", "/user/hand/right"});
  // For later convenience we create the XrPaths for the subaction path names.
  m_handPaths[0] = CreateXrPath("/user/hand/left", xrInstance);
  m_handPaths[1] = CreateXrPath("/user/hand/right", xrInstance);

  return 0;
}

bool SimulateXrControllers::suggest_single_binding(
    const char *profile_path, std::vector<XrActionSuggestedBinding> bindings,
    XrInstance &xrInstance) {
  // The application can call xrSuggestInteractionProfileBindings once per
  // interaction profile that it supports.
  XrInteractionProfileSuggestedBinding interactionProfileSuggestedBinding{
      XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
  interactionProfileSuggestedBinding.interactionProfile =
      CreateXrPath(profile_path, xrInstance);
  interactionProfileSuggestedBinding.suggestedBindings = bindings.data();
  interactionProfileSuggestedBinding.countSuggestedBindings =
      (uint32_t)bindings.size();
  if (xrSuggestInteractionProfileBindings(
          xrInstance, &interactionProfileSuggestedBinding) ==
      XrResult::XR_SUCCESS)
    return true;
  mju_warning("Failed to suggest bindings with ", profile_path, ".");
  return false;
}

int SimulateXrControllers::suggest_bindings(XrInstance &xrInstance) {
  bool any_ok = false;
  // TODO (AS) Check handedness
  // Each Action here has two paths, one for each SubAction path.
  any_ok |= suggest_single_binding(
      "/interaction_profiles/khr/simple_controller",
      {
       {m_grabAction,
        CreateXrPath("/user/hand/left/input/select/click", xrInstance)},
       {m_grabAction,
        CreateXrPath("/user/hand/right/input/select/click", xrInstance)},
       {m_switchGrabAction,
        CreateXrPath("/user/hand/right/input/menu/click", xrInstance)},
       {m_palmPoseAction,
        CreateXrPath("/user/hand/left/input/grip/pose", xrInstance)},
       {m_palmPoseAction,
        CreateXrPath("/user/hand/right/input/grip/pose", xrInstance)},
       {m_buzzAction,
        CreateXrPath("/user/hand/left/output/haptic", xrInstance)},
       {m_buzzAction,
        CreateXrPath("/user/hand/right/output/haptic", xrInstance)}},
      xrInstance);
  // Each Action here has two paths, one for each SubAction path.
  // seems to work okay for MetaXr - cannot find binding for buttons
  any_ok |= suggest_single_binding(
      "/interaction_profiles/oculus/touch_controller",
      {
       {m_grabAction,
        CreateXrPath("/user/hand/left/input/trigger/value", xrInstance)},
       {m_grabAction,
        CreateXrPath("/user/hand/right/input/trigger/value", xrInstance)},
       {m_switchGrabAction,
        CreateXrPath("/user/hand/right/input/a/click", xrInstance)},
       {m_palmPoseAction,
        CreateXrPath("/user/hand/left/input/grip/pose", xrInstance)},
       {m_palmPoseAction,
        CreateXrPath("/user/hand/right/input/grip/pose", xrInstance)},
       {m_buzzAction,
        CreateXrPath("/user/hand/left/output/haptic", xrInstance)},
       {m_buzzAction,
        CreateXrPath("/user/hand/right/output/haptic", xrInstance)}},
      xrInstance);
  //// specifically for quest 3 controllers - Meta Quest Touch Pro Controller 
  //any_ok |= suggest_single_binding(
  //    "/interaction_profiles/facebook/touch_controller_pro",
  //    {{m_grabAction,
  //      CreateXrPath("/user/hand/left/input/trigger/value", xrInstance)},
  //     {m_grabAction,
  //      CreateXrPath("/user/hand/right/input/trigger/value", xrInstance)},
  //     {m_switchGrabAction,
  //      CreateXrPath("/user/hand/right/input/a/click", xrInstance)},
  //     {m_palmPoseAction,
  //      CreateXrPath("/user/hand/left/input/grip/pose", xrInstance)},
  //     {m_palmPoseAction,
  //      CreateXrPath("/user/hand/right/input/grip/pose", xrInstance)},
  //     {m_buzzAction,
  //      CreateXrPath("/user/hand/left/output/haptic", xrInstance)},
  //     {m_buzzAction,
  //      CreateXrPath("/user/hand/right/output/haptic", xrInstance)}},
  //    xrInstance);
  if (!any_ok) {
    mju_warning("ERROR: Could not select binding.");
    return -1;
  }

  return 0;
}

XrSpace SimulateXrControllers::create_action_pose_space(
    XrSession session, XrAction xrAction, XrInstance &xrInstance,
    const char *subaction_path) {
  // Create an xrSpace for a pose action.
  XrSpace xrSpace;
  const XrPosef xrPoseIdentity = {{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}};
  // Create frame of reference for a pose action
  XrActionSpaceCreateInfo actionSpaceCI{XR_TYPE_ACTION_SPACE_CREATE_INFO};
  actionSpaceCI.action = xrAction;
  actionSpaceCI.poseInActionSpace = xrPoseIdentity;
  if (subaction_path)
    actionSpaceCI.subactionPath = CreateXrPath(subaction_path, xrInstance);
  if (xrCreateActionSpace(session, &actionSpaceCI, &xrSpace) < 0)
    mju_warning("Failed to create ActionSpace.");
  return xrSpace;
}

int SimulateXrControllers::create_action_poses(XrInstance &xrInstance,
                                               XrSession &m_session) {
  m_handPoseSpace[0] = create_action_pose_space(m_session, m_palmPoseAction,
                                                xrInstance, "/user/hand/left");
  m_handPoseSpace[1] = create_action_pose_space(m_session, m_palmPoseAction,
                                                xrInstance, "/user/hand/right");

  return 0;
}

int SimulateXrControllers::attach_action_set(XrSession &m_session) {
  // Attach the action set we just made to the session. We could attach multiple
  // action sets!
  XrSessionActionSetsAttachInfo actionSetAttachInfo{
      XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
  actionSetAttachInfo.countActionSets = 1;
  actionSetAttachInfo.actionSets = &m_actionSet;
  if (xrAttachSessionActionSets(m_session, &actionSetAttachInfo) < 0) {
    mju_warning("Failed to attach ActionSet to Session.");
    return -1;
  }
  return 0;
}

void SimulateXrControllers::copy_xr_posef(XrPosef &from, XrPosef &to) {
  to.position.x = from.position.x;
  to.position.y = from.position.y;
  to.position.z = from.position.z;

  to.orientation.w = from.orientation.w;
  to.orientation.x = from.orientation.x;
  to.orientation.y = from.orientation.y;
  to.orientation.z = from.orientation.z;
}
