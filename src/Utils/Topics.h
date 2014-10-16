#pragma once

#include <string>

/// ROS topics.
static const std::string SUBSCRIBER_POINTS_LIST_STRING		= "PointsListString";
static const std::string SUBSCRIBER_COMMAND_LOAD			= "CommandLoad";
static const std::string SUBSCRIBER_GOAL_DONE				= "Results";
static const std::string SUBSCRIBER_ROBOT_POSE				= "base_pose_ground_truth";
static const std::string PUBLISHER_END_PATH					= "EndPath";                    /// Responses message are: pathDone e stopDone.
static const std::string PUBLISHER_FEEDBACK_MOTION			= "feedbackMotion";

/// ROS params.
static const std::string PARAMS_FILE_NAME_SERVER			= "pathFilename";

/// Message types to be send between topics.
static const std::string PATH_DONE							= "pathDone";
static const std::string PATH_STOP							= "pathStop";
