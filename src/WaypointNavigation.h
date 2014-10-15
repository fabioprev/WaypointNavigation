#ifndef WAYPOINT_NAVIGATION_H
#define WAYPOINT_NAVIGATION_H

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/node_handle.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <list>

namespace SSI
{
	class WaypointNavigation
	{
		protected:
			typedef std::vector<geometry_msgs::Pose> PosesQueue;
			
			/**
			 *	Represents the Waypoint structure.
			 */
			struct Waypoints
			{
				PosesQueue points;
				bool isCyclic;
				
				Waypoints() : isCyclic(false) {;}
				
				Waypoints(const Waypoints& waypoints)
				{
					this->isCyclic = waypoints.isCyclic;
					this->points = waypoints.points;
				}
			};
			
			typedef std::map<std::string,Waypoints> MapPosesQueue;
			
			/**
			 *	Represents the object used for communicating with the move_base navigation node.
			 */
			actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* actionClient;
			
			/**
			 *	Handle to a ros node.
			 */
			ros::NodeHandle nodeHandle;
			
			/**
			 *	Subscriber associated to topic SUBSCRIBER_POINTS_LIST_STRING.
			 */
			ros::Subscriber subscriberCommandPath;
			
			/**
			 *	Subscriber associated to topic SUBSCRIBER_COMMAND_LOAD.
			 */
			ros::Subscriber subscriberCommandControl;
			
			/**
			 *	Subscriber associated to topic SUBSCRIBER_GOAL_DONE.
			 */
			ros::Subscriber subscriberGoalDone;
			
			/**
			 *	Subscriber associated to topic SUBSCRIBER_ROBOT_POSE.
			 */
			ros::Subscriber subscriberRobotPose;
			
			/**
			 *	Publisher for sending responses to whom subscribed PUBLISHER_END_PATH.
			 */
			ros::Publisher publisherStringFeedback;
			
			/**
			 *	Publisher for sending feedback to whom subscribed PUBLISHER_FEEDBACK_MOTION.
			 */
			ros::Publisher publisherCoordinationFeedback;
			
			/**
			 *	Represents the map of points' list.
			 */
			MapPosesQueue posesQueueMap;
			
			/**
			 *	Represents the points' list.
			 */
			PosesQueue posesQueue;
			
			/**
			 *	Represents the next point in the list.
			 */
			PosesQueue::iterator nextPoint;
			
			/**
			 *	Represents the path of the directory containing all the files.
			 */
			std::string pathDirectory;
			
			/**
			 *	Represents the name of the file containing the points' list.
			 */
			std::string pathFilename;
			
			/**
			 *	Represents the current path in execution.
			 */
			std::string pathName;
			
			/**
			 *	Represents the last path executed.
			 */
			std::string lastPathName;
			
			/**
			 *	Represents the robot pose.
			 */
			double robotPoseX, robotPoseY, robotPoseTheta;
			
			/**
			 *	Represents the old robot pose.
			 */
			double oldRobotPoseX, oldRobotPoseY, oldRobotPoseTheta;
			
			/**
			 *	Represents the agent id.
			 */
			int agentId;
			
			/**
			 *	Represents the last index of the action in execution.
			 */
			int lastPathIndex;
			
			/**
			 *	Represents the index of the action in execution.
			 */
			int pathIndex;
			
			/**
			 *	Am I executing a given path?
			 */
			bool executingPath;
			
			/**
			 *	Is it a cyclic path?
			 */
			bool isCyclic;
			
			/**
			 *	Am I using move base?
			 */
			bool isMoveBase;
			
			/**
			 *	@return returns an empty waypoints list.
			 */
			static std::pair<std::string,Waypoints> emptyWaypointList();
			
			/**
			 *	In order to send a new goal to amcl node.
			 */
			void goToNextGoal();
			
			/**
			 *	Make ready for execution a new path received.
			 *	@return return true if everything is gone well, otherwise false.
			 */
			bool initCustomPath(std::stringstream& pathStream);
			
			/**
			 *	Make ready for execution a standard path.
			 *	@return return true if everything is gone well, otherwise false.
			 */
			bool initStandardPath(MapPosesQueue::iterator it);
			
			/**
			 *	The path filename is read from a topic.
			 *	@return returns true if the file has been read correctly, otherwise false.
			 */
			bool loadFilePath(const std::string& filename);
			
		public:
			/**
			 *	Class constructor. Here, every ROS structure is inizialized.
			 */
			WaypointNavigation();
			
			/**
			 *	Class destructor. Here, every structure is destroyed.
			 */
			virtual ~WaypointNavigation();
			
			/**
			 *	Callback associated to topic SUBSCRIBER_COMMAND_LOAD.
			 *	@param message represents the message read from the topic for reading a new file and then for sending a message "reloadFile".
			 */
			void commandLoadCallback(const std_msgs::String::ConstPtr& message);
			
			/**
			 *	Callback associated to topic SUBSCRIBER_POINTS_LIST_STRING.
			 *	@param message represents the message read from the topic.
			 */
			void commandPathCallback(const std_msgs::String::ConstPtr& message);
			
			/**
			 *	Callback associated to goal topic.
			 *	@param message represents the message read from the topic.
			 */
			void goalDoneCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& message);
			
			/**
			 *	Initialization of every structure used.
			 *	@return returns true if the initialization is gone well, otherwise false.
			 */
			bool init();
			
			/**
			 *	Callback associated to robot pose topic.
			 *	@param message represents the message read from the topic.
			 */
			void updateRobotPose(const nav_msgs::Odometry::ConstPtr& message);
	};
}

#endif
