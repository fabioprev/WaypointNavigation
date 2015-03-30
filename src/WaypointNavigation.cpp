#include "WaypointNavigation.h"
#include "Utils/Topics.h"
#include "Utils/Utils.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <fstream>

using namespace std;
using namespace ros;
using geometry_msgs::Quaternion;
using geometry_msgs::Pose;
using geometry_msgs::PoseWithCovarianceStamped;
using move_base_msgs::MoveBaseActionResult;
using move_base_msgs::MoveBaseGoal;
using nav_msgs::Odometry;
using std_msgs::String;

namespace SSI
{
	WaypointNavigation::WaypointNavigation() : nodeHandle("~"), nextPoint(0), pathDirectory("."), pathFilename(""), pathName(""), lastPathName(""), lastPathIndex(0), pathIndex(-1), executingPath(false), isCyclic(false)
	{
		nodeHandle.getParam("agentId",agentId);
		nodeHandle.getParam(PARAMS_FILE_NAME_SERVER,pathFilename);
		nodeHandle.getParam("/isMoveBase",isMoveBase);
		
		pathDirectory = pathFilename.substr(0,pathFilename.rfind("/"));
		
		subscriberCommandPath = nodeHandle.subscribe<String>(SUBSCRIBER_POINTS_LIST_STRING,1,boost::bind(&SSI::WaypointNavigation::commandPathCallback,this,_1));
		subscriberCommandControl = nodeHandle.subscribe<String>(SUBSCRIBER_COMMAND_LOAD,1,boost::bind(&SSI::WaypointNavigation::commandLoadCallback,this,_1));
		subscriberGoalDone = nodeHandle.subscribe<>(SUBSCRIBER_GOAL_DONE,1024,&SSI::WaypointNavigation::goalDoneCallback,this);
		subscriberRobotPose = nodeHandle.subscribe(SUBSCRIBER_ROBOT_POSE,1024,&SSI::WaypointNavigation::updateRobotPose,this);
		
		publisherCoordinationFeedback = nodeHandle.advertise<String>(PUBLISHER_FEEDBACK_MOTION,1);
		publisherStringFeedback = nodeHandle.advertise<String>(PUBLISHER_END_PATH,1);
		
		actionClient = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base",true);
		
		Utils::println(string("File to read: ") + pathFilename + string("."),Utils::Yellow);
	}
	
	WaypointNavigation::~WaypointNavigation()
	{
		if (actionClient != 0) delete actionClient;
	}
	
	void WaypointNavigation::checkRobotStacked()
	{
		static int64_t initialStackTime;
		static bool firstTime = true;
		
		/// Recovery procedure when the robot's stacking for an unknown reason...
		if (/*isMoveBase &&*/ executingPath)
		{
			bool hasReachedGoal;
			
			mutex.lock();
			
			--nextPoint;
			hasReachedGoal = false;
			
			float theta = tf::getYaw(nextPoint->orientation);
			
			if ((fabs(robotPoseX - nextPoint->position.x) < GOAL_DISTANCE_THRESHOLD) && (fabs(robotPoseY - nextPoint->position.y) < GOAL_DISTANCE_THRESHOLD) && (fabs(robotPoseTheta - theta) < GOAL_ANGLE_THRESHOLD))
			{
				Utils::println("Robot has reached the goal.",Utils::Blue);
				
				hasReachedGoal = true;
			}
			
			++nextPoint;
			
			mutex.unlock();
			
			if (hasReachedGoal)
			{
				goToNextGoal();
				
				return;
			}
			
			if ((fabs(robotPoseX - oldRobotPoseX) < 0.05) && (fabs(robotPoseY - oldRobotPoseY) < 0.05) && (fabs(robotPoseTheta - oldRobotPoseTheta) < 0.05))
			{
				if (firstTime)
				{
					initialStackTime = Time::now().toNSec();
					firstTime = false;
				}
				else
				{
					if ((Time::now().toNSec() - initialStackTime) > 2e9)
					{
						Utils::println("Robot's stacking... I gotta send the point again.",Utils::Yellow);
						
						/// I send the point again.
						if (pathIndex >= 0)
						{
							--pathIndex;
						}
						
						goToNextGoal();
						
						firstTime = true;
					}
				}
			}
			else
			{
				mutex.lock();
				
				oldRobotPoseX = robotPoseX;
				oldRobotPoseY = robotPoseY;
				oldRobotPoseTheta = robotPoseTheta;
				
				mutex.unlock();
				
				firstTime = true;
			}
		}
	}
	
	void WaypointNavigation::commandLoadCallback(const String::ConstPtr& message)
	{
		stringstream s;
		string command = message->data, filename;
		
		transform(command.begin(),command.end(),command.begin(),::tolower);
		
		s << command;
		s >> command >> filename;
		
		if (command == "reloadfile") loadFilePath(pathDirectory + string("/") + filename);
		else if (command == "reloadfile_reset")
		{
			pathName = "";
			lastPathName = "";
			lastPathIndex = 0;
			pathIndex = -1;
			executingPath = false;
			
			actionClient->cancelAllGoals();
			
			loadFilePath(pathDirectory + string("/") + filename);
		}
	}
	
	void WaypointNavigation::commandPathCallback(const String::ConstPtr& message)
	{
		Utils::print("I received this path: ",Utils::White);
		Utils::println(message->data,Utils::Cyan);
		
		/// Checking if the path already exists. If so, I load it otherwise I check if it is valid and then I load it.
		stringstream pathStream(stringstream::in | stringstream::out);
		bool isPathLoaded;
		
		pathStream << message->data;
		
		if (!pathStream.good())
		{
			Utils::println("Wrong message sent...",Utils::Red);
			
			return;
		}
		
		pathStream >> pathName;
		
		transform(pathName.begin(),pathName.end(),pathName.begin(),::tolower);
		
		if (strcasecmp(pathName.c_str(),lastPathName.c_str()) != 0)
		{
			/// Resetting current path.
			posesQueue.clear();
			nextPoint = posesQueue.end();
			
			if (pathName == "cancelling")
			{
				lastPathName = "";
				pathIndex = -1;
				executingPath = false;
				
				actionClient->cancelAllGoals();
			}
			else if (pathName == "interrupt")
			{
				if (pathIndex >= 0) lastPathIndex = pathIndex;
				
				executingPath = false;
				actionClient->cancelAllGoals();
			}
			else
			{
				MapPosesQueue::iterator it = posesQueueMap.find(pathName);
				
				if (it == posesQueueMap.end()) isPathLoaded = initCustomPath(pathStream);
				else isPathLoaded = initStandardPath(it);
				
				if (isPathLoaded)
				{
					Utils::println("New path loaded correctly.\n",Utils::Green);
					
					if (pathName != "gotopose")
					{
						lastPathName = pathName;
						pathIndex = -1;
					}
					
					/// Cancelling old goals.
					//actionClient->cancelAllGoals();
					goToNextGoal();
					
					executingPath = true;
				}
				else Utils::println("Failed in loading new path file.\n",Utils::Red);
			}
		}
		else
		{
			MapPosesQueue::iterator it = posesQueueMap.find(pathName);
			
			if (it == posesQueueMap.end()) isPathLoaded = initCustomPath(pathStream);
			else isPathLoaded = initStandardPath(it);
			
			Utils::print("Resuming path ",Utils::Yellow);
			Utils::println(pathName,Utils::Red);
			
			nextPoint = posesQueue.begin() + lastPathIndex;
			pathIndex = lastPathIndex - 1;
			
			/// Cancelling old goals.
			//actionClient->cancelAllGoals();
			goToNextGoal();
			
			executingPath = true;
		}
	}
	
	pair<string,WaypointNavigation::Waypoints> WaypointNavigation::emptyWaypointList()
	{
		return make_pair("stop",Waypoints());
	}
	
	void WaypointNavigation::goalDoneCallback(const MoveBaseActionResult::ConstPtr&)
	{
		String toSend;
		stringstream s;
		string state;
		
		state = actionClient->getState().toString();
		
		s << agentId;
		
		toSend.data = /*_s.str() + " " + */state;
		
		publisherCoordinationFeedback.publish(toSend);
		
		Utils::print("Client response (MoveBase) -> ",Utils::White);
		Utils::println(state,Utils::Cyan);
		
		if (state == "SUCCEEDED")
		{
			String feedback;
			
			s.str("");
			s.clear();
			
			if (nextPoint == posesQueue.end()) s << "Robot " << agentId << " " << ((pathName == "") ? "none" : pathName) << " done";
			else s << "Agent " << agentId << " none" << ((posesQueue.size() == 1) ? " doneChasing" : " donePatroling");
			
			feedback.data = s.str();
			//publisherCoordinationFeedback.publish(feedback);
			
			goToNextGoal();
		}
		else
		{
			if ((state == "PREEMPTED") && (posesQueue.size() > 0))
			{
				Utils::println("I gotta send the point again.",Utils::Yellow);
				
				/// I send the point again.
				if (pathIndex >= 0)
				{
					--pathIndex;
					--nextPoint;
				}
				
				goToNextGoal();
			}
			else if (state == "ACTIVE")
			{
				mutex.lock();
				
				--nextPoint;
				
				float theta = tf::getYaw(nextPoint->orientation);
				
				if ((fabs(robotPoseX - nextPoint->position.x) < GOAL_DISTANCE_THRESHOLD) && (fabs(robotPoseY - nextPoint->position.y) < GOAL_DISTANCE_THRESHOLD) && (fabs(robotPoseTheta - theta) < GOAL_ANGLE_THRESHOLD))
				{
					Utils::println("Robot has reached the goal.",Utils::Blue);
					
					String toSendSucceeded;
					
					toSendSucceeded.data = /*_s.str() + " " + */"FINISHED";
					publisherCoordinationFeedback.publish(toSendSucceeded);
					
					++nextPoint;
				}
				
				goToNextGoal();
				
				mutex.unlock();
			}
		}
	}
	
	void WaypointNavigation::goToNextGoal()
	{
		if ((nextPoint == posesQueue.end()) && isCyclic) nextPoint = posesQueue.begin();
		
		/// Checking if the path is finished.
		if (nextPoint != posesQueue.end())
		{
			ostringstream s;
			
			s << "I'm executing step: (" << nextPoint->position.x << "," << nextPoint->position.y << ").";
			
			Utils::println(s.str(),Utils::White);
			
			/// Each launch file must contains a remap of topic /map for each robot in the simulator because the map_server node ain't launched in a
			/// particular namespace.
			///
			///	Example:
			///		- robot_1.launch:
			///			<group ns="robot_1">
			///				<remap from="/robot_1/map" to="/map" />
			///				...
			///			</group>
			///
			///		- robot_2.launch:
			///			<group ns="robot_2">
			///				<remap from="/robot_2/map" to="/map" />
			///				...
			///			</group>
			
			stringstream temp;
			
			temp << "/map";
			
			if (isMoveBase)
			{
				MoveBaseGoal goal;
				
				goal.target_pose.header.frame_id = temp.str();
				goal.target_pose.header.stamp = Time::now();
				
				/// rand is used to avoid nasty behaviors...
				goal.target_pose.pose.position.x = nextPoint->position.x + ((rand() % 2) * 0.1);
				goal.target_pose.pose.position.y = nextPoint->position.y;
				goal.target_pose.pose.orientation.z = nextPoint->orientation.z;
				goal.target_pose.pose.orientation.w = nextPoint->orientation.w;
				
				//actionClient->sendGoalAndWait(goal,Duration(0.0),Duration(1.0));
				actionClient->sendGoal(goal);
				//actionClient->cancelGoal();
			}
			
			++pathIndex;
			++nextPoint;
		}
		else
		{
			String message;
			
			/// Here the navigation phase is done. There are 2 cases:
			///		- the queue is empty so I received a stop command.
			///		- the queue is not empty but I completed the path. In this case the navigation phase is actually done.
			if (posesQueue.size() > 0) message.data = PATH_DONE;
			else message.data = PATH_STOP;
			
			executingPath = false;
			
			publisherStringFeedback.publish(message);
			
			//if (isMoveBase) actionClient->cancelAllGoals();
		}
	}
	
	bool WaypointNavigation::init()
	{
		if (isMoveBase)
		{
			Utils::println("Waiting that move_base comes up...",Utils::White);
			
			if (!actionClient->waitForServer())
			{
				Utils::println("Action server is not responding...",Utils::Red);
				
				return false;
			}
			
			Utils::println("move_base has started!",Utils::Magenta);
		}
		
		/// A stop command is inserted in the queue.
		posesQueueMap.insert(emptyWaypointList());
		
		/// Reading configuration file.
		bool loadFile = loadFilePath(pathFilename);
		
		if (loadFile)
		{
			Utils::println("\n********************************************************",Utils::Blue);
			Utils::println("\tWaypointNavigation initialization done",Utils::Green);
			Utils::println("********************************************************\n",Utils::Blue);
		}
		else Utils::println("Unable to read waypoints' list file.",Utils::Yellow);
		
		return loadFile;
	}
	
	bool  WaypointNavigation::initCustomPath(stringstream& pathStream)
	{
		string angle;
		double x, y, theta;
		
		Utils::print(string("Reading a custom path: ") + pathStream.str() + string(" and it is "),Utils::White);
		
		pathStream >> isCyclic;
		
		if (isCyclic) Utils::println("cyclic.",Utils::White);
		else Utils::println("non-cyclic.",Utils::White);
		
		if (pathStream.eof()) return false;
		
		while (pathStream.good())
		{
			pathStream >> x >> y >> angle;
			
			if (angle == "inf") theta = 2 * M_PI;
			else theta = Utils::deg2rad(atof(angle.c_str()));
			
			Pose p;
			
			p.position.x = x;
			p.position.y = y;
			p.position.z = 0.0;
			p.orientation.z = sin(theta / 2);
			p.orientation.w = cos(theta / 2);
			
			ostringstream s;
			
			s << "Adding to custom path point: (" << p.position.x << "," << p.position.y << "," << p.orientation.z << "," << p.orientation.w << ").";
			
			Utils::println(s.str(),Utils::White);
			
			posesQueue.push_back(p);
		}
		
		nextPoint = posesQueue.begin();
		
		return true;
	}
	
	bool WaypointNavigation::initStandardPath(MapPosesQueue::iterator it)
	{
		isCyclic = it->second.isCyclic;
		
		posesQueue = it->second.points;
		nextPoint = posesQueue.begin();
		
		ostringstream s;
		
		s << "I'm gonna execute a new " << ((isCyclic) ? "cyclic" : "non-cyclic") << " path with " << posesQueue.size() << " points.";
		
		Utils::println(s.str(),Utils::White);
		
		return true;
	}
	
	bool WaypointNavigation::loadFilePath(const string& filename)
	{
		ifstream file;
		string line, path;
		
		/// Opening file.
		file.open(filename.c_str());
		
		if (!file.is_open())
		{
			Utils::println(string("Failed to open file: ") + filename,Utils::Red);
			
			return false;
		}
		
		while (getline(file,line))
		{
			istringstream iss(line);
			
			iss >> path;
			
			transform(path.begin(),path.end(),path.begin(),::tolower);
			
			/// Checking if the path has been already inserted in queue.
			MapPosesQueue::iterator it = posesQueueMap.find(path);
			
			/// The path is new.
			if (it == posesQueueMap.end())
			{
				/// Inserting new points' list.
				Waypoints newWaypoints;
				string angle;
				double x, y, theta;
				
				iss >> newWaypoints.isCyclic;
				
				if (newWaypoints.isCyclic) Utils::println("Found a cyclic path.",Utils::Blue);
				else Utils::println("Found a non cyclic path.",Utils::Blue);
				
				while (iss.good())
				{
					iss >> x >> y >> angle;
					
					if (angle == "inf") theta = 2 * M_PI;
					else theta = Utils::deg2rad(atof(angle.c_str()));
					
					Pose p;
					
					p.position.x = x;
					p.position.y = y;
					p.position.z = 0.0;
					p.orientation = tf::createQuaternionMsgFromYaw(theta);
					
					newWaypoints.points.push_back(p);
					
					stringstream s;
					
					s << "Adding to " << path << " point: (" << p.position.x << "," << p.position.y << "," << p.orientation.z << "," << p.orientation.w << ").";
					
					Utils::println(s.str(),Utils::White);
				}
				
				transform(path.begin(),path.end(),path.begin(),::tolower);
				
				/// Inserting the new path in the queue.
				posesQueueMap.insert(make_pair<string,Waypoints>(path,newWaypoints));
			}
			else Utils::println("Found an existing path.",Utils::Magenta);
		}
		
		Utils::println("File reading phase done.",Utils::Green);
		
		return true;
	}
	
#ifndef LOCALIZER
	void WaypointNavigation::updateRobotPose(const Odometry::ConstPtr& message)
	{
		double roll, pitch, yaw;
		
		const Quaternion& q = message->pose.pose.orientation;
		
		tf::Matrix3x3(tf::Quaternion(q.x,q.y,q.z,q.w)).getRPY(roll,pitch,yaw);
		
		mutex.lock();
		
		robotPoseX = message->pose.pose.position.x;
		robotPoseY = message->pose.pose.position.y;
		robotPoseTheta = yaw;
		
		mutex.unlock();
		
		checkRobotStacked();
	}
#else
	void WaypointNavigation::updateRobotPose(const PoseWithCovarianceStamped::ConstPtr& message)
	{
		double roll, pitch, yaw;
		
		const Quaternion& q = message->pose.pose.orientation;
		
		tf::Matrix3x3(tf::Quaternion(q.x,q.y,q.z,q.w)).getRPY(roll,pitch,yaw);
		
		mutex.lock();
		
		robotPoseX = message->pose.pose.position.x;
		robotPoseY = message->pose.pose.position.y;
		robotPoseTheta = yaw;
		
		mutex.unlock();
		
		checkRobotStacked();
	}
#endif
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"WaypointNavigation");
	
	SSI::WaypointNavigation waypointNavigation;
	
	if (!waypointNavigation.init())
	{
		SSI::Utils::println("Something went wrong during the initialization phase... Exiting!",SSI::Utils::Red);
		
		return -1;
	}
	
	ros::spin();
	
	return 0;
}
