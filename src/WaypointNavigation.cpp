#include "WaypointNavigation.h"
#include "WaypointNavigationNames.h"
#include "Utils/Utils.h"
#include <ros/ros.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <LinearMath/btQuaternion.h>
#include <fstream>

using namespace std;
using namespace ros;
using geometry_msgs::Pose;
using geometry_msgs::Quaternion;
using move_base_msgs::MoveBaseActionResult;
using move_base_msgs::MoveBaseGoal;
using nav_msgs::Odometry;
using std_msgs::String;

namespace SSI
{
	WaypointNavigation::WaypointNavigation(const string& serverActionName) : actionClient(serverActionName,true), nodeHandle("~"),
																			 nextPoint(0), pathDirectory(""), pathFilename(""),
																			 executingPath(false), isCyclic(false)
	{
		nodeHandle.getParam("agentId",agentId);
		nodeHandle.getParam(PARAMS_FILE_NAME_SERVER,pathFilename);
		
		pathDirectory = pathFilename.substr(0,pathFilename.rfind("/"));
		
		subscriberCommandPath = nodeHandle.subscribe<String>(SUBSCRIBER_POINTS_LIST_STRING,1,boost::bind(&SSI::WaypointNavigation::commandPathCallback,
																										 this,_1));
		
		subscriberCommandControl = nodeHandle.subscribe<String>(SUBSCRIBER_COMMAND_LOAD,1,boost::bind(&SSI::WaypointNavigation::commandLoadCallback,
																									  this,_1));
		
		subscriberGoalDone = nodeHandle.subscribe<>(SUBSCRIBER_GOAL_DONE,1024,&SSI::WaypointNavigation::goalDoneCallback,this);
		subscriberRobotPose = nodeHandle.subscribe(SUBSCRIBER_ROBOT_POSE,1024,&SSI::WaypointNavigation::updateRobotPose,this);
		
		publisherStringFeedback = nodeHandle.advertise<String>(PUBLISHER_END_PATH,1);
		publisherCoordinationFeedback = nodeHandle.advertise<String>(PUBLISHER_FEEDBACK_MOTION,1);
		
		Utils::println(string("File to read: ") + pathFilename + string("."),Utils::Yellow);
	}
	
	WaypointNavigation::~WaypointNavigation() {;}
	
	pair<string,WaypointNavigation::Waypoints> WaypointNavigation::emptyWaypointList()
	{
		return make_pair("stop",Waypoints());
	}
	
	void WaypointNavigation::commandLoadCallback(const String::ConstPtr& message)
	{
		stringstream s;
		string command = message->data, filename;
		
		transform(command.begin(),command.end(),command.begin(),::tolower);
		
		s << command;
		s >> command >> filename;
		
		if (command == "reloadfile") loadFilePath(pathDirectory + string("/") + filename);
	}
	
	void WaypointNavigation::commandPathCallback(const String::ConstPtr& message)
	{
		Utils::print("I received this path: ",Utils::White);
		Utils::println(message->data,Utils::Cyan);
		
		/// Checking if the path already exists. If so, I load it otherwise I check if it is valid and then I load it.
		stringstream pathStream(stringstream::in | stringstream::out);
		string pathName;
		bool isPathLoaded;
		
		pathStream << message->data;
		
		if (!pathStream.good())
		{
			Utils::println("Wrong message sent...",Utils::Red);
			
			return;
		}
		
		/// Resetting current path.
		posesQueue.clear();
		nextPoint = posesQueue.end();
		
		pathStream >> pathName;
		
		transform(pathName.begin(),pathName.end(),pathName.begin(),::tolower);
		
		if (pathName == "cancelling") actionClient.cancelAllGoals();
		else
		{
			MapPosesQueue::iterator it = posesQueueMap.find(pathName);
			
			if (it == posesQueueMap.end()) isPathLoaded = initCustomPath(pathStream);
			else isPathLoaded = initStandardPath(it);
			
			if (isPathLoaded)
			{
				Utils::println("New path loaded correctly.\n",Utils::Green);
				
				/// Cancelling old goals.
				actionClient.cancelAllGoals();
				goToNextGoal();
				
				executingPath = true;
			}
			else Utils::println("Failed in loading new path file.\n",Utils::Red);
		}
	}
	
	void WaypointNavigation::goalDoneCallback(const MoveBaseActionResult::ConstPtr&)
	{
		string state;
		
		state = actionClient.getState().toString();
		
		Utils::print("Client response -> ",Utils::White);
		Utils::println(state,Utils::Cyan);
		
		if (state == "SUCCEEDED")
		{
			String feedback;
			stringstream s;
			
			s << "Agent " << agentId << ((posesQueue.size() == 1) ? " doneChasing" : " donePatroling");
			
			feedback.data = s.str();
			publisherCoordinationFeedback.publish(feedback);
			
			goToNextGoal();
		}
		else
		{
			if ((state == "PREEMPTED") && (posesQueue.size() > 0))
			{
				Utils::println("I gotta send the point again.",Utils::Yellow);
				
				/// I send the point again.
				--nextPoint;
				goToNextGoal();
			}
		}
	}
	
	void WaypointNavigation::goToNextGoal()
	{
		MoveBaseGoal goal;
		
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
			
			temp << "/robot_" << agentId << "/map";
			
			goal.target_pose.header.frame_id = temp.str();
			goal.target_pose.header.stamp = Time::now();
			
			/// rand is used to avoid nasty behaviours...
			goal.target_pose.pose.position.x = nextPoint->position.x + ((rand() % 2) * 0.1);
			goal.target_pose.pose.position.y = nextPoint->position.y;
			goal.target_pose.pose.orientation.z = nextPoint->orientation.z;
			goal.target_pose.pose.orientation.w = nextPoint->orientation.w;
			
			++nextPoint;
			actionClient.sendGoal(goal);
		}
		else
		{
			String message;
			
			/// Here the navigation phase is done. There are 2 cases:
			///		- the queue is empty so I received a stop command.
			///		- the queue is not empty but I completed the path. In this case the navigation phase is indeed done.
			if (posesQueue.size() > 0) message.data = PATH_DONE;
			else message.data = PATH_STOP;
			
			executingPath = false;
			
			publisherStringFeedback.publish(message);
			actionClient.cancelAllGoals();
		}
	}
	
	bool WaypointNavigation::init()
	{
		Utils::println("Waiting that move_base's comes up...",Utils::White);
		
		if (!actionClient.waitForServer())
		{
			Utils::println("Action server is not responding...",Utils::Red);
			
			return false;
		}
		
		Utils::println("move_base started!",Utils::Magenta);
		
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
		
		/// Clearing old points' list.
		posesQueueMap.clear();
		
		while (getline(file,line))
		{
			istringstream iss(line);
			
			iss >> path;
			
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
					
					s << "Adding to " << path << " point: (" << p.position.x << "," << p.position.y << "," << p.orientation.z << "," << p.orientation.w
					  << ").";
					
					Utils::println(s.str(),Utils::White);
				}
				
				transform(path.begin(),path.end(),path.begin(),::tolower);
				
				/// Inserting the new path in the queue.
				posesQueueMap.insert(make_pair<string,Waypoints>(path,newWaypoints));
			}
		}
		
		Utils::println("File reading phase done.",Utils::Green);
		
		return true;
	}
	
	void WaypointNavigation::updateRobotPose(const Odometry::ConstPtr& message)
	{
		static int64_t initialStackTime;
		static bool firstTime = true;
		double roll, pitch, yaw;
		
		const Quaternion& q = message->pose.pose.orientation;
		
		tf::Matrix3x3(btQuaternion(q.x,q.y,q.z,q.w)).getRPY(roll,pitch,yaw);
		
		robotPoseX = -message->pose.pose.position.y;
		robotPoseY = message->pose.pose.position.x;
		robotPoseTheta = (M_PI / 2) + yaw;
		
		/// Recovery procedure when the robot's stacking for some unknown reason...
		if (executingPath)
		{
			if ((fabs(robotPoseX - oldRobotPoseX) < 0.05) && (fabs(robotPoseY - oldRobotPoseY) < 0.05) && (fabs(robotPoseTheta - oldRobotPoseTheta) < 0.05))
			{
				if (firstTime)
				{
					initialStackTime = Time::now().toNSec();
					firstTime = false;
				}
				else
				{
					if ((Time::now().toNSec() - initialStackTime) > 2000000000)
					{
						Utils::println("Robot's stacking... I gotta send the point again.",Utils::Yellow);
						
						/// I send the point again.
						--nextPoint;
						goToNextGoal();
						
						firstTime = true;
					}
				}
			}
			else
			{
				oldRobotPoseX = robotPoseX;
				oldRobotPoseY = robotPoseY;
				oldRobotPoseTheta = robotPoseTheta;
				
				firstTime = true;
			}
		}
	}
}

int main (int argc, char** argv)
{
	ros::init(argc,argv,"WaypointNavigation");
	
	SSI::WaypointNavigation waypointNavigation(SSI::ACTION_SERVER_NAME);
	
	if (!waypointNavigation.init())
	{
		SSI::Utils::println("Something went wrong during the initialization phase... Exiting!",SSI::Utils::Red);
		
		return -1;
	}
	
	ros::spin();
	
	return 0;
}
