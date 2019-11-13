#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <unistd.h>

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GetMetricService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_dispatch_msgs/EsterelPlan.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ProblemService.h"
#include "rosplan_dispatch_msgs/PlanningService.h"

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#ifndef KCL_strategic_control
#define KCL_strategic_control

/**
 * This file defines the RPStrategicControl class.
 * RPStrategicControl is used to decompose the goal into subtasks and generate new mission objects.
 * Plans are generated to compute duration and resource use of tasks.
 */
namespace KCL_rosplan {

	class RPStrategicControl
	{

	private:

		ros::NodeHandle* node_handle;

        std_srvs::Empty empty;

		/* rosplan knowledge interface */
		ros::ServiceClient update_tactical_knowledge_client;
        ros::ServiceClient update_strategic_knowledge_client;
        ros::ServiceClient update_array_tactical_knowledge_client;
        ros::ServiceClient update_array_strategic_knowledge_client;
        ros::ServiceClient update_array_knowledge_client;
        ros::ServiceClient clear_tactical_knowledge_client;
        ros::ServiceClient clear_strategic_knowledge_client;
        ros::ServiceClient current_instances_client;
		ros::ServiceClient current_goals_client;
		ros::ServiceClient current_propositions_client;
        ros::ServiceClient current_functions_client;
        ros::ServiceClient current_knowledge_client;
        ros::ServiceClient current_constants_client;
        ros::ServiceClient current_metric_client;

        ros::ServiceClient current_tils_client;

		std::vector<rosplan_knowledge_msgs::KnowledgeItem> goals;
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> propositions;
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> functions;
        std::vector<std::string> drone_instances;
        std::vector<std::string> component_instances;
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> timed_knowledge;
        rosplan_knowledge_msgs::KnowledgeItem metric;


        rosplan_knowledge_msgs::KnowledgeItem item;
        rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
        rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updateSrvArray;


        struct mission_details{
            bool deny_goal;
            std::vector<rosplan_knowledge_msgs::KnowledgeItem> goals;
            std::vector<rosplan_knowledge_msgs::KnowledgeItem> propositions;
            std::vector<rosplan_knowledge_msgs::KnowledgeItem> functions;
            std::vector<rosplan_knowledge_msgs::KnowledgeItem> instances;
            std::vector<rosplan_knowledge_msgs::KnowledgeItem> timed_knowledge;
            int site;
            int station;
            std::string location;
            std::vector<std::string> types;
            std::vector<double> durations;
            std::map<std::string,std::vector<rosplan_knowledge_msgs::KnowledgeItem> > type_goals;

        };
        
        //name, goals, location, types, durations for each type, consumption for each type
		std::map< std::string, mission_details > missions;

        /* planning interface */
		ros::ServiceClient problem_client;
        ros::ServiceClient problem_client_params;
		ros::ServiceClient planning_client;
        ros::ServiceClient planning_client_params;
		ros::ServiceClient parsing_client;



		rosplan_dispatch_msgs::EsterelPlan last_plan;
		bool new_plan_recieved;

		int getMinTime(rosplan_dispatch_msgs::EsterelPlan& plan) const;

        std::pair<int,int> getSites(std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator);
        std::pair<int,int> getStations(std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator);
        std::pair<std::string,std::vector<std::string> > splitIndividualGoals(int, int, std::string);
        std::string getMissionStrategicLocation(int);
        std::string getMissionTacticalLocation(int);
        void createMissions();
        void storeInitialState();
        void addDronesOffline(std::string, std::string, std::string);
        void addInstances(std::string, int);

            public:

		/* constructor */
		RPStrategicControl(ros::NodeHandle &nh);

		/* plan topic callback */
		void planCallback(const rosplan_dispatch_msgs::EsterelPlan& msg);

		/* problem decomposition service method */
		bool decomposeProblem(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

		// create random TILs that will cause plan failure
		bool createWeatherDisturbance(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool disableRandomDrone(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	};
}
#endif
