#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <unistd.h>

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GetMetricService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_dispatch_msgs/EsterelPlan.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ProblemService.h"

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

		/* rosplan knowledge interface */
		ros::ServiceClient update_knowledge_client;
        ros::ServiceClient clear_knowledge_client;
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

        rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;


        struct drone{
            int config;
            int velocity;
            double charge;
            double max_charge;
            bool camera;
            bool thermal_camera;
            bool signal_measurer;
            std::string is_at;
            std::string is_at_component;
        };

        std::vector<drone> drones;

        struct type{

        };

        struct mission_details{
            bool deny_goal;
            std::vector<rosplan_knowledge_msgs::KnowledgeItem> goals;
            std::vector<rosplan_knowledge_msgs::KnowledgeItem> propositions;
            std::vector<rosplan_knowledge_msgs::KnowledgeItem> functions;
            std::vector<rosplan_knowledge_msgs::KnowledgeItem> instances;
            std::vector<rosplan_knowledge_msgs::KnowledgeItem> timed_knowledge;
            int site;
            std::string location;
            std::vector<std::string> types;
            std::vector<double> durations;
            std::map<std::string,std::vector<rosplan_knowledge_msgs::KnowledgeItem> > type_goals;
            std::map<std::string,drone> type_drones;

        };
        
        //name, goals, location, types, durations for each type, consumption for each type
		std::map< std::string, mission_details > missions;

        /* planning interface */
		ros::ServiceClient problem_client;
		ros::ServiceClient planning_client;
		ros::ServiceClient parsing_client;

        ros::ServiceClient problem_client_params;

		rosplan_dispatch_msgs::EsterelPlan last_plan;
		bool new_plan_recieved;
		diagnostic_msgs::KeyValue getEndPoint(std::vector<rosplan_dispatch_msgs::EsterelPlanNode> & node) const;
		int getMinTime(rosplan_dispatch_msgs::EsterelPlan& plan) const;
        std::pair<int,int> getSites(std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator);
        std::pair<std::string,std::vector<std::string> > splitIndividualGoals(int, std::string);
        std::string getMissionLocation(int);
        void createMissions();
        void storeInitialState();
        //void clearInitialState();
        void addDronesOffline(std::string, std::string, std::string);
        void addInstances(std::string, int);

            public:

		/* constructor */
		RPStrategicControl(ros::NodeHandle &nh);

		/* plan topic callback */
		void planCallback(const rosplan_dispatch_msgs::EsterelPlan& msg);

		/* problem decomposition service method */
		bool decomposeProblem(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool getMissionGoals(rosplan_knowledge_msgs::GetAttributeService::Request &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);
	};
}
#endif
