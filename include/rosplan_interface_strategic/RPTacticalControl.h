#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <tf/transform_listener.h>

#include "rosplan_action_interface/RPActionInterface.h"

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/DispatchService.h"
#include "rosplan_dispatch_msgs/ProblemService.h"

#include "rosplan_knowledge_msgs/ImportStateFromFileService.h"

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#ifndef KCL_tactical_control
#define KCL_tactical_control

/**
 * This file defines the RPTacticalControl class.
 * RPTacticalControl encapsulates as an action:
 * problem generation, planning, and dispatch.
 */
namespace KCL_rosplan {

    class RPTacticalControl: public RPActionInterface
    {

    private:

        ros::ServiceClient cancel_client;
        ros::ServiceClient problem_client;
        ros::ServiceClient planning_client;
        ros::ServiceClient parsing_client;
        ros::ServiceClient dispatch_client;

        //tactical kb clients
        ros::ServiceClient clear_tactical_knowledge_client;
        ros::ServiceClient import_state_client;
        ros::ServiceClient update_tactical_knowledge_client;
        ros::ServiceClient mission_goals_client;
        ros::ServiceClient mission_propositions_client;
        ros::ServiceClient mission_functions_client;

        //execution (runtime) kb clients
        ros::ServiceClient current_propositions_client;
        ros::ServiceClient current_functions_client;
        ros::ServiceClient current_instances_client;
        ros::ServiceClient current_tils_client;

        //execution (runtime)
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> propositions;
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> functions;
        std::vector<std::string> drone_instances;
        std::vector<std::string> component_instances;
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> timed_knowledge;

        //tactical
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> mission_goals;
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> mission_propositions;
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> mission_functions;


        rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
        std_srvs::Empty empty;

    public:

        /* constructor */
        RPTacticalControl(ros::NodeHandle &nh);

        /* listen to and process action_dispatch topic */
        void storeCurrentState();
        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
        bool initState(const std::string &mission, const std::string &mission_type, const std::pair<std::string,std::string> &drones);

    };
}
#endif