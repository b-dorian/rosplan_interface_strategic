#include "rosplan_interface_strategic/RPTacticalControl.h"

/* The implementation of RPTacticalControl.h */
namespace KCL_rosplan {

	/* constructor */
	RPTacticalControl::RPTacticalControl(ros::NodeHandle &nh) {

//		current_goals_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/goals");
		local_update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/rosplan_knowledge_base_tactical/update");
        import_state_client = nh.serviceClient<rosplan_knowledge_msgs::ImportStateFromFileService>("/rosplan_knowledge_base_tactical/import_state");

		// planning interface
//		std::string goalTopic = "/rosplan_interface_strategic_control/get_mission_goals";
		std::string cancTopic = "/rosplan_plan_dispatcher/cancel_dispatch";
		std::string probTopic = "/rosplan_problem_interface/problem_generation_server";
		std::string planTopic = "/rosplan_planner_interface/planning_server";
		std::string parsTopic = "/rosplan_parsing_interface/parse_plan";
		std::string dispTopic = "/rosplan_plan_dispatch/dispatch_plan";

        std::string probTopicParams = "/rosplan_problem_interface/problem_generation_server_params";

//		nh.getParam("mission_goals_topic", goalTopic);
		nh.getParam("cancel_service_topic", cancTopic);
		nh.getParam("problem_service_topic", probTopic);
		nh.getParam("planning_service_topic", planTopic);
		nh.getParam("parsing_service_topic", parsTopic);
		nh.getParam("dispatch_service_topic", dispTopic);
        nh.getParam("dispatch_service_topic", dispTopic);

        nh.getParam("problem_service_topic_params", probTopicParams);

//		mission_goals_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(goalTopic);
		cancel_client = nh.serviceClient<std_srvs::Empty>(cancTopic);
		problem_client = nh.serviceClient<std_srvs::Empty>(probTopic);
		planning_client = nh.serviceClient<std_srvs::Empty>(planTopic);
		parsing_client = nh.serviceClient<std_srvs::Empty>(parsTopic);
		dispatch_client = nh.serviceClient<rosplan_dispatch_msgs::DispatchService>(dispTopic);

        problem_client_params = nh.serviceClient<rosplan_dispatch_msgs::ProblemService>(probTopicParams);

	}

//	/**
//	 * remove mission goals from KB and restore saved goals
//	 */
//	void RPTacticalControl::restoreGoals() {
//
//		// remove mission goal from KB
//		rosplan_knowledge_msgs::KnowledgeUpdateService updateGoalSrv;
//		updateGoalSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
//		for(int i = 0; i<mission_goals.size(); i++) {
//			updateGoalSrv.request.knowledge = mission_goals[i];
//			if(!local_update_knowledge_client.call(updateGoalSrv)) {
//				ROS_INFO("KCL: (%s) failed to update PDDL goal.", ros::this_node::getName().c_str());
//			}
//		}
//
//		// add old goal to knowledge base
//		updateGoalSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
//		updateGoalSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
//		for(int i = 0; i<old_goals.size(); i++) {
//			updateGoalSrv.request.knowledge = old_goals[i];
//			if(!local_update_knowledge_client.call(updateGoalSrv)) {
//				ROS_INFO("KCL: (%s) failed to update PDDL goal.", ros::this_node::getName().c_str());
//			}
//		}
//	}

	/**
	 * fetch goals for corresponding mission and update KB
	 */
    bool RPTacticalControl::initState(const std::string &mission, const std::string &mission_type, const std::pair<std::string,std::string> &drones) {

		mission_goals.clear();
		old_goals.clear();

		// fetch state
        std::stringstream ss;
        ss << "/home/nq/ROSPlan/src/rosplan_interface_strategic/common/tactical/problem_" << mission << mission_type << ".pddl";

        rosplan_knowledge_msgs::ImportStateFromFileService importStateSrv;
        importStateSrv.request.domain_path = "/home/nq/ROSPlan/src/rosplan_interface_strategic/common/droneacharya/droneacharya-domain-all.pddl";
        importStateSrv.request.problem_path = ss.str();
        importStateSrv.request.domain_string_response = false;
        importStateSrv.request. problem_string_response = false;
        import_state_client.call(importStateSrv);

		// fetch completed goals from executionKB, compare them to tacticalKB goals, store positives to problem initial state
        rosplan_knowledge_msgs::GetAttributeService gsrv;
        gsrv.request.predicate_name = mission;
        if(!mission_goals_client.call(gsrv)) {
            ROS_ERROR("KCL: (%s) Failed to call Knowledge Base for goals.", ros::this_node::getName().c_str());
            return false;
        } else {
            mission_goals = gsrv.response.attributes;
        }

		// remove all drones propositions and functions from tacticalKB

		// add relevant drone info from executionKB to tacticalKB








		gsrv.request.predicate_name = mission;
		if(!mission_goals_client.call(gsrv)) {
			ROS_ERROR("KCL: (%s) Failed to call Knowledge Base for goals.", ros::this_node::getName().c_str());
			return false;
		} else {
			mission_goals = gsrv.response.attributes;
		}

		// fetch and store old goals
		rosplan_knowledge_msgs::GetAttributeService currentGoalSrv;
		if (!current_goals_client.call(currentGoalSrv)) {
			ROS_ERROR("KCL: (%s) Failed to call Knowledge Base for goals.", ros::this_node::getName().c_str());
			return false;
		} else {
			old_goals = currentGoalSrv.response.attributes;
		}

		// clear old goals
		rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		updateSrv.request.knowledge.attribute_name = "";
		updateSrv.request.knowledge.values.clear();
		local_update_knowledge_client.call(updateSrv);

		// add mission goal to knowledge base
		rosplan_knowledge_msgs::KnowledgeUpdateService updateGoalSrv;
		updateGoalSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		updateGoalSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		for(int i = 0; i<mission_goals.size(); i++) {
			updateGoalSrv.request.knowledge = mission_goals[i];
			if(!local_update_knowledge_client.call(updateGoalSrv)) {
				ROS_INFO("KCL: (%s) failed to update PDDL goal.", ros::this_node::getName().c_str());
				//restoreGoals();
				return false;
			}
		}
		return true;
	}


	/* action dispatch callback */
	bool RPTacticalControl::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// get mission ID from action dispatch complete_mission_xx (?mission - mission ?drone1 (and maybe ?drone2) - drone ?station - component)
		std::string mission;
		bool found_mission = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("mission")) {
				mission = msg->parameters[i].value;
				found_mission = true;
			}
		}
		if(!found_mission) {
			ROS_INFO("KCL: (%s) aborting action dispatch; PDDL action missing required parameter ?mission", params.name.c_str());
			return false;
		}

        // get mission type
        std::string mission_type;
        std::size_t cm_1 = params.name.find("cm_1");if(cm_1!=std::string::npos)mission_type = "cm-1";
        std::size_t tc_1 = params.name.find("tc_1");if(tc_1!=std::string::npos)mission_type = "tc-1";
        std::size_t sm_a_1 = params.name.find("sm_a_1");if(sm_a_1!=std::string::npos)mission_type = "sm-a-1";
        std::size_t sm_b_1 = params.name.find("sm_b_1");if(sm_b_1!=std::string::npos)mission_type = "sm-b-1";
        std::size_t im_a_2 = params.name.find("im_a_2");if(im_a_2!=std::string::npos)mission_type = "im-a-2";
        std::size_t im_b_2 = params.name.find("im_b_2");if(im_b_2!=std::string::npos)mission_type = "im-b-2";
        std::size_t im_c_2 = params.name.find("im_c_2");if(im_c_2!=std::string::npos)mission_type = "im-c-2";


        // get drone(s) ID from action dispatch complete_mission (?mission - mission ?drone1 (and maybe ?drone2) - drone ?station - component)
        std::pair<std::string,std::string> drones;
		drones.first = "";
		drones.second = "";
        bool drone_found = false;
        for(size_t i=0; i<msg->parameters.size(); i++) {
            if(0==msg->parameters[i].key.compare("drone")) {
                if(drone_found == false){
                    drones.first = msg->parameters[i].value;
                    drone_found = true;
                }
                else{
                    drones.second = msg->parameters[i].value;
                }

            }
        }
        if(!drone_found) {
            ROS_INFO("KCL: (%s) aborting action dispatch; PDDL action missing required parameter ?drone", params.name.c_str());
            return false;
        }




		if(!initState(mission,mission_type,drones)) return false;


		// generate problem and plan
		ROS_INFO("KCL: (%s) Sending to planning system.", ros::this_node::getName().c_str());

		std_srvs::Empty empty;
		rosplan_dispatch_msgs::DispatchService dispatch;
		cancel_client.call(empty);
		ros::Duration(1).sleep(); // sleep for a second
		problem_client.call(empty);
		ros::Duration(1).sleep(); // sleep for a second

		// send to planner
		if(planning_client.call(empty)) {
			ros::Duration(1).sleep(); // sleep for a second
			// parse planner output
			parsing_client.call(empty);
			ros::Duration(1).sleep(); // sleep for a second

			// dispatch tactical plan
			bool dispatch_success = dispatch_client.call(dispatch);

//			restoreGoals();
			return dispatch_success;
		}

//		restoreGoals();
		return false;
	}
} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

	ros::init(argc, argv, "rosplan_interface_tactical_control");
	ros::NodeHandle nh("~");

	// create PDDL action subscriber
	KCL_rosplan::RPTacticalControl rptc(nh);

	rptc.runActionInterface();

	return 0;
}
