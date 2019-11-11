#include "rosplan_interface_strategic/RPTacticalControl.h"

/* The implementation of RPTacticalControl.h */
namespace KCL_rosplan {

	/* constructor */
	RPTacticalControl::RPTacticalControl(ros::NodeHandle &nh) {



		clear_tactical_knowledge_client = nh.serviceClient<std_srvs::Empty>("/rosplan_knowledge_base_tactical/clear");
		update_tactical_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/rosplan_knowledge_base_tactical/update");
        import_state_client = nh.serviceClient<rosplan_knowledge_msgs::ImportStateFromFileService>("/rosplan_knowledge_base_tactical/import_state");
        mission_goals_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base_tactical/state/goals");
        mission_propositions_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base_tactical/state/propositions");
        mission_functions_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base_tactical/state/functions");

        current_propositions_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/propositions");
        current_functions_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/functions");
        current_instances_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/rosplan_knowledge_base/state/instances");
        current_tils_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/timed_knowledge");

		// planning interface
//		std::string goalTopic = "/rosplan_interface_strategic_control/get_mission_goals";
		std::string cancTopic = "/rosplan_plan_dispatcher/cancel_dispatch";
		std::string probTopic = "/rosplan_problem_interface/problem_generation_server";
		std::string planTopic = "/rosplan_planner_interface/planning_server";
		std::string parsTopic = "/rosplan_parsing_interface/parse_plan";
		std::string dispTopic = "/rosplan_plan_dispatch/dispatch_plan";

//		nh.getParam("mission_goals_topic", goalTopic);
		nh.getParam("cancel_service_topic", cancTopic);
		nh.getParam("problem_service_topic", probTopic);
		nh.getParam("planning_service_topic", planTopic);
		nh.getParam("parsing_service_topic", parsTopic);
		nh.getParam("dispatch_service_topic", dispTopic);
        nh.getParam("dispatch_service_topic", dispTopic);

//		mission_goals_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(goalTopic);
		cancel_client = nh.serviceClient<std_srvs::Empty>(cancTopic);
		problem_client = nh.serviceClient<std_srvs::Empty>(probTopic);
		planning_client = nh.serviceClient<std_srvs::Empty>(planTopic);
		parsing_client = nh.serviceClient<std_srvs::Empty>(parsTopic);
		dispatch_client = nh.serviceClient<rosplan_dispatch_msgs::DispatchService>(dispTopic);


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

    // retrieve and store initial state information
    void RPTacticalControl::storeCurrentState(){

        propositions.clear();
        functions.clear();
        drone_instances.clear();
        component_instances.clear();
        timed_knowledge.clear();

        // store propositions
        rosplan_knowledge_msgs::GetAttributeService currentPropositionsSrv;
        if (!current_propositions_client.call(currentPropositionsSrv)) {
            ROS_ERROR("KCL: (%s) Failed to call propositions service.", ros::this_node::getName().c_str());
        } else {
            propositions = currentPropositionsSrv.response.attributes;
        }

        // store functions
        rosplan_knowledge_msgs::GetAttributeService currentFunctionsSrv;
        if (!current_functions_client.call(currentFunctionsSrv)) {
            ROS_ERROR("KCL: (%s) Failed to call functions service.", ros::this_node::getName().c_str());
        } else {
            functions = currentFunctionsSrv.response.attributes;
        }

        // store instances
        rosplan_knowledge_msgs::GetInstanceService currentInstancesSrv;
        currentInstancesSrv.request.type_name = "drone";
        if (!current_instances_client.call(currentInstancesSrv)) {
            ROS_ERROR("KCL: (%s) Failed to call instances service.", ros::this_node::getName().c_str());
        } else {
            drone_instances = currentInstancesSrv.response.instances;
        }
        currentInstancesSrv.request.type_name = "component";
        if (!current_instances_client.call(currentInstancesSrv)) {
            ROS_ERROR("KCL: (%s) Failed to call instances service.", ros::this_node::getName().c_str());
        } else {
            component_instances = currentInstancesSrv.response.instances;
        }

        // store tils
        rosplan_knowledge_msgs::GetAttributeService currentTilsSrv;
        if (!current_tils_client.call(currentTilsSrv)) {
            ROS_ERROR("KCL: (%s) Failed to call tils service.", ros::this_node::getName().c_str());
        } else {
            timed_knowledge = currentTilsSrv.response.attributes;
        }

    }

	/**
	 * fetch goals for corresponding mission and update KB
	 */
    bool RPTacticalControl::initState(const std::string &mission, const std::string &mission_type, const std::pair<std::string,std::string> &drones) {

        clear_tactical_knowledge_client.call(empty);

        // fetch execution (runtime) state
        storeCurrentState();

		// import mission state into tactical kb
        std::stringstream ss;
        ss << "/home/nq/ROSPlan/src/rosplan_interface_strategic/common/tactical/problem_" << mission << "_" << mission_type << ".pddl";

        rosplan_knowledge_msgs::ImportStateFromFileService importStateSrv;
        importStateSrv.request.domain_path = "/home/nq/ROSPlan/src/rosplan_interface_strategic/common/droneacharya/droneacharya-domain-all.pddl";
        importStateSrv.request.problem_path = ss.str();
        importStateSrv.request.domain_string_response = false;
        importStateSrv.request. problem_string_response = false;
        import_state_client.call(importStateSrv);

        // fetch mission goals
        rosplan_knowledge_msgs::GetAttributeService missionGoalsSrv;
        if (!mission_goals_client.call(missionGoalsSrv)) {
            ROS_ERROR("KCL: (%s) Failed to call goals service.", ros::this_node::getName().c_str());
            return false;
        } else {
            mission_goals = missionGoalsSrv.response.attributes;
        }

		// fetch completed goals from executionKB, compare them to tacticalKB goals, store positives to problem initial state
        std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit = propositions.begin();
        for (; pit != propositions.end(); pit++) {
            std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator git = mission_goals.begin();
            for (; git != mission_goals.end(); git++) {
                if(pit->attribute_name.compare(git->attribute_name) == 0){
                    updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
                    updateSrv.request.knowledge = *pit;
                    update_tactical_knowledge_client.call(updateSrv);
                }
            }
        }

        //remove duplicate instances
        // configuration inspection perspective capability knowledges drone(think drone removes all other)

		//remove drone propositions
		rosplan_knowledge_msgs::GetAttributeService missionPropositionsSrv;
        if (!mission_propositions_client.call(missionPropositionsSrv)) {
            ROS_ERROR("KCL: (%s) Failed to call goals service.", ros::this_node::getName().c_str());
            return false;
        } else {
            mission_propositions = missionPropositionsSrv.response.attributes;
        }
        pit = mission_propositions.begin();
        for (; pit != mission_propositions.end(); pit++) {
            for(int i = 0; i < pit->values.size(); ++i ){
                if(pit->values[i].key.compare("drone") == 0){
                    updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
                    updateSrv.request.knowledge = *pit;
                    update_tactical_knowledge_client.call(updateSrv);
                }
            }
        }

        //remove drone functions
        rosplan_knowledge_msgs::GetAttributeService missionFunctionsSrv;
        if (!mission_functions_client.call(missionFunctionsSrv)) {
            ROS_ERROR("KCL: (%s) Failed to call goals service.", ros::this_node::getName().c_str());
            return false;
        } else {
            mission_functions = missionFunctionsSrv.response.attributes;
        }
        std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator fit = mission_functions.begin();
        for (; fit != mission_functions.end(); fit++) {
            for(int i = 0; i < fit->values.size(); ++i ){
                if(fit->values[i].key.compare("drone") == 0){
                    updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
                    updateSrv.request.knowledge = *fit;
                    update_tactical_knowledge_client.call(updateSrv);
                }
            }
        }

        // add relevant drone instances and propositions from executionKB to tacticalKB
        pit = propositions.begin();
        for (; pit != propositions.end(); pit++) {
            for(int i = 0; i < pit->values.size(); ++i ){
                if((pit->values[i].key.compare("drone") == 0) && (pit->values[i].value.compare(drones.first) == 0)){
                    updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
                    updateSrv.request.knowledge.knowledge_type = 0;
                    updateSrv.request.knowledge.instance_type = "drone";
                    updateSrv.request.knowledge.instance_name = drones.first;
                    update_tactical_knowledge_client.call(updateSrv);

                    updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
                    updateSrv.request.knowledge = *pit;
                    update_tactical_knowledge_client.call(updateSrv);
                }
                if((pit->values[i].key.compare("drone") == 0) && (pit->values[i].value.compare(drones.second) == 0)){
                    updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
                    updateSrv.request.knowledge.knowledge_type = 0;
                    updateSrv.request.knowledge.instance_type = "drone";
                    updateSrv.request.knowledge.instance_name = drones.second;
                    update_tactical_knowledge_client.call(updateSrv);

                    updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
                    updateSrv.request.knowledge = *pit;
                    update_tactical_knowledge_client.call(updateSrv);
                }
            }
        }

        // add relevant drone functions
        fit = functions.begin();
        for (; fit != functions.end(); fit++) {
            for(int i = 0; i < fit->values.size(); ++i ){
                if((fit->values[i].key.compare("drone") == 0) && (fit->values[i].value.compare(drones.first) == 0)){
                    updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
                    updateSrv.request.knowledge = *fit;
                    update_tactical_knowledge_client.call(updateSrv);
                }
                if((fit->values[i].key.compare("drone") == 0) && (fit->values[i].value.compare(drones.second) == 0)){
                    updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
                    updateSrv.request.knowledge = *fit;
                    update_tactical_knowledge_client.call(updateSrv);
                }
            }
        }




//        rosplan_knowledge_msgs::GetAttributeService gsrv;
//        gsrv.request.predicate_name = mission;
//        if(!mission_goals_client.call(gsrv)) {
//            ROS_ERROR("KCL: (%s) Failed to call Knowledge Base for goals.", ros::this_node::getName().c_str());
//            return false;
//        } else {
//            mission_goals = gsrv.response.attributes;
//        }



//		gsrv.request.predicate_name = mission;
//		if(!mission_goals_client.call(gsrv)) {
//			ROS_ERROR("KCL: (%s) Failed to call Knowledge Base for goals.", ros::this_node::getName().c_str());
//			return false;
//		} else {
//			mission_goals = gsrv.response.attributes;
//		}
//
//		// fetch and store old goals
//		rosplan_knowledge_msgs::GetAttributeService currentGoalSrv;
//		if (!current_goals_client.call(currentGoalSrv)) {
//			ROS_ERROR("KCL: (%s) Failed to call Knowledge Base for goals.", ros::this_node::getName().c_str());
//			return false;
//		} else {
//			old_goals = currentGoalSrv.response.attributes;
//		}
//
//		// clear old goals
//		rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
//		updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
//		updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
//		updateSrv.request.knowledge.attribute_name = "";
//		updateSrv.request.knowledge.values.clear();
//        update_tactical_knowledge_client.call(updateSrv);
//
//		// add mission goal to knowledge base
//		rosplan_knowledge_msgs::KnowledgeUpdateService updateGoalSrv;
//		updateGoalSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
//		updateGoalSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
//		for(int i = 0; i<mission_goals.size(); i++) {
//			updateGoalSrv.request.knowledge = mission_goals[i];
//			if(!update_tactical_knowledge_client.call(updateGoalSrv)) {
//				ROS_INFO("KCL: (%s) failed to update PDDL goal.", ros::this_node::getName().c_str());
//				//restoreGoals();
//				return false;
//			}
//		}
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
            std::size_t drone_t = msg->parameters[i].key.find("drone");
            if(drone_t!=std::string::npos) {
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
