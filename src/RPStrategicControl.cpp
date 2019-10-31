#include "rosplan_interface_strategic/RPStrategicControl.h"
#include <iostream>

/* The implementation of RPStrategicControl.h */
namespace KCL_rosplan {

	/* constructor */
	RPStrategicControl::RPStrategicControl(ros::NodeHandle &nh) {

		node_handle = &nh;

		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/rosplan_knowledge_base/update");
		current_goals_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/goals");
        current_propositions_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/propositions");
        current_functions_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/functions");

		// planning interface
		std::string probTopic = "/rosplan_problem_interface/problem_generation_server";
		std::string planTopic = "/rosplan_planner_interface/planning_server";
		std::string parsTopic = "/rosplan_parsing_interface/parse_plan";


		node_handle->getParam("problem_service_topic", probTopic);
		node_handle->getParam("planning_service_topic", planTopic);
		node_handle->getParam("parsing_service_topic", parsTopic);

		problem_client = nh.serviceClient<std_srvs::Empty>(probTopic);
		planning_client = nh.serviceClient<std_srvs::Empty>(planTopic);
		parsing_client = nh.serviceClient<std_srvs::Empty>(parsTopic);
	}

	void RPStrategicControl::planCallback(const rosplan_dispatch_msgs::EsterelPlan& msg) {
		last_plan = msg;
		new_plan_recieved = true;
	}

	/*----------*/
	/* missions */
	/*----------*/



	// called at the tactical level to get the relevant goals of a particular mission
	/**
	 * get goals for particular mission
	 */
	bool RPStrategicControl::getMissionGoals(rosplan_knowledge_msgs::GetAttributeService::Request &req, rosplan_knowledge_msgs::GetAttributeService::Response &res) {

		std::string mission_name = req.predicate_name;
		if(missions.find(mission_name)!=missions.end()) {
			res.attributes = missions.find(mission_name)->second.goals; // all pddl goals in the mission
		}
	}


    std::pair<int,int> RPStrategicControl::getSites(std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator it){
        std::pair<int,int> temp;
        temp.first = 0;
        temp.second = 0;

        for (int i = 0; i < 7; ++i) {
            std::stringstream site_ss1;
            site_ss1 << "s" << (i*3+1) <<"-";
            std::stringstream site_ss2;
            site_ss2 << "s" << (i*3+2) <<"-";
            std::stringstream site_ss3;
            site_ss3 << "s" << (i*3+3) <<"-";

            for (int j = 0; j < it->values.size(); ++j) {

                std::size_t station1 = it->values[j].value.find(site_ss1.str());
                std::size_t station2 = it->values[j].value.find(site_ss2.str());
                std::size_t station3 = it->values[j].value.find(site_ss3.str());
                if( (station1!=std::string::npos) || (station2!=std::string::npos) || (station3!=std::string::npos)) {
                    if(temp.first == 0){
                        temp.first = i+1;
                    }
                    else{
                        temp.second = i+1;
                    }
                }
            }
        }
        return temp;
	};



	// sort and store pddl goals into desired missions
    std::pair<std::string,std::vector<std::string> > RPStrategicControl::splitIndividualGoals(int site, std::string knowledge){

        std::pair<std::string,std::vector<std::string> > name_and_type;
        std::stringstream ss;
        ss << "site-" << site << "-" << knowledge << "-mission-A";
        if(!missions[ss.str()].deny_goal){
            name_and_type.first = ss.str();
            missions[ss.str()].deny_goal = true;
        }
        else {
            missions[ss.str()].deny_goal = false;
            ss.str("");
            ss << "site-" << site << "-" << knowledge << "-mission-B";
            name_and_type.first = ss.str();
        }
        if(knowledge.compare("image") == 0){
            name_and_type.second.push_back("cm-1");
        }
        if(knowledge.compare("thermal-image") == 0){
            name_and_type.second.push_back("tc-1");
        }
        if(knowledge.compare("signal-measurement") == 0){
            name_and_type.second.push_back("sm-a-1");
            name_and_type.second.push_back("sm-b-1");
        }
        return name_and_type;
	}

	// get mission location component
    std::string RPStrategicControl::getMissionLocation(int site){
        std::string location = "";
        switch(site){
            case 1:
                location = "s1-tower-launchpad";
                break;
            case 2:
                location = "s4-tower-launchpad";
                break;
            case 3:
                location = "s7-tower-launchpad";
                break;
            case 4:
                location = "s10-tower-launchpad";
                break;
            case 5:
                location = "s13-tower-launchpad";
                break;
            case 6:
                location = "s16-tower-launchpad";
                break;
            case 7:
                location = "s19-tower-launchpad";
                break;
        }
        return location;
	}

    // extract information from the initial problem and create missions based on it
	void RPStrategicControl::createMissions(){

        //add mission initial state propositions
        std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator git = goals.begin();
        for(; git!=goals.end(); git++) {

            std::pair<std::string,std::vector<std::string> > name_and_type;
            int site = getSites(git).first;
            if (git->attribute_name.compare("know") == 0) {

                if (git->values[0].value.compare("image") == 0) {
                    name_and_type = splitIndividualGoals(site,"image");
                }
                if (git->values[0].value.compare("thermal-image") == 0) {
                    name_and_type = splitIndividualGoals(site,"thermal-image");
                }
                if (git->values[0].value.compare("signal-measurement") == 0) {
                    name_and_type = splitIndividualGoals(site,"signal-measurement");
                }
            }
            if (git->attribute_name == "know-simultaneous") {
                std::stringstream ss;
                ss << "site-" << site << "-simultaneous-mission";
                name_and_type.first = ss.str();
                name_and_type.second.push_back("im-a-2");
                name_and_type.second.push_back("im-b-2");
                name_and_type.second.push_back("im-c-2");
            }
                missions[name_and_type.first].goals.push_back(*git);
                missions[name_and_type.first].site = getSites(git).first;
                missions[name_and_type.first].location = getMissionLocation(missions[name_and_type.first].site);
                missions[name_and_type.first].types = name_and_type.second;
        }

        // add mission initial state propositions and functionss
        std::map< std::string, mission_details>::iterator mit = missions.begin();
        for(; mit!=missions.end(); mit++) {

            //add propositions
            std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit = propositions.begin();
            for (; pit != propositions.end(); pit++) { ;
                if (getSites(pit).first == mit->second.site) {
                    mit->second.propositions.push_back(*pit);
                }
                if ((pit->attribute_name.compare("inspects") == 0) || (pit->attribute_name.compare("is-available") == 0) || (pit->attribute_name.compare("is-dock") == 0)){
                    mit->second.propositions.push_back(*pit);
                }
            }

            //add functions
            std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator fit = functions.begin();
            for (; fit != functions.end(); fit++) { ;
                if (((getSites(fit).first == mit->second.site) && (getSites(fit).second == mit->second.site)) ||
                    ((getSites(fit).first == mit->second.site) && (getSites(fit).second == 0))) {
                    if ((fit->attribute_name.compare("max-dock") == 0) &&
                        (fit->values[0].value.compare(mit->second.location) == 0)) {
                        fit->function_value = 0;
                    }
                    mit->second.functions.push_back(*fit);
                }
                if ((fit->attribute_name.compare("inspection-duration") == 0) || (fit->attribute_name.compare("capability-consumption") == 0)){
                    mit->second.functions.push_back(*fit);
                }

            }
        }
	}

    void RPStrategicControl::addDronesOffline(std::string mission_type,std::string mission_location){

        //add propositions
        std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit = propositions.begin();
        for(; pit!=propositions.end(); pit++) {

            updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
            for (int i = 0; i < pit->values.size() ; ++i){
                if(mission_type.compare("cm-1") == 0){
                    if(pit->values[i].value.compare("drone1") == 0){
                        if(pit->attribute_name.compare("is-at") == 0){
                            pit->values[1].value = mission_location;
                        }
                        updateSrv.request.knowledge = *pit;
                        update_knowledge_client.call(updateSrv);
                    }
                }
                if(mission_type.compare("tc-1") == 0){
                    if(pit->values[i].value.compare("drone2") == 0){
                        if(pit->attribute_name.compare("is-at") == 0){
                            pit->values[1].value = mission_location;
                        }
                        updateSrv.request.knowledge = *pit;
                        update_knowledge_client.call(updateSrv);
                    }
                }
                if(mission_type.compare("sm-a-1") == 0){
                    if(pit->values[i].value.compare("drone1") == 0){
                        if(pit->attribute_name.compare("is-at") == 0){
                            pit->values[1].value = mission_location;
                        }
                        updateSrv.request.knowledge = *pit;
                        update_knowledge_client.call(updateSrv);
                    }
                }
                if(mission_type.compare("sm-b-1") == 0){
                    if(pit->values[i].value.compare("drone3") == 0){
                        if(pit->attribute_name.compare("is-at") == 0){
                            pit->values[1].value = mission_location;
                        }
                        updateSrv.request.knowledge = *pit;
                        update_knowledge_client.call(updateSrv);
                    }
                }
                if(mission_type.compare("im-a-2") == 0){
                    if(pit->values[i].value.compare("drone1") == 0){
                        if(pit->attribute_name.compare("is-at") == 0){
                            pit->values[1].value = mission_location;
                        }
                        updateSrv.request.knowledge = *pit;
                        update_knowledge_client.call(updateSrv);
                    }
                    if(pit->values[i].value.compare("drone3") == 0){
                        if(pit->attribute_name.compare("is-at") == 0){
                            pit->values[1].value = mission_location;
                        }
                        updateSrv.request.knowledge = *pit;
                        update_knowledge_client.call(updateSrv);
                    }
                }
                if(mission_type.compare("im-b-2") == 0){
                    if(pit->values[i].value.compare("drone1") == 0){
                        if(pit->attribute_name.compare("is-at") == 0){
                            pit->values[1].value = mission_location;
                        }
                        updateSrv.request.knowledge = *pit;
                        update_knowledge_client.call(updateSrv);
                    }
                    if(pit->values[i].value.compare("drone4") == 0){
                        if(pit->attribute_name.compare("is-at") == 0){
                            pit->values[1].value = mission_location;
                        }
                        updateSrv.request.knowledge = *pit;
                        update_knowledge_client.call(updateSrv);
                    }
                }
                if(mission_type.compare("im-a-2") == 0){
                    if(pit->values[i].value.compare("drone3") == 0){
                        if(pit->attribute_name.compare("is-at") == 0){
                            pit->values[1].value = mission_location;
                        }
                        updateSrv.request.knowledge = *pit;
                        update_knowledge_client.call(updateSrv);
                    }
                    if(pit->values[i].value.compare("drone6") == 0){
                        if(pit->attribute_name.compare("is-at") == 0){
                            pit->values[1].value = mission_location;
                        }
                        updateSrv.request.knowledge = *pit;
                        update_knowledge_client.call(updateSrv);
                    }
                }
            }
        }

        //add functions and set charge
        std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator fit = functions.begin();
        for(; fit!=functions.end(); fit++) {
            updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
            for (int i = 0; i < fit->values.size() ; ++i){
                if(mission_type.compare("cm-1") == 0){
                    if(fit->values[i].value.compare("drone1") == 0){
                        if(fit->attribute_name.compare("drone-charge") == 0){
                            fit->function_value = 0;
                        }
                        updateSrv.request.knowledge = *fit;
                        update_knowledge_client.call(updateSrv);

                    }
                }
                if(mission_type.compare("tc-1") == 0){
                    if(fit->values[i].value.compare("drone2") == 0){
                        if(fit->attribute_name.compare("drone-charge") == 0){
                            fit->function_value = 0;
                        }
                        updateSrv.request.knowledge = *fit;
                        update_knowledge_client.call(updateSrv);
                    }
                }
                if(mission_type.compare("sm-a-1") == 0){
                    if(fit->values[i].value.compare("drone1") == 0){
                        if(fit->attribute_name.compare("drone-charge") == 0){
                            fit->function_value = 0;
                        }
                        updateSrv.request.knowledge = *fit;
                        update_knowledge_client.call(updateSrv);
                    }
                }
                if(mission_type.compare("sm-b-1") == 0){
                    if(fit->values[i].value.compare("drone3") == 0){
                        if(fit->attribute_name.compare("drone-charge") == 0){
                            fit->function_value = 0;
                        }
                        updateSrv.request.knowledge = *fit;
                        update_knowledge_client.call(updateSrv);
                    }
                }
                if(mission_type.compare("im-a-2") == 0){
                    if(fit->values[i].value.compare("drone1") == 0){
                        if(fit->attribute_name.compare("drone-charge") == 0){
                            fit->function_value = 0;
                        }
                        updateSrv.request.knowledge = *fit;
                        update_knowledge_client.call(updateSrv);
                    }
                    if(fit->values[i].value.compare("drone3") == 0){
                        if(fit->attribute_name.compare("drone-charge") == 0){
                            fit->function_value = 0;
                        }
                        updateSrv.request.knowledge = *fit;
                        update_knowledge_client.call(updateSrv);
                    }
                }
                if(mission_type.compare("im-b-2") == 0){
                    if(fit->values[i].value.compare("drone1") == 0){
                        if(fit->attribute_name.compare("drone-charge") == 0){
                            fit->function_value = 0;
                        }
                        updateSrv.request.knowledge = *fit;
                        update_knowledge_client.call(updateSrv);
                    }
                    if(fit->values[i].value.compare("drone4") == 0){
                        if(fit->attribute_name.compare("drone-charge") == 0){
                            fit->function_value = 0;
                        }
                        updateSrv.request.knowledge = *fit;
                        update_knowledge_client.call(updateSrv);
                    }
                }
                if(mission_type.compare("im-a-2") == 0){
                    if(fit->values[i].value.compare("drone3") == 0){
                        if(fit->attribute_name.compare("drone-charge") == 0){
                            fit->function_value = 0;
                        }
                        updateSrv.request.knowledge = *fit;
                        update_knowledge_client.call(updateSrv);
                    }
                    if(fit->values[i].value.compare("drone6") == 0){
                        if(fit->attribute_name.compare("drone-charge") == 0){
                            fit->function_value = 0;
                        }
                        updateSrv.request.knowledge = *fit;
                        update_knowledge_client.call(updateSrv);
                    }
                }
            }
        }
    }

    // retrieve and store initial state information
	void RPStrategicControl::storeInitialState(){

        goals.clear();
        propositions.clear();
        functions.clear();

        // store goals
        rosplan_knowledge_msgs::GetAttributeService currentGoalSrv;
        if (!current_goals_client.call(currentGoalSrv)) {
            ROS_ERROR("KCL: (%s) Failed to call goal service.", ros::this_node::getName().c_str());
        } else {
            goals = currentGoalSrv.response.attributes;
        }

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
	}

	void RPStrategicControl::clearInitialState(){
        updateSrv.request.knowledge.attribute_name = "";
        updateSrv.request.knowledge.values.clear();

        // clear old goals from initial problem file
        updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
        updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
        update_knowledge_client.call(updateSrv);

        // clear old prepositions from initial problem file
        updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
        updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
        update_knowledge_client.call(updateSrv);

        // clear old functions from initial problem file
        updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
        updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
        update_knowledge_client.call(updateSrv);
    }

	/**
	 * mission generation service method
	 */
	bool RPStrategicControl::decomposeProblem(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {


		ROS_INFO("KCL: (%s) Decomposing problem by subgoals.", ros::this_node::getName().c_str());

        storeInitialState();
        clearInitialState();
        createMissions();

        std::stringstream ss;




//        std::map< std::string, mission_details>::iterator tmit = missions.begin();
//        for(; tmit!=missions.end(); tmit++) {
//            std::cout << tmit->first << "\n";
//            std::cout << tmit->second.location << "\n";
//
//            std::vector<rosplan_knowledge_msgs::KnowledgeItem> goals = tmit->second.goals;
//            std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator git = goals.begin();
//            for(; git!=goals.end(); git++) {
// //               std::cout << git->values[0].value << " " << git->values[1].value << "\n";
//            }
//        }


        // tactical offline: generate a problem for each subgoal, find a plan, and extract mission details from the found plan

        std::map< std::string, mission_details>::iterator mit = missions.begin();
        for(; mit!=missions.end(); mit++) {

            for(int i = 0; i < mit->second.types.size() ; i++){


                addDronesOffline(mit->second.types[i],mit->second.location);


                //add subgoal pddl goals
                std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator git = mit->second.goals.begin();
                for(; git!=mit->second.goals.end(); git++) {;
                    updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
                    updateSrv.request.knowledge = *git;
                    update_knowledge_client.call(updateSrv);
                }

                //add subgoal initial state propositions
                std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit = mit->second.propositions.begin();
                for(; pit!=mit->second.propositions.end(); pit++) {;
                    updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
                    updateSrv.request.knowledge = *pit;
                    update_knowledge_client.call(updateSrv);
                }

                //add subgoal initial state functions
                std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator fit = mit->second.functions.begin();
                for(; fit!=mit->second.functions.end(); fit++) {;
                    updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
                    updateSrv.request.knowledge = *fit;
                    update_knowledge_client.call(updateSrv);
                }


                // generate problem and plan from the initial problem file state and 1 added goal
                ROS_INFO("KCL: (%s) Generating plan for %s.", ros::this_node::getName().c_str(), ss.str().c_str());
                new_plan_recieved = false;

                std_srvs::Empty empty;
                problem_client.call(empty);
                ros::Duration(1).sleep(); // sleep for a second
                planning_client.call(empty);
                ros::Duration(1).sleep(); // sleep for a second
                parsing_client.call(empty);
                ros::Duration(1).sleep(); // sleep for a second

                while(!new_plan_recieved && ros::ok()) ros::spinOnce();

                // start to compute duration

                //			while(!new_plan_recieved && ros::ok()) ros::spinOnce();
//
//			// compute mission duration by parsing the obtained plan
//			double max_time = 0;
//			std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::iterator nit = last_plan.nodes.begin();
//			for(; nit != last_plan.nodes.end(); nit++) {
//				//problem that this will overshoot time of plan now - however, could be fine as we should do upper estimate
//				double time = nit->action.dispatch_time + nit->action.duration;
//				if(time > max_time) max_time = time;
//				//get the only first non move or undock action
//				if(start_locations.size() <= mission_durations.size() && !(nit->action.name == "goto_waypoint" || nit->action.name == "undock" || nit->action.name == "localise")){
//					//loop through parameters of that action
//					for(std::vector<diagnostic_msgs::KeyValue>::iterator i = nit->action.parameters.begin(); i != nit->action.parameters.end(); ++i){
//						if(i->key == "wp"){
//							//first one that is a waypoint parameter is where the mission has to start (at_mission)
//							start_locations.push_back(*i);
//						}
//					}
//				}
//			}
//
//			//get the end points
//			end_points.push_back(getEndPoint(last_plan.nodes));
//
//			mission_durations.push_back(max_time);
//			missions[ss.str()];
//			missions[ss.str()].push_back(*git);
//
                clearInitialState();

            }



        }
//


//		// add new mission goals and other info to the strategic problem
//		ROS_INFO("KCL: (%s) Adding new mission goals.", ros::this_node::getName().c_str());
//		for(int i=0; i<mission_durations.size(); i++) {
//
//			ss.str("");
//			ss << "mission_" << i;
//
//			// mission instance
//			updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
//			updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
//			updateSrv.request.knowledge.instance_type = "mission";
//			updateSrv.request.knowledge.instance_name = ss.str();
//			updateSrv.request.knowledge.values.clear();
//			update_knowledge_client.call(updateSrv);
//
//			// mission duration
//			updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
//			updateSrv.request.knowledge.attribute_name = "mission_duration";
//			updateSrv.request.knowledge.values.clear();
//			diagnostic_msgs::KeyValue pair_mission;
//			pair_mission.key = "m";
//			pair_mission.value = ss.str();
//			updateSrv.request.knowledge.values.push_back(pair_mission);
//			updateSrv.request.knowledge.function_value = mission_durations[i];
//			update_knowledge_client.call(updateSrv);
//
//			// mission goal
//			updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
//			updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
//			updateSrv.request.knowledge.attribute_name = "mission_complete";
//			update_knowledge_client.call(updateSrv);
//
//			// mission start
//			updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
//			updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
//			updateSrv.request.knowledge.instance_type = "";
//			updateSrv.request.knowledge.instance_name = "";
//			updateSrv.request.knowledge.function_value = 0;
//			updateSrv.request.knowledge.attribute_name = "mission_at";
//			updateSrv.request.knowledge.values.clear();
//			diagnostic_msgs::KeyValue pair_at_mission;
//			pair_at_mission.key = "m";
//			pair_at_mission.value = ss.str();
//			updateSrv.request.knowledge.values.push_back(pair_at_mission);
//			//am changing it now that the start of a mission is at the end of the last one - to change back can delete the if else
//			//decided to keep it the normal way as explained in notes
//			//if(i == 0){
////				updateSrv.request.knowledge.values.push_back(start_locations[i]);
//			/*}
//			else{
//				updateSrv.request.knowledge.values.push_back(end_points[i - 1]);
//			}*/
//			update_knowledge_client.call(updateSrv);
//
////			//end_point
////			updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
////			updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
////			updateSrv.request.knowledge.instance_type = "";
////			updateSrv.request.knowledge.instance_name = "";
////			updateSrv.request.knowledge.function_value = 0;
////			updateSrv.request.knowledge.attribute_name = "end_point";
////			updateSrv.request.knowledge.values.clear();
////			diagnostic_msgs::KeyValue pair_end_point;
////			pair_end_point.key = "m";
////			pair_end_point.value = ss.str();
////			updateSrv.request.knowledge.values.push_back(pair_end_point);
////			//updateSrv.request.knowledge.values.push_back(end_points[i]);
////			update_knowledge_client.call(updateSrv);
//		}
//	}
//
//	// method for getting location endpoints
//	diagnostic_msgs::KeyValue RPStrategicControl::getEndPoint(std::vector<rosplan_dispatch_msgs::EsterelPlanNode> & nodes) const{
//		std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::reverse_iterator nrit = nodes.rbegin();
//		for(; nrit != nodes.rend(); ++nrit){
//			if(!(nrit->action.name == "goto_waypoint" || nrit->action.name == "dock")){
//				for(std::vector<diagnostic_msgs::KeyValue>::iterator i = nrit->action.parameters.begin(); i != nrit->action.parameters.end(); ++i){
//					if(i->key == "wp"){
//						return *i;
//					}
//				}
//			}
//		}
//		return nodes[nodes.size() - 1].action.parameters[0];
//	}
//
//	//going to change when I have the updated edge durations
//	int RPStrategicControl::getMinTime(rosplan_dispatch_msgs::EsterelPlan& plan) const{
//		//create graph layout to help with the algorithm - max 10000 so that non connected edges wont be taken into account
//		std::vector<std::vector<int> > graph(plan.nodes.size(), std::vector<int>(plan.nodes.size(), 10000));
//		std::vector<std::vector<int> > newGraph(plan.nodes.size(), std::vector<int>(plan.nodes.size(), 1000));
//		//loop around all nodes
//		std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::iterator nit = plan.nodes.begin();
//		for(int i = 0; i < plan.nodes.size(); ++i){
//			rosplan_dispatch_msgs::EsterelPlanNode* nit = &plan.nodes[i];
//			//the distance from a vertex to itself is 0
//			graph[i][i] = 0;
//			//for each node loop around all incoming edges
//			std::vector<int>::iterator eit = nit->edges_in.begin();
//			for(; eit != nit->edges_in.end(); ++eit){
//				//get the incoming edge from the plan
//				rosplan_dispatch_msgs::EsterelPlanEdge tempEdge = plan.edges[*eit];
//				//use the edges source to get the node that the edge starts at - assuming edges can only come from one node
//				rosplan_dispatch_msgs::EsterelPlanNode tempNode = plan.nodes[tempEdge.source_ids[0]];
//				//the distance between the node the edge starts, at and the node we are looking at: is it's duration
//				graph[tempEdge.source_ids[0]][i] = tempNode.action.duration;
//			}
//		}
//		for(int k = 0; k < graph.size(); ++k){
//			for(int i = 0; i < graph.size(); ++i){
//				for(int j = 0; j < graph.size(); ++j){
//					if(graph[i][k] + graph[k][j] < newGraph[i][j]){
//						newGraph[i][j] = graph[i][k] + graph[k][j];
//					}
//				}
//			}
//		}
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_strategic_control");
		ros::NodeHandle nh("~");

		// params
		std::string lps_topic;
		nh.param("complete_plan_topic", lps_topic, std::string("/rosplan_plan_dispatcher/dispatch_plan"));

		// create bidder
		KCL_rosplan::RPStrategicControl rpsc(nh);

		// listen
		ros::ServiceServer service1 = nh.advertiseService("decompose_problem", &KCL_rosplan::RPStrategicControl::decomposeProblem, &rpsc);
		ros::ServiceServer service2 = nh.advertiseService("get_mission_goals", &KCL_rosplan::RPStrategicControl::getMissionGoals, &rpsc);

		ros::Subscriber lps = nh.subscribe(lps_topic, 100, &KCL_rosplan::RPStrategicControl::planCallback, &rpsc);

		ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());

		ros::spin();

		return 0;
	}
