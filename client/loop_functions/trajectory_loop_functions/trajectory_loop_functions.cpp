#include "trajectory_loop_functions.h"

/****************************************/
/****************************************/

/*
 * To reduce the number of waypoints stored in memory,
 * consider two robot positions distinct if they are
 * at least MIN_DISTANCE away from each other
 * This constant is expressed in meters
 */
static const Real MIN_DISTANCE = 0.05f;
/* Convenience constant to avoid calculating the square root in PostStep() */
static const Real MIN_DISTANCE_SQUARED = MIN_DISTANCE * MIN_DISTANCE;

/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::Init(TConfigurationNode& t_tree) {
    /*
     * Go through all the robots in the environment
     * and create an entry in the waypoint map for each of them
     */
    /* Get the map of all foot-bots from the space */
    CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
    /* Go through them */
    for (CSpace::TMapPerType::iterator it = tFBMap.begin(); it != tFBMap.end();
         ++it) {
        /* Create a pointer to the current foot-bot */
        CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
        /* Create a waypoint vector */
        m_tWaypoints[pcFB] = std::vector<CVector3>();
        /* Add the initial position of the foot-bot */
        m_tWaypoints[pcFB].push_back(
            pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
    }

    TConfigurationNode& portParams = GetNode(t_tree, "port_number");
    GetNodeAttribute(portParams, "value", port_number);

    // Set up logger
    auto console_logger = spdlog::default_logger()->clone("Argos Client");
    spdlog::set_default_logger(console_logger);
}

/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::Reset() {
    /*
     * Clear all the waypoint vectors
     */
    /* Get the map of all foot-bots from the space */
    CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
    /* Go through them */
    for (CSpace::TMapPerType::iterator it = tFBMap.begin(); it != tFBMap.end();
         ++it) {
        /* Create a pointer to the current foot-bot */
        CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
        /* Clear the waypoint vector */
        m_tWaypoints[pcFB].clear();
        /* Add the initial position of the foot-bot */
        m_tWaypoints[pcFB].push_back(
            pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
    }
}

void CTrajectoryLoopFunctions::addMobileVisualization() {
    /* Get the map of all foot-bots from the space */
    CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
    /* Go through them */
    task_stations.clear();
    for (CSpace::TMapPerType::iterator it = tFBMap.begin(); it != tFBMap.end();
         ++it) {
        /* Create a pointer to the current foot-bot */
        CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);

        // Get the controller
        CFootBotDiffusion* pcController = dynamic_cast<CFootBotDiffusion*>(
            &(pcFB->GetControllableEntity().GetController()));

        if (pcController) {
            // Retrieve step count or any other return value
            auto tmp_station = pcController->getCurrStation();
            if (tmp_station.GetZ() != -100) {
                task_stations.push_back(tmp_station);
            }
        }
    }
}
/****************************************/

void CTrajectoryLoopFunctions::PostStep() {
    if (not is_initialized) {
        if (is_port_open("127.0.0.1", port_number)) {
            client = std::make_shared<rpc::client>("127.0.0.1", port_number);
        } else {
            // std::cout << "Failed to connect to server. Retrying..." <<
            // std::endl; std::this_thread::sleep_for(std::chrono::seconds(5));
            // // Wait for 1 second before retrying
            return;
        }
        is_initialized = true;
        spdlog::info("Connected to server at port {}", port_number);
    }
    addMobileVisualization();

    // Get simulation status, return true if the simulation shall end
    this->end_sim = client->call("sim_status").as<bool>();
}

void CTrajectoryLoopFunctions::PostExperiment() {
    // We want to stop the ADG server here, instead of from the controller.
    spdlog::info("PostExperiment called, closing server at port {}",
                 port_number);
    client->async_call("close_server");
    spdlog::info("Closing argos client");
    exit(0);  // Exit the argos simulator after closing the server
}

// Additional custom logic to determine if the experiment is finished
bool CTrajectoryLoopFunctions::IsExperimentFinished() {
    if (this->end_sim) {
        spdlog::info("Simulation is either congested or time step exceeded, "
                     "stopping the simulation.");
    }

    return this->end_sim;
}

void CTrajectoryLoopFunctions::Destroy() {
    m_tWaypoints.clear();
    task_stations.clear();
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CTrajectoryLoopFunctions, "trajectory_loop_functions")
