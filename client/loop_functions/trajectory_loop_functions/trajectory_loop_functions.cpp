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

void CTrajectoryLoopFunctions::Init(TConfigurationNode &t_tree) {
    // Set up logger
    auto console_logger = spdlog::default_logger()->clone("Argos Client");
    spdlog::set_default_logger(console_logger);

    /*
     * Go through all the robots in the environment
     * and create an entry in the waypoint map for each of them
     */
    /* Get the map of all foot-bots from the space */
    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("foot-bot");
    /* Go through them */
    for (CSpace::TMapPerType::iterator it = tFBMap.begin(); it != tFBMap.end();
         ++it) {
        /* Create a pointer to the current foot-bot */
        CFootBotEntity *pcFB = any_cast<CFootBotEntity *>(it->second);
        /* Create a waypoint vector */
        m_tWaypoints[pcFB] = vector<CVector3>();
        /* Add the initial position of the foot-bot */
        m_tWaypoints[pcFB].push_back(
            pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
    }

    TConfigurationNode &portParams = GetNode(t_tree, "port_number");
    GetNodeAttribute(portParams, "value", port_number);

    while (!is_port_open("127.0.0.1", port_number)) {
        spdlog::info("Waiting for server to open at port {}...", port_number);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    this->client = make_shared<rpc::client>("127.0.0.1", port_number);
    this->is_initialized = true;
    spdlog::info("Loop func: Connected to server at port {}", port_number);
}

/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::Reset() {
    /*
     * Clear all the waypoint vectors
     */
    /* Get the map of all foot-bots from the space */
    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("foot-bot");
    /* Go through them */
    for (CSpace::TMapPerType::iterator it = tFBMap.begin(); it != tFBMap.end();
         ++it) {
        /* Create a pointer to the current foot-bot */
        CFootBotEntity *pcFB = any_cast<CFootBotEntity *>(it->second);
        /* Clear the waypoint vector */
        m_tWaypoints[pcFB].clear();
        /* Add the initial position of the foot-bot */
        m_tWaypoints[pcFB].push_back(
            pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
    }
}

void CTrajectoryLoopFunctions::addMobileVisualization() {
    /* Get the map of all foot-bots from the space */
    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("foot-bot");
    /* Go through them */
    task_stations.clear();
    for (CSpace::TMapPerType::iterator it = tFBMap.begin(); it != tFBMap.end();
         ++it) {
        /* Create a pointer to the current foot-bot */
        CFootBotEntity *pcFB = any_cast<CFootBotEntity *>(it->second);

        // Get the controller
        CFootBotDiffusion *pcController = dynamic_cast<CFootBotDiffusion *>(
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

void CTrajectoryLoopFunctions::PreStep() {
    // Free simulation if necessary
    client->call("freeze_simulation_if_necessary");

    // Loop until the simulation is defrozen
    while (client->call("is_simulation_frozen").as<bool>()) {
        // spdlog::info("Simulation is frozen, waiting to defrost...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void CTrajectoryLoopFunctions::PostStep() {
    if (not is_initialized) {
        spdlog::error("CTrajectoryLoopFunctions::PostStep: LoopFunctions is "
                      "not initialized");
        exit(1);
    }
    addMobileVisualization();

    // Invoke the server to record per tick stats. We do it here because
    // PostStep function is invoked after each tick.
    client->call("record_stats_per_tick");

    client->call("update_sim_step");

    // Get simulation status, return true if the simulation shall end
    this->end_sim = client->call("sim_status").as<bool>();

    // TODO: Add a cmd to capture visualization frames
    // static int frame_id = 0;
    // std::stringstream filename_s;
    // filename_s << "frames/frame_" << std::setw(5) << std::setfill('0')
    //            << frame_id++ << ".png";
    // std::string filename = filename_s.str();
    // argos::CVisualization &cVisualization = GetSimulator().GetVisualization();
    // // Ensure the visualization is of the expected Qt OpenGL type
    // auto *pcQtRender = dynamic_cast<argos::CQTOpenGLRender *>(&cVisualization);
    // if (pcQtRender) {
    //     // Access the main window
    //     CQTOpenGLMainWindow &pcMainWindow = pcQtRender->GetMainWindow();

    //     // Capture the current window as a pixmap
    //     QPixmap pixmap = pcMainWindow.grab();

    //     // Save the pixmap to the specified file
    //     if (!pixmap.save(QString::fromStdString(filename))) {
    //         spdlog::error("Failed to save the visualization snapshot to {}",
    //                       filename);
    //     }
    // } else {
    //     spdlog::error("Qt OpenGL Visualization is not available.");
    // }
}

void CTrajectoryLoopFunctions::PostExperiment() {
    // We want to stop the ADG server here, instead of from the controller.
    spdlog::info("PostExperiment called, closing server at port {}",
                 port_number);
    client->async_call("close_server");
    spdlog::info("Closing argos client");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
