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
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current foot-bot */
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      /* Create a waypoint vector */
      m_tWaypoints[pcFB] = std::vector<CVector3>();
      /* Add the initial position of the foot-bot */
      m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
   }

//   task_goals.emplace_back(-3.0, -0.0, 0.0);
//   task_goals.emplace_back(-0.0, -3.0, 0.0);
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
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current foot-bot */
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
      /* Clear the waypoint vector */
      m_tWaypoints[pcFB].clear();
      /* Add the initial position of the foot-bot */
      m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
   }
}

/****************************************/
/****************************************/

void CTrajectoryLoopFunctions::PostStep() {
   /* Get the map of all foot-bots from the space */
   CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
   /* Go through them */
   task_pods.clear();
   task_stations.clear();
   for(CSpace::TMapPerType::iterator it = tFBMap.begin();
       it != tFBMap.end();
       ++it) {
      /* Create a pointer to the current foot-bot */
      CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);

      // /* Add the current position of the foot-bot if it's sufficiently far from the last */
      // if(SquareDistance(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position,
      //                   m_tWaypoints[pcFB].back()) > MIN_DISTANCE_SQUARED) {
      //    m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
      // }

      // Get the controller
      CFootBotDiffusion* pcController = dynamic_cast<CFootBotDiffusion*>(
          &(pcFB->GetControllableEntity().GetController())
      );

      if (pcController) {
         // Retrieve step count or any other return value
         // LOG << "Step Count: " << pcController->getCurrGoal() << std::endl;
         auto tmp_goal = pcController->getCurrPod();
         if (tmp_goal.GetZ() != -100) {
            task_pods.push_back(tmp_goal);
         }
         auto tmp_station = pcController->getCurrStation();
         if (tmp_station.GetZ() != -100) {
            task_stations.push_back(tmp_station);
         }
      }
   }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CTrajectoryLoopFunctions, "trajectory_loop_functions")
