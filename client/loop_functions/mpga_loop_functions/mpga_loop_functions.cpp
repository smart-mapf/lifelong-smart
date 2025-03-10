#include "mpga_loop_functions.h"

/****************************************/
/****************************************/

CMPGALoopFunctions::CMPGALoopFunctions() :
   m_unTrial(0) {
   task_goals.emplace_back(-3.0, -0.0, 0.0);
   task_goals.emplace_back(-0.0, -3.0, 0.0);
}

/****************************************/
/****************************************/

UInt32 CMPGALoopFunctions::GetTrial() const {
   return m_unTrial;
}

/****************************************/
/****************************************/

void CMPGALoopFunctions::SetTrial(UInt32 un_trial) {
   m_unTrial = un_trial;
}

/****************************************/
/****************************************/

// void CMPGALoopFunctions::PostStep() {
//    /* Get the map of all foot-bots from the space */
//    CSpace::TMapPerType& tFBMap = GetSpace().GetEntitiesByType("foot-bot");
//    /* Go through them */
//    for(CSpace::TMapPerType::iterator it = tFBMap.begin();
//        it != tFBMap.end();
//        ++it) {
//       /* Create a pointer to the current foot-bot */
//       CFootBotEntity* pcFB = any_cast<CFootBotEntity*>(it->second);
//       /* Add the current position of the foot-bot if it's sufficiently far from the last */
//       if(SquareDistance(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position,
//                         m_tWaypoints[pcFB].back()) > MIN_DISTANCE_SQUARED) {
//          m_tWaypoints[pcFB].push_back(pcFB->GetEmbodiedEntity().GetOriginAnchor().Position);
//                         }
//        }
// }