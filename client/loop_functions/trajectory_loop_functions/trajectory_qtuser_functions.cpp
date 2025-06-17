#include "trajectory_qtuser_functions.h"
#include "trajectory_loop_functions.h"

/****************************************/
/****************************************/

CTrajectoryQTUserFunctions::CTrajectoryQTUserFunctions() :
   m_cTrajLF(dynamic_cast<CTrajectoryLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) {
}

/****************************************/
/****************************************/

void CTrajectoryQTUserFunctions::DrawInWorld() {
   /* Go through all the robot waypoints and draw them */
   for(auto loc: m_cTrajLF.task_stations) {
      // DrawWaypoints(it->second);
      //  const CVector3& c_position,
      //              const CQuaternion& c_orientation,
      //              const CVector3& c_size
      CQuaternion c_orient = CQuaternion(0, 0, 0, 0);
      CVector3 c_size = CVector3(0.9, 0.9, 0.11);
      CColor c_color = CColor(255, 120, 120);
      DrawBox(loc, c_orient, c_size, c_color);
      CVector3 loc_tmp = loc;
      loc_tmp.SetX(loc_tmp.GetX());
      DrawCylinder(loc_tmp, c_orient, 0.2, 0.01, c_color);
   }
}

/****************************************/
/****************************************/

void CTrajectoryQTUserFunctions::DrawWaypoints(const std::vector<CVector3>& c_waypoints) {
   /* Start drawing segments when you have at least two points */
   if(c_waypoints.size() > 1) {
      size_t unStart = 0;
      size_t unEnd = 1;
      while(unEnd < c_waypoints.size()) {
         DrawRay(CRay3(c_waypoints[unEnd],
                       c_waypoints[unStart]));
         ++unStart;
         ++unEnd;
      }
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CTrajectoryQTUserFunctions, "trajectory_qtuser_functions")
