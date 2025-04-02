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
//   for(CTrajectoryLoopFunctions::TWaypointMap::const_iterator it = m_cTrajLF.GetWaypoints().begin();
//       it != m_cTrajLF.GetWaypoints().end();
//       ++it) {
//      DrawWaypoints(it->second);
//   }
   /* Go through all the robot waypoints and draw them */
   for(auto loc: m_cTrajLF.task_pods) {
      // DrawWaypoints(it->second);
      //  const CVector3& c_position,
      //              const CQuaternion& c_orientation,
      //              const CVector3& c_size
      CQuaternion c_orient = CQuaternion(0, 0, 0, 0);
      CVector3 c_size = CVector3(0.85, 0.85, 0.05);
      CColor c_color = CColor(255, 0, 0);
      DrawBox(loc, c_orient, c_size, c_color);
      CVector3 loc_tmp = loc;
      loc_tmp.SetX(loc_tmp.GetX());
      DrawCylinder(loc_tmp, c_orient, 0.2, 0.01, c_color);
   }

   // Ground for msic
   double scale = 1.0;
   double ground_cell_size = 0.97;
   std::pair<double,double> top_left = {-3.0, -0.0};
   CColor ground_msic_color = CColor(235, 225, 255, 12);

   for(int i = 0; i < 37; i++) {
      for (int j = 0; j < 8; j++) {
         CVector3 loc = CVector3(top_left.first - i*scale, top_left.second - j*scale, 0);
         CQuaternion c_orient = CQuaternion(0, 0, 0, 0);
         CVector3 c_size = CVector3(ground_cell_size, ground_cell_size, 0.01);
         DrawBox(loc, c_orient, c_size, ground_msic_color);
         CVector3 loc_tmp = loc;
         loc_tmp.SetX(loc_tmp.GetX());
         // DrawCylinder(loc_tmp, c_orient, 0.2, 0.01, ground_msic_color);
      }
   }

   top_left = {-3.0, -16.0};
   for(int i = 0; i < 37; i++) {
      for (int j = 0; j < 16; j++) {
         CVector3 loc = CVector3(top_left.first - i*scale, top_left.second - j*scale, 0);
         CQuaternion c_orient = CQuaternion(0, 0, 0, 0);
         CVector3 c_size = CVector3(ground_cell_size, ground_cell_size, 0.01);
         DrawBox(loc, c_orient, c_size, ground_msic_color);
         CVector3 loc_tmp = loc;
         loc_tmp.SetX(loc_tmp.GetX());
         // DrawCylinder(loc_tmp, c_orient, 0.2, 0.01, ground_msic_color);
      }
   }

   // Ground for soap
   top_left = {-3.0, -32.0};
   for(int i = 0; i < 37; i++) {
      for (int j = 0; j < 8; j++) {
         CVector3 loc = CVector3(top_left.first - i*scale, top_left.second - j*scale, 0);
         CQuaternion c_orient = CQuaternion(0, 0, 0, 0);
         CVector3 c_size = CVector3(ground_cell_size, ground_cell_size, 0.05);
         CColor c_color = CColor(235, 255, 225, 12);
         DrawBox(loc, c_orient, c_size, c_color);
         CVector3 loc_tmp = loc;
         loc_tmp.SetX(loc_tmp.GetX());
         DrawCylinder(loc_tmp, c_orient, 0.2, 0.01, c_color);
      }
   }

   for(auto loc: m_cTrajLF.task_stations) {
      // DrawWaypoints(it->second);
      //  const CVector3& c_position,
      //              const CQuaternion& c_orientation,
      //              const CVector3& c_size
      CQuaternion c_orient = CQuaternion(0, 0, 0, 0);
      CVector3 c_size = CVector3(0.9, 0.9, 0.05);
      CColor c_color = CColor(255, 120, 120);
      DrawBox(loc, c_orient, c_size, c_color);
      CVector3 loc_tmp = loc;
      loc_tmp.SetX(loc_tmp.GetX());
      DrawCylinder(loc_tmp, c_orient, 0.2, 0.01, c_color);
   }

   for(auto loc: m_cTrajLF.all_stations) {
      // DrawWaypoints(it->second);
      //  const CVector3& c_position,
      //              const CQuaternion& c_orientation,
      //              const CVector3& c_size
      CQuaternion c_orient = CQuaternion(0, 0, 0, 0);
      CVector3 c_size = CVector3(0.9, 0.9, 0.05);
      CColor c_color = CColor(120, 120, 255);
      DrawBox(loc, c_orient, c_size, c_color);
      CVector3 loc_tmp = loc;
      loc_tmp.SetX(loc_tmp.GetX());
      DrawCylinder(loc_tmp, c_orient, 0.2, 0.01, c_color);
   }

   // std::vector<CVector3> picker_curr_locs;
   // std::vector<CVector3> curr_picking_objs;
   // std::vector<CVector3> picker_unload_locs;
   for(auto loc: m_cTrajLF.picker_curr_locs) {
      // DrawWaypoints(it->second);
      //  const CVector3& c_position,
      //              const CQuaternion& c_orientation,
      //              const CVector3& c_size
      CQuaternion c_orient = CQuaternion(0, 0, 0, 0);
      CVector3 c_size = CVector3(0.9, 0.9, 1.05);
      CColor c_color = CColor(0, 0, 255);
      DrawBox(loc, c_orient, c_size, c_color);
      CVector3 loc_tmp = loc;
      loc_tmp.SetX(loc_tmp.GetX());
      DrawCylinder(loc_tmp, c_orient, 0.2, 0.01, c_color);
   }

   for(auto loc: m_cTrajLF.curr_picking_objs) {
      // DrawWaypoints(it->second);
      //  const CVector3& c_position,
      //              const CQuaternion& c_orientation,
      //              const CVector3& c_size
      CQuaternion c_orient = CQuaternion(0, 0, 0, 0);
      CVector3 c_size = CVector3(0.9, 0.9, 1.05);
      CColor c_color = CColor(0, 255, 0);
      DrawBox(loc, c_orient, c_size, c_color);
      CVector3 loc_tmp = loc;
      loc_tmp.SetX(loc_tmp.GetX());
      DrawCylinder(loc_tmp, c_orient, 0.2, 0.01, c_color);
   }

   for(auto loc: m_cTrajLF.picker_unload_locs) {
      // DrawWaypoints(it->second);
      //  const CVector3& c_position,
      //              const CQuaternion& c_orientation,
      //              const CVector3& c_size
      CQuaternion c_orient = CQuaternion(0, 0, 0, 0);
      CVector3 c_size = CVector3(0.9, 0.9, 1.05);
      CColor c_color = CColor(255, 0, 0);
      DrawBox(loc, c_orient, c_size, c_color);
      CVector3 loc_tmp = loc;
      loc_tmp.SetX(loc_tmp.GetX());
      DrawCylinder(loc_tmp, c_orient, 0.2, 0.01, c_color);
   }


   // iter_val++;
   // CFootBotEntity* pcFootBot = dynamic_cast<CFootBotEntity*>(
   //         GetSpace().GetEntity("fb0")  // Assuming "fb0" is the robot's ID in the .argos file
   //     );
   //
   // if (pcFootBot) {
   //     // Get the controller
   //     MyController* pcController = dynamic_cast<MyController*>(
   //         &(pcFootBot->GetControllableEntity().GetController())
   //     );
   //
   //     if (pcController) {
   //         // Retrieve step count or any other return value
   //         LOG << "Step Count: " << pcController->GetStepCount() << std::endl;
   //     }
   // }
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
