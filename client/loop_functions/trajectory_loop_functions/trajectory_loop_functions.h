#ifndef TRAJECTORY_LOOP_FUNCTIONS_H
#define TRAJECTORY_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_render.h>
#include <utility>
#include <numeric>
#include "controllers/footbot_diffusion/footbot_diffusion.h"

#define PICK_T 120
#define MOVE_T 15
#define UNLOAD_T 60
#define LOAD_NUM 6
#define WINDOW_SIE 40

using namespace argos;


class CTrajectoryLoopFunctions : public CLoopFunctions {

public:

   typedef std::map<CFootBotEntity*, std::vector<CVector3> > TWaypointMap;
   TWaypointMap m_tWaypoints;
   
public:

   virtual ~CTrajectoryLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);

   virtual void Reset();

   virtual void PostStep();

   inline const TWaypointMap& GetWaypoints() const {
      return m_tWaypoints;
   }

private:
   void addMobileVisualization();

   static CVector3 coordPlanner2Sim(std::pair<int, int>& loc);
   static std::pair<int, int> coordSim2Planner(CVector3& loc);


public:
   std::vector<CVector3> task_stations;
   std::vector<CVector3> all_stations;

   /* Indicates the states of the picker robots, mainly for visualization */
   // std::vector<CVector3> picker_curr_locs;
   // std::vector<CVector3> curr_picking_objs;
   // std::vector<CVector3> picker_unload_locs;

private:
   bool is_initialized = false;
   long long int time_step = 0;
   bool task_finished = false;
};

#endif
