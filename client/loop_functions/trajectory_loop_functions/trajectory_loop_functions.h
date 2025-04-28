#ifndef TRAJECTORY_LOOP_FUNCTIONS_H
#define TRAJECTORY_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_render.h>
#include <utility>
#include <numeric>
#include "controllers/footbot_diffusion/footbot_diffusion.h"

#define PICK_T 100
#define MOVE_T 50
#define UNLOAD_T 60
#define LOAD_NUM 8
#define WINDOW_SIE 40

using namespace argos;

enum ACT_TYPE {
   NONE = 0,
   MOVE = 1,
   PICK = 2,
   UNLOAD = 3
};

struct PickerAction {
   int timer;
   ACT_TYPE act;
   int step_id;
   int task_id;
   std::pair<int, int> start;
   std::pair<int, int> end;
   std::pair<int, int> start_obj;
   std::pair<int, int> end_obj;
   // std::pair<int, int> obj;
   PickerAction(int t, ACT_TYPE act_type, std::pair<int, int> s, std::pair<int, int> e, std::pair<int, int> s_obj,
      std::pair<int, int> e_obj, int move_step_id = -1,
      int task_id = -1):
      timer(t), act(act_type), start(std::move(s)), end(std::move(e)), start_obj(std::move(s_obj)),
      end_obj(std::move(e_obj)), step_id(move_step_id), task_id(task_id) {}
};

struct PickerRobot {
   std::pair<int, int> position;
   int loads = 0;
   int curr_load = 0;
   std::deque<PickerAction> acts;
};

struct pair_hash {
   std::size_t operator()(const std::pair<int, int>& p) const {
      return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
   }
};

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

   bool is_port_open(const std::string& ip, int port) {
      boost::asio::io_context io_context;
      boost::asio::ip::tcp::socket socket(io_context);

      try {
         // Create an endpoint with the provided IP and port
         boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(ip), port);

         // Attempt to connect to the endpoint
         socket.connect(endpoint);
         return true;  // Connection succeeded
      } catch (const boost::system::system_error& e) {
         // Catch connection errors
         // std::cerr << "Connection failed: " << e.what() << std::endl;
         return false;
      }
   }

private:
   void getNextAction(int agent_id);
   void initRobot(int agent_id);
   void readPickerPath();
   inline int nextloc(int agent_id, int curr_loc_id, std::pair<int, int>& next_loc) {
      const auto& picker_path = all_picker_paths[agent_id];
      int next_loc_id = curr_loc_id + 1;
      if (next_loc_id < picker_path.size()) {
         ;
      } else if (next_loc_id == picker_path.size()) {
         next_loc_id = 0;
      } else {
         std::cerr << "ERROR: next_loc_id out of range" << std::endl;
         exit(-1);
      }
      next_loc = picker_path[next_loc_id];
      return next_loc_id;
   }
   int requestMobileRobot(int agent_id, std::pair<int, int>& loc);
   bool executeMove(int agent_id, PickerAction& act);
   bool executePick(int agent_id, PickerAction& act);
   bool executeUnload(int agent_id, PickerAction& act);
   void initActionQueue();
   void requestNewPickTasks();
   void addMobileVisualization();

   static CVector3 coordPlanner2Sim(std::pair<int, int>& loc);
   static std::pair<int, int> coordSim2Planner(CVector3& loc);


public:
   std::vector<CVector3> task_pods;
   std::vector<CVector3> task_stations;
   std::vector<CVector3> all_stations;

   /* Indicates the states of the picker robots, mainly for visualization */
   std::vector<CVector3> picker_curr_locs;
   std::vector<CVector3> curr_picking_objs;
   std::vector<CVector3> picker_unload_locs;

private:
   bool is_initialized = false;
   std::shared_ptr<rpc::client> client;
   std::unordered_map<std::pair<int, int>, std::deque<int>, pair_hash> all_pick_tasks;
   std::vector<std::vector<std::pair<int, int>>> all_picker_paths;
   std::vector<std::vector<std::pair<int, int>>> picker_objs;
   std::vector<PickerRobot> all_pickers;
   std::vector<int> agents_wait_time;
   int port_number = 0;
   int num_picker = 0;
   long long int time_step = 0;
};

#endif
