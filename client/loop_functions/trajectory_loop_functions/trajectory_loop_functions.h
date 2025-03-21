#ifndef TRAJECTORY_LOOP_FUNCTIONS_H
#define TRAJECTORY_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include "controllers/footbot_diffusion/footbot_diffusion.h"

using namespace argos;

struct PickerRobot {
   std::pair<int, int> position;
   int loads = 0;
   int stay_timer = 10;
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

   std::vector<CVector3> task_pods;
   std::vector<CVector3> task_stations;
   std::vector<CVector3> all_stations;

private:
   bool is_initialized = false;
   std::shared_ptr<rpc::client> client;
   std::unordered_map<std::pair<int, int>, int> all_pick_tasks;
   int port_number = 0;
   int num_picker = 0;
};

#endif
