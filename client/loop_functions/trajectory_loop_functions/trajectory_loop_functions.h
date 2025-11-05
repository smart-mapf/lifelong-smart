#ifndef TRAJECTORY_LOOP_FUNCTIONS_H
#define TRAJECTORY_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_render.h>

#include "controllers/footbot_diffusion/footbot_diffusion.h"
#include "utils/common.h"

using namespace argos;

class CTrajectoryLoopFunctions : public CLoopFunctions {
public:
    typedef map<CFootBotEntity*, vector<CVector3> > TWaypointMap;
    TWaypointMap m_tWaypoints;

public:
    virtual ~CTrajectoryLoopFunctions() {
    }

    virtual void Init(TConfigurationNode& t_tree);

    virtual void Reset();

    virtual void PostStep();

    virtual void PreStep();

    void PostExperiment() override;

    bool IsExperimentFinished() override;

    void Destroy() override;

    inline const TWaypointMap& GetWaypoints() const {
        return m_tWaypoints;
    }

private:
    void addMobileVisualization();

public:
    vector<CVector3> task_stations;

private:
    bool is_initialized = false;
    int port_number = -1;
    shared_ptr<rpc::client> client;
    int time_step_tick = 0;
    bool end_sim = false;
};

#endif
