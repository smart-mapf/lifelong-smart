/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */

#ifndef FOOTBOT_DIFFUSION_H
#define FOOTBOT_DIFFUSION_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_encoder_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_turret_encoder_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_turret_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
// #include <argos3/core/simulator/entity/positional_entity.h>
#include <queue>
#include <map>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <rpc/client.h>
#include <fstream>
#include <filesystem>
#include <boost/asio.hpp>

#define EPS 0.03f
#define DELIVER_T 60
#define PICKER_T 60
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;
using outputTuple = std::tuple<std::string, int, double, std::string, std::pair<double, double>, std::pair<double, double>, int>;

struct Action {
    // target position and orientation
    Real x;
    Real y;
    Real angle;
    std::deque<int> nodeIDS;
    int timer = -1;
    int task_id = -1;
    enum Type {
        MOVE,
        TURN,
        STOP,
        PICKER,
        STATION
    } type;
    Action() {
        x = 0.0, y = 0.0, angle = 0.0;
        type= STOP;
    }
    Action(Real x, Real y, Real angle, std::deque<int> node_ids, Type act_type, int time_dura = -1, int task_id = -1):
        x(x), y(y), angle(angle), nodeIDS(std::move(node_ids)), timer(time_dura), type(act_type), task_id(task_id) {}
};

struct Pos {
    Real x;
    Real y;

    // Define less-than operator for Pos
    bool operator<(const Pos &other) const {
        // Define your comparison logic here
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
};

class PIDController {
public:
    PIDController(Real kp, Real ki, Real kd)
            : kp_(kp), ki_(ki), kd_(kd), prevError_(0), integral_(0) {}

    Real calculate(Real error) {
        integral_ += error * dt;
        Real derivative = (error - prevError_) / dt;
        prevError_ = error;

        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

private:
    Real kp_, ki_, kd_;
    Real prevError_;
    Real integral_;
    Real dt = 0.1;
};

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotDiffusion : public CCI_Controller {


public:

    /* Class constructor. */
    CFootBotDiffusion();

    /* Class destructor. */
    virtual ~CFootBotDiffusion() {}

    /*
     * This function initializes the controller.
     * The 't_node' variable points to the <parameters> section in the XML
     * file in the <controllers><footbot_diffusion_controller> section.
     */
    virtual void Init(TConfigurationNode &t_node);

    /*
     * This function is called once every time step.
     * The length of the time step is set in the XML file.
     */
    virtual void ControlStep();

    /*
     * This function resets the controller to its state right after the
     * Init().
     * It is called when you press the reset button in the GUI.
     * In this example controller there is no need for resetting anything,
     * so the function could have been omitted. It's here just for
     * completeness.
     */
    virtual void Reset() {}

    /*
     * Called to cleanup what done by Init() when the experiment finishes.
     * In this example controller there is no need for clean anything up,
     * so the function could have been omitted. It's here just for
     * completeness.
     */
    virtual void Destroy() {}

    CVector3 getCurrPod() {return curr_pod;};

    CVector3 getCurrStation() {return curr_station;}

private:
    Real pidLinear(Real error);
    std::pair<Real, Real> Move(const CVector3& targetPos, const CVector3& currPos, Real currAngle, Real tolerance);
    std::pair<Real, Real> inline pidAngular(Real error);
    std::pair<Real, Real> Turn(Real targetAngle, Real currAngle, Real tolerance);
    void TurnLeft(Real angle, Real currAngle, Real tolerance) const;

    static Real ChangeCoordinateFromMapToArgos(Real x);
    static Real ChangeCoordinateFromArgosToMap(Real x);
    void insertActions(const std::vector<outputTuple>& actions);
    double getReferenceSpeed(double dist) const;
    void updateQueue();

public:
    std::map< int, std::pair<bool, bool> > picker_task;

private:
    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator *m_pcWheels;
    /* Pointer to the foot-bot proximity sensor */
    CCI_FootBotProximitySensor *m_pcProximity;
    CCI_PositioningSensor *m_pcPosSens;
    int m_tickPerSec = 10;
    // CPositionalEntity* m_pcPosEntity;
    std::string robot_id;
    CDegrees m_cAlpha;
    int port_number;
    Real m_linearVelocity;
    Real m_angularVelocity;
    Real m_linearAcceleration;
    Real m_currVelocity;
    /* Wheel speed. */
    Real m_fWheelVelocity;
    CRange<CRadians> m_cGoStraightAngleRange;
    std::string m_outputDir;

    std::deque<Action> q;
    std::queue<Real> velocityQueue;
    int moveForwardFlag;
    int count = 0;
    Real offset = 15.0;
    std::shared_ptr<rpc::client> client;

    std::shared_ptr<PIDController> angular_pid_ptr;
    std::shared_ptr<PIDController> linear_pid_ptr;

    Real dt = 0.1;
    // Hyper-parameter for PID::Turn
    Real prev_turn_error=0.0;
    Real integral_turn_error=0.0;
    Real kp_turn_ = 0.8;
    Real ki_turn_ = 0.0;
    Real kd_turn_ = 0.1;

    // Hyper-Parameter for PID::Move
    Real prevLeftVelocity_ = 0.0;
    Real prevRightVelocity_ = 0.0;
    Real prevVelocity_ = 0.0;
    Real prev_move_error=0.0;
    Real integral_move_error=0.0;
    Real kp_move_ = 0.6;
    Real ki_move_ = 0.0;
    Real kd_move_ = 0.0;
//    std::string debug_id = "7_6";
//    std::string debug_id = "1_4";
    std::string debug_id = "10_4";
    // std::string debug_id = "-1";

    int lineExistFlag = 0;
    bool terminateFlag = false;

    std::ofstream outputFile;
    std::string outputDir;

    bool is_initialized = false;
    CVector3 curr_pod{-1,-1,-100};
    CVector3 curr_station{-1,-1,-100};
};

#endif
