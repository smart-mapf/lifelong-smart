/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector3.h>

/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion()
    : m_pcWheels(nullptr),
      m_pcProximity(nullptr),
      m_pcPosSens(nullptr),
      m_cAlpha(10.0f),
      m_angularVelocity(0.5f),
      m_fWheelVelocity(2.5f),
      m_cGoStraightAngleRange(-ToRadians(m_cAlpha), ToRadians(m_cAlpha)) {
}

/****************************************/
/****************************************/
void CFootBotDiffusion::insertActions(const std::vector<outputTuple>& actions) {
    // std::vector<action> tmp_cache;
    for (const auto& action : actions) {
        // std::cout << "not empty init" << std::endl;
        std::string action1 = std::get<3>(action);
        int nodeID = std::get<1>(action);
        //        std::cout << "NodeID init: " << nodeID << std::endl;
        std::tuple<double, double> start_pos = std::get<4>(action);
        double start_x = ChangeCoordinateFromMapToArgos(std::get<1>(start_pos));
        double start_y = ChangeCoordinateFromMapToArgos(std::get<0>(start_pos));
        std::tuple<double, double> end_pos = std::get<5>(action);
        //        std::cout << "End Position init: " << std::get<0>(end_pos) <<
        //        " " << std::get<1>(end_pos) << std::endl;
        double x = ChangeCoordinateFromMapToArgos(std::get<1>(end_pos));
        double y = ChangeCoordinateFromMapToArgos(std::get<0>(end_pos));
        double angle;
        if (std::get<2>(action) == 0) {
            angle = 0.0;
        } else if (std::get<2>(action) == 1) {
            angle = 270.0;
        } else if (std::get<2>(action) == 2) {
            angle = 180.0;
        } else {
            angle = 90.0;
        }
        int task_id = std::get<6>(action);
        if (robot_id == debug_id) {
            std::cout << "Action: " << action1 << " NodeID: " << nodeID
                      << " End Position: " << x << " " << y
                      << " Angle: " << angle << std::endl;
        }
        if (action1 == "M") {
            std::deque<int> prev_ids;
            if (not q.empty() and q.back().type == Action::MOVE) {
                prev_ids = q.back().nodeIDS;
                q.pop_back();
            }
            prev_ids.push_back(nodeID);
            q.emplace_back(x, y, angle, prev_ids, Action::MOVE);
        } else if (action1 == "T") {
            q.emplace_back(x, y, angle, std::deque<int>{nodeID}, Action::TURN);
        } else if (action1 == "S") {
            q.emplace_back(x, y, angle, std::deque<int>{nodeID},
                           Action::STATION, DELIVER_T, task_id);
        } else {
            printf("Unknown action %s\n", action1.c_str());
            exit(-2);
            q.emplace_back(x, y, angle, std::deque<int>{nodeID}, Action::STOP);
        }
    }
    // q.back_insert(q.begin(), tmp_cache.begin(), tmp_cache.end());
}

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
    /*
     * Get sensor/actuator handles
     *
     * The passed string (ex. "differential_steering") corresponds to the
     * XML tag of the device whose handle we want to have. For a list of
     * allowed values, type at the command prompt:
     *
     * $ argos3 -q actuators
     *
     * to have a list of all the possible actuators, or
     *
     * $ argos3 -q sensors
     *
     * to have a list of all the possible sensors.
     *
     * NOTE: ARGoS creates and initializes actuators and sensors
     * internally, on the basis of the lists provided the configuration
     * file at the <controllers><footbot_diffusion><actuators> and
     * <controllers><footbot_diffusion><sensors> sections. If you forgot to
     * list a device in the XML, and then you request it here, an error
     * occurs.
     */
    m_pcWheels =
        GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    m_pcPosSens = GetSensor<CCI_PositioningSensor>("positioning");

    angular_pid_ptr = std::make_shared<PIDController>(1.0, 0.0, 0.1);
    linear_pid_ptr = std::make_shared<PIDController>(2.0, 0.0, 0.1);
    /*
     * Parse the configuration file
     *
     * The user defines this part. Here, the algorithm accepts three
     * parameters, and it's nice to put them in the config file so we don't
     * have to recompile if we want to try other settings.
     */
    GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
    m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
    GetNodeAttributeOrDefault(t_node, "omega", m_angularVelocity,
                              m_angularVelocity);
    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity,
                              m_fWheelVelocity);
    GetNodeAttributeOrDefault(t_node, "acceleration", m_linearAcceleration,
                              m_linearAcceleration);
    GetNodeAttributeOrDefault(t_node, "portNumber", port_number, 8080);
    GetNodeAttributeOrDefault(t_node, "simDuration", total_sim_duration, 1200);
    GetNodeAttributeOrDefault(t_node, "outputDir", m_outputDir,
                              std::string("metaData/"));
    GetNodeAttributeOrDefault(t_node, "screen", screen, 0);
    m_linearVelocity = 1.22 * m_angularVelocity;
    m_currVelocity = 0.0;
    this->init_pos = m_pcPosSens->GetReading().Position;
    // CVector3 currPos = m_pcPosSens->GetReading().Position;
    // robot_id =
    //     std::to_string(
    //         static_cast<int>(ChangeCoordinateFromArgosToMap(currPos.GetY())))
    //         +
    //     "_" +
    //     std::to_string(
    //         static_cast<int>(ChangeCoordinateFromArgosToMap(currPos.GetX())));
    // spdlog::info("My Robot ID: {}, argos Robot ID: {}", robot_id,
    //              this->GetId());
    this->robot_id = this->GetId();
}

/****************************************/

Real normalizeAngle(Real angle) {
    while (angle > 180)
        angle -= 360;
    while (angle < -180)
        angle += 360;
    return angle;
}

double angleDifference(double curr_angle, double target_angle) {
    double diff = normalizeAngle(target_angle) - normalizeAngle(curr_angle);
    // Adjust the difference to be in the range [-180, 180] degrees
    if (diff > 180.0) {
        diff -= 360.0;
    } else if (diff < -180.0) {
        diff += 360.0;
    }

    return abs(diff);
}
/****************************************/

void CFootBotDiffusion::updateQueue() {
    if (q.empty()) {
        return;
    }
    auto first_act = q.front();
    if (first_act.type == Action::MOVE) {
        q.pop_front();
        while (not q.empty() and q.front().type == Action::MOVE) {
            auto next_act = q.front();
            q.pop_front();
            first_act.x = next_act.x;
            first_act.y = next_act.y;
            first_act.nodeIDS.insert(first_act.nodeIDS.end(),
                                     next_act.nodeIDS.begin(),
                                     next_act.nodeIDS.end());
        }
        q.push_front(first_act);
    }
    if (robot_id == debug_id) {
        for (auto& tmp_act : q) {
            std::cout << tmp_act.type << ", Goal position: (" << tmp_act.x
                      << ", " << tmp_act.y << ")" << std::endl;
        }
        std::cout << "#######################" << std::endl;
    }
}

void CFootBotDiffusion::ControlStep() {
    if (not is_initialized) {
        if (is_port_open("127.0.0.1", port_number)) {
            client = std::make_shared<rpc::client>("127.0.0.1", port_number);
        } else {
            std::cout << "Failed to connect to server. Retrying..."
                      << std::endl;
            return;
        }
        is_initialized = true;
        auto init_loc = std::make_tuple(
            static_cast<int>(
                ChangeCoordinateFromArgosToMap(this->init_pos.GetY())),
            static_cast<int>(
                ChangeCoordinateFromArgosToMap(this->init_pos.GetX())))

            ;
        client->call("init", robot_id, init_loc);
        if (screen > 0) {
            // std::cout << "Robot " << robot_id
            //           << " connected to server at port: " << port_number
            //           << std::endl;
            spdlog::info("Robot {} at ({},{}) connected to server at port: {}",
                         robot_id, std::get<1>(init_loc), std::get<0>(init_loc),
                         port_number);
        }
        return;
    }
    auto start = std::chrono::high_resolution_clock::now();
    Action a;
    a.type = Action::STOP;
    CQuaternion currOrient = m_pcPosSens->GetReading().Orientation;
    CRadians cZAngle, cYAngle, cXAngle;
    currOrient.ToEulerAngles(cZAngle, cYAngle, cXAngle);
    Real currAngle = ToDegrees(cZAngle).GetValue();
    if (currAngle < 0.0) {
        currAngle = currAngle + 360.0;
    }
    Real left_v, right_v;
    CVector3 currPos = m_pcPosSens->GetReading().Position;
    if (count % 10 == 0) {
        auto updateActions =
            client->call("update", robot_id).as<std::vector<outputTuple>>();
        if (not updateActions.empty()) {
            insertActions(updateActions);
        }
    }
    auto end_update = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> update_duration_ms =
        end_update - start;

    count++;
    std::string receive_msg;
    updateQueue();
    auto end_queue_update = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> queue_update_duration_ms =
        end_queue_update - end_update;
    while (!q.empty()) {
        a = q.front();
        CVector3 targetPos = CVector3(a.x, a.y, 0.0f);
        if (a.type == Action::MOVE && ((currPos - targetPos).Length() < EPS) and
            (abs(prevVelocity_)) <= dt * m_fWheelVelocity) {
            if (robot_id == debug_id) {
                std::cout << "Confirm move when reach the goal! Remaining "
                             "actions num: "
                          << a.nodeIDS.size() << std::endl;
                std::cout << "Action: " << a.type << ", Target Position: ("
                          << a.x << ", " << a.y << ")"
                          << ", Current Position: (" << currPos.GetX() << ", "
                          << currPos.GetY()
                          << "). Previous speed is: " << prevVelocity_
                          << std::endl;
            }
            a.type = Action::STOP;
            q.pop_front();
            for (auto tmp_nodeId : a.nodeIDS) {
                receive_msg =
                    client->call("receive_update", robot_id, tmp_nodeId)
                        .as<std::string>();
            }
            continue;
        } else if (a.type == Action::MOVE &&
                   ((currPos - targetPos).Length() +
                    MOVE_DIS * (-static_cast<double>(a.nodeIDS.size()) + 1)) <
                       EPS) {
            if (robot_id == debug_id) {
                std::cout << "Confirm move on the fly! Remaining actions num: "
                          << a.nodeIDS.size() << std::endl;
                std::cout << "Action: " << a.type << ", Target Position: ("
                          << a.x << ", " << a.y << ")"
                          << ", Current Position: (" << currPos.GetX() << ", "
                          << currPos.GetY()
                          << "). Previous speed is: " << prevVelocity_
                          << std::endl;
            }
            if (a.nodeIDS.size() > 1) {
                receive_msg =
                    client->call("receive_update", robot_id, a.nodeIDS.front())
                        .as<std::string>();
                q.front().nodeIDS.pop_front();
            }
            if ((currPos - targetPos).Length() < EPS) {
                if ((abs(prevVelocity_)) <= dt * m_fWheelVelocity) {
                    a.type = Action::STOP;
                    q.pop_front();
                    for (auto tmp_nodeId : a.nodeIDS) {
                        receive_msg =
                            client->call("receive_update", robot_id, tmp_nodeId)
                                .as<std::string>();
                    }
                    continue;
                }
            }
        } else if (a.type == Action::TURN &&
                   angleDifference(currAngle, a.angle) < 0.5f) {
            a.type = Action::STOP;
            receive_msg =
                client->call("receive_update", robot_id, a.nodeIDS.front())
                    .as<std::string>();
            q.pop_front();
            continue;
        } else if (a.type == Action::STATION and a.timer <= 0) {
            a.type = Action::STOP;
            receive_msg =
                client->call("receive_update", robot_id, a.nodeIDS.front())
                    .as<std::string>();
            q.pop_front();
            curr_station = CVector3{-1, -1, -100};
            continue;
        }
        break;
    }

    auto end_find_act = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> find_act_duration_ms =
        end_find_act - end_queue_update;

    if (a.type == Action::MOVE) {
        CVector3 targetPos = CVector3(a.x, a.y, 0.0f);
        std::pair<Real, Real> velocities =
            Move(targetPos, currPos, currAngle, 1.0f);
        left_v = velocities.first;
        right_v = velocities.second;
    } else if (a.type == Action::TURN) {
        //        TurnLeft(a.angle, currAngle, 1.0f);
        std::pair<Real, Real> turn_velocities = Turn(a.angle, currAngle, 1.0f);
        left_v = turn_velocities.first;
        right_v = turn_velocities.second;
    } else if (a.type == Action::STATION) {
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
        curr_station = CVector3{a.x, a.y, 0.0f};
        q.front().timer--;
    } else {
        // stop state, waiting for next instruction
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
        left_v = 0.0f;
        right_v = 0.0f;
    }

    // Update sim step count in client and server.
    step_count_++;
    bool end_sim = client->call("update_sim_step", robot_id).as<bool>();
    // if (end_sim) {
    //     client->async_call("close_server");
    //     exit(0);
    // }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> exec_duration_ms =
        end - end_find_act;
    std::chrono::duration<double, std::milli> total_duration_ms = end - start;
    if (total_duration_ms.count() > 1 && screen > 0) {
        spdlog::info(
            "Agent {}: Total time: {:.2f} ms, Update time: {:.2f} ms, "
            "Queue update time: {:.2f} ms, Find action time: {:.2f} ms, "
            "Execution time: {:.2f} ms, with control step count: {}",
            robot_id, total_duration_ms.count(), update_duration_ms.count(),
            queue_update_duration_ms.count(), find_act_duration_ms.count(),
            exec_duration_ms.count(), step_count_);
        // std::cout << "Agent " << robot_id
        //           << ": Total time:" << total_duration_ms.count() << " ms, "
        //           << "Update time: " << update_duration_ms.count() << " ms, "
        //           << "Queue update time: " <<
        //           queue_update_duration_ms.count()
        //           << " ms, "
        //           << "Find action time: " << find_act_duration_ms.count()
        //           << " ms, "
        //           << "Execution time: " << exec_duration_ms.count()
        //           << " ms, with control step count: " << step_count_ << "\n";
    }

    // Free simulation if necessary
    client->call("freeze_simulation_if_necessary", this->robot_id);

    // Loop until the simulation is defrozen
    while (client->call("is_simulation_frozen").as<bool>()) {
        // std::cout << "Robot " << this->robot_id
        //           << ": Simulation is frozen, waiting to defrost..."
        //           << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

std::pair<Real, Real> CFootBotDiffusion::pidAngular(Real error) {
    //    Real error = normalizeAngle(targetAngle - currAngle);
    integral_turn_error += error * dt;
    Real derivative = (error - prev_turn_error) / dt;
    prev_turn_error = error;

    Real output = kp_turn_ * error + ki_turn_ * integral_turn_error +
                  kd_turn_ * derivative;
    // Clamp the output to the max velocity
    output = std::clamp(output, -m_linearVelocity, m_linearVelocity);
    Real left_v = -output, right_v = output;
    return std::make_pair(left_v, right_v);
}

Real CFootBotDiffusion::pidLinear(Real error) {
    integral_move_error += error * dt;
    Real derivative = (error - prev_move_error) / dt;
    prev_move_error = error;
    Real desiredVelocity = kp_move_ * error + ki_move_ * integral_move_error +
                           kd_move_ * derivative;
    return desiredVelocity;
}

std::pair<Real, Real> CFootBotDiffusion::Turn(Real targetAngle, Real currAngle,
                                              Real tolerance = 1.0f) {
    Real error = normalizeAngle(targetAngle - currAngle);
    auto turn_v = pidAngular(error);
    Real left_v = turn_v.first;
    Real right_v = turn_v.second;
    // if ("04" == robot_id){
    // std::cout << "PID modified velocity" << left_v<< ", " << right_v <<
    // std::endl;
    // }

    //    std::cout << "PID::Left_v: " << left_v << ", right_v: " << right_v <<
    //    std::endl;
    m_pcWheels->SetLinearVelocity(left_v, right_v);
    return std::make_pair(left_v, right_v);
}

inline Real toAngle(Real deltaX, Real deltaY) {
    Real targetAngle = std::atan2(deltaY, deltaX);
    Real tmp_angle = targetAngle / M_PI * 180.0 + 360;
    if (tmp_angle > 360) {
        tmp_angle -= 360;
    }
    assert(tmp_angle >= 0.0);
    return tmp_angle;
}

double CFootBotDiffusion::getReferenceSpeed(double dist) const {
    if (debug_id == robot_id)
        std::cout << "reference dist is: " << dist << std::endl;
    int dist_flag = 0;
    if (dist < 0) {
        dist_flag = -1;
    } else if (dist > 0) {
        dist_flag = 1;
    }
    return dist_flag *
           std::min(sqrt(2 * m_linearAcceleration * std::abs(dist) / dt),
                    m_fWheelVelocity);
}

std::pair<Real, Real> CFootBotDiffusion::Move(const CVector3& targetPos,
                                              const CVector3& currPos,
                                              Real currAngle,
                                              Real tolerance = 1.0f) {
    // Calculate the distance and angle to the target
    Real deltaX = targetPos.GetX() - currPos.GetX();
    Real deltaY = targetPos.GetY() - currPos.GetY();
    Real distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);
    Real targetAngle = toAngle(deltaX, deltaY);
    Real targetAngle2 = toAngle(-deltaX, -deltaY);

    // Normalize angle difference
    Real angleError1 = normalizeAngle(targetAngle - currAngle);
    Real angleError2 = normalizeAngle(targetAngle2 - currAngle);
    Real angleError = 0.0;
    Real flag = 0.0;
    //    std::cout << "angle error 1: " << angleError1 << ", angle error 2: "
    //    << angleError2 << std::endl;
    if (std::abs(angleError1) < std::abs(angleError2)) {
        angleError = angleError1;
        flag = 1.0;
    } else {
        angleError = angleError2;
        flag = -1.0;
    }

    // PID calculations
    Real refer_velocity = flag * getReferenceSpeed(distance);
    // std::cout << "Reference velocity: " << refer_velocity << std::endl;
    Real control_acc = pidLinear(refer_velocity - prevVelocity_);
    auto angularVelocity = pidAngular(angleError);

    //    std::cout << "PID::init_l_v: " << left_v_total << ", init_r_v: " <<
    //    right_v_total << std::endl;

    // Clamp the output to the max velocity
    Real maxDeltaV = m_linearAcceleration * dt;
    Real linearVelocity =
        prevVelocity_ + std::clamp(control_acc, -maxDeltaV, maxDeltaV);
    ;
    //    std::cout << "linearVelocity: " << linearVelocity << std::endl;
    // if (debug_id == robot_id){
    //     std::cout <<
    //     "PID::start###########################################################"
    //     << std::endl; std::cout << "Distance error: " << distance << ",
    //     PID::Angle Error: " << angleError << ", curr angle: " <<
    //               currAngle << ", target angle: " << targetAngle <<", flag
    //               is: " << flag << std::endl;
    //     std::cout << "PID::Linear Velocity: " << linearVelocity << ", refer
    //     vel: " << refer_velocity
    //         << "Prev vel: " << prevVelocity_ << std::endl;
    //     std::cout << "PID::Angular Velocity: " << angularVelocity.first << ",
    //     " << angularVelocity.second << std::endl; std::cout << "control acc:
    //     " << control_acc << std::endl; std::cout << "curr pos (" <<
    //     currPos.GetX() << ", " << currPos.GetY()
    //               << "). curr angle: " << currAngle
    //               << ", target angle: " << targetAngle
    //               << ", target angle2: " << targetAngle2
    //               << ", delta_x: " << deltaX
    //               << ", delta_y: " << deltaY
    //               << std::endl;
    // }
    Real left_v_total = linearVelocity;
    Real right_v_total = linearVelocity;
    //    left_v_total = std::clamp(left_v_total, prevLeftVelocity_ - maxDeltaV,
    //    prevLeftVelocity_ + maxDeltaV); right_v_total =
    //    std::clamp(right_v_total, prevRightVelocity_ - maxDeltaV,
    //    prevRightVelocity_ + maxDeltaV);

    left_v_total =
        std::clamp(left_v_total, -m_fWheelVelocity, m_fWheelVelocity);
    right_v_total =
        std::clamp(right_v_total, -m_fWheelVelocity, m_fWheelVelocity);
    left_v_total += (1 / m_fWheelVelocity) * std::abs(linearVelocity) *
                    angularVelocity.first;
    right_v_total += (1 / m_fWheelVelocity) * std::abs(linearVelocity) *
                     angularVelocity.second;
    // Update previous velocities for the next iteration
    prevLeftVelocity_ = left_v_total;
    prevRightVelocity_ = right_v_total;
    prevVelocity_ = linearVelocity;
    // std::cout << "PID::Left_v: " << left_v_total << ", right_v: " <<
    // right_v_total << ", refer speed: " << refer_velocity << std::endl;
    m_pcWheels->SetLinearVelocity(left_v_total, right_v_total);
    // if (debug_id == robot_id){
    // std::cout << "target position: " << targetPos.GetX() << ", " <<
    // targetPos.GetY() << std::endl; std::cout << "PID modified velocity" <<
    // left_v_total<< ", " << right_v_total << std::endl;
    // }
    return std::make_pair(left_v_total, right_v_total);
}

void CFootBotDiffusion::TurnLeft(Real targetAngle, Real currAngle,
                                 Real tolerance = 1.0f) const {
    Real angleDifference = targetAngle - currAngle;
    Real left_v, right_v;
    if (angleDifference > 0.0 && angleDifference < 180.0) {
        if (abs(angleDifference) <= 10.0) {
            left_v = -m_linearVelocity / (11.0 - abs(angleDifference));
            right_v = +m_linearVelocity / (11.0 - abs(angleDifference));
        } else {
            left_v = -m_linearVelocity;
            right_v = m_linearVelocity;
        }
    } else {
        if (abs(angleDifference) <= 10.0) {
            left_v = m_linearVelocity / (11.0 - abs(angleDifference));
            right_v = -m_linearVelocity / (11.0 - abs(angleDifference));
        } else {
            left_v = m_linearVelocity;
            right_v = -m_linearVelocity;
        }
    }
    ////  cout cout << "Left_v: " << : " << << " left_v <: " << right_v <<
    /// std::endl;
    m_pcWheels->SetLinearVelocity(left_v, right_v);
}

Real CFootBotDiffusion::ChangeCoordinateFromMapToArgos(Real x) {
    if (x == 0) {
        return 0;
    }
    return -x;
}

Real CFootBotDiffusion::ChangeCoordinateFromArgosToMap(Real x) {
    if (x == 0) {
        return 0;
    }
    return -x;
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")
