#include "main.h"

std::vector<std::vector<Action>> readActionPlan(std::string path_filename)
{
    std::vector<std::vector<Action>> plans;
    double raw_plan_cost;
    if (path_filename == "none") {
        plans.clear();
        plans.resize(1);
        // 0, 2, 0, 1, 3, 0, 2, 3, 1, 0, 3, 0, 1, 3, 2, 1, 3, 2, 3, 1, 0, 2, 3, 0, 2, 0, 1, 3, 1, 0, 2, 3
        std::vector<double> all_angles = {};
        for (int i = 0; i < all_angles.size(); i++) {
            Action processedAction;
            processedAction.robot_id = 0;
            processedAction.time = i;
            processedAction.type = 'T';
            processedAction.start.first = 4;
            processedAction.start.second = 1;
            processedAction.goal.first = 4;
            processedAction.goal.second = 1;
            processedAction.orientation = all_angles[i];
            processedAction.nodeID = i;
            plans[0].push_back(processedAction);
        }
        // Action processedAction;
        // processedAction.robot_id = 0;
        // processedAction.time = 0;
        // processedAction.type = 'T';
        // processedAction.start.first = 1;
        // processedAction.start.second = 7;
        // processedAction.goal.first = 1;
        // processedAction.goal.second = 4;
        // processedAction.orientation = 2;
        // processedAction.nodeID = 0;
        // plans[0].push_back(processedAction);
        Action processedAction;
        processedAction.robot_id = 0;
        processedAction.time = 0;
        processedAction.type = 'M';
        processedAction.start.first = 1;
        processedAction.start.second = 7;
        processedAction.goal.first = 1;
        processedAction.goal.second = 6;
        processedAction.orientation = 0;
        processedAction.nodeID = 0;
        plans[0].push_back(processedAction);
        // Action processedAction2;
        // processedAction2.robot_id = 0;
        // processedAction2.time = 1;
        // processedAction2.type = 'M';
        // processedAction2.start.first = 1;
        // processedAction2.start.second = 7;
        // processedAction2.goal.first = 7;
        // processedAction2.goal.second = 7;
        // processedAction2.orientation = 1;
        // processedAction2.nodeID = 1;
        // plans[0].push_back(processedAction2);
        // Action process_rotate = processedAction;
        // process_rotate.nodeID = 2;
        // plans[0].push_back(processedAction);
        // Action processedAction3;
        // processedAction3.robot_id = 0;
        // processedAction3.time = 2;
        // processedAction3.type = 'M';
        // processedAction3.start.first = 7;
        // processedAction3.start.second = 7;
        // processedAction3.goal.first = 1;
        // processedAction3.goal.second = 7;
        // processedAction3.orientation = 1;
        // processedAction3.nodeID = 3;
        // plans[0].push_back(processedAction3);
    } else {
        bool success = parseEntirePlan(path_filename, plans, raw_plan_cost);
        if (not success){
            std::cerr << "Incorrect path, no ADG constructed! exiting ..." << std::endl;
            exit(-1);
        }
    }
    return plans;
}

std::vector<std::vector<Point>> readPlans(std::string path_filename)
{
    std::vector<std::vector<Point>> mapf_plan;
    ifstream inFile(path_filename);
    if (!inFile.is_open()) {
        cerr << "Failed to open the file." << endl;
        exit(-1);
    }
    string line;
    int agentId = 0;
    while (getline(inFile, line)) {
        std::vector<Step> tmp_plan;
        if (!line.empty()) {
            vector<Point> points = parseLine(line);
            mapf_plan.push_back(points);
        }
    }
    inFile.close();
    return mapf_plan;
}

int main(int argc, char **argv) {
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "produce help message")
            // params for the input instance and experiment settings
            ("path_file,p", po::value<string>(), "input file for path")
//            ("path_file,p", po::value<string>()->default_value("../data/maze-32-32-4_paths.txt"), "input file for path")
            ("port_number,n", po::value<int>()->default_value(8080), "rpc port number")
            ("output_file,o", po::value<string>()->default_value("stats.csv"), "output statistic filename")
            ("map_file,m", po::value<string>()->default_value("empty-8-8"), "map filename")
            ("scen_file,s", po::value<string>()->default_value("empty-8-8-random-1"), "scen filename")
            ("method_name", po::value<string>()->default_value("PBS"), "method we used")
            ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        cout << desc << endl;
        return 1;
    }
    po::notify(vm);
    std::string filename = "none";

    if (vm.count("path_file")) {
        filename = vm["path_file"].as<string>();
    }
    // std::cout << "Solving for path name: " << filename << std::endl;
    std::string out_filename = vm["output_file"].as<string>();

    rpc::client client("127.0.0.1", 8080);
    auto commit_cut = client.call("get_location", 2).as<std::vector<std::pair<double, double>>>();
    for (auto loc: commit_cut)
    {
        std::cout << loc.first << " " << loc.second << std::endl;
    }

    auto new_plan = readPlans(filename);
    std::vector<std::vector<std::tuple<int, int, double>>> msg_plan;
    for (auto& plan: new_plan)
    {
        std::vector<std::tuple<int, int, double>> tmp_msg_plan;
        for (auto& action: plan)
        {
            tmp_msg_plan.emplace_back(action.x, action.y, action.time);
        }
        msg_plan.push_back(tmp_msg_plan);
    }
    client.call("add_plan", msg_plan);
    return 0;
}