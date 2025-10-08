#include "heuristics/LandmarkHeuristicTable.h"

LandmarkHeuristicTable::LandmarkHeuristicTable(
    const BasicGraph& G, vector<int> task_locations, int seed, string log_dir,
    bool save_heuristics_table, int num_landmarks, string landmark_selection)
    : BasicHeuristicTable(G, task_locations, seed, log_dir,
                          save_heuristics_table) {
    // Select the landmarks based on the graph
    this->select_landmarks(num_landmarks, landmark_selection);
}

double LandmarkHeuristicTable::get(int goal_location, int start_location) {
    if (this->landmarks.empty()) {
        throw std::runtime_error(
            "LandmarkHeuristicTable::get(): No landmarks selected.");
    }

    double h = 0;
    for (const int l : this->landmarks) {
        // Check if we have already computed the heuristics from this landmark
        if (this->heuristics.find(l) == this->heuristics.end()) {
            spdlog::error(
                "LandmarkHeuristicTable::get(): Heuristics from landmark {} "
                "not found.",
                l);
        }

        // Obtain the h val of current landmark
        double d_landmark_to_goal = this->heuristics.at(l)[goal_location];
        double d_landmark_to_start = this->heuristics.at(l)[start_location];
        h = std::max(h, d_landmark_to_start - d_landmark_to_goal);
    }
    return h;
}

void LandmarkHeuristicTable::reset_heuristics() {
    heuristics.clear();
    // Compute heuristics for all task locations
    for (int loc : this->landmarks) {
        heuristics[loc] = compute_heuristics(loc);
    }
    // cout << this->save_path << endl;
    if (this->_save_heuristics_table)
        save_heuristics_table(this->save_path.string());
}

void LandmarkHeuristicTable::select_landmarks(int num_landmarks,
                                              string landmark_selection) {
    if (num_landmarks <= 0) {
        throw std::invalid_argument("Number of landmarks must be positive.");
    }

    if (num_landmarks > this->task_locations.size()) {
        spdlog::warn(
            "Number of landmarks {} is greater than number of task locations "
            "{}. Setting number of landmarks to {}.",
            num_landmarks, this->task_locations.size(),
            this->task_locations.size());
        num_landmarks = this->task_locations.size();
    }

    // Select landmarks randomly
    if (landmark_selection == "random") {
        vector<int> candidates(this->task_locations);
        std::shuffle(candidates.begin(), candidates.end(), this->rng);
        for (int i = 0; i < num_landmarks; i++)
            this->landmarks.push_back(candidates[i]);
    } else if (landmark_selection == "farthest") {
        throw std::runtime_error(
            "Farthest landmark selection not implemented yet.");
    }
    // Select all workstations and the top/bottom of each connected
    // task_loc component as the landmarks
    else if (landmark_selection == "workstation+endpt_corners") {
        // Find all the task location components that are connected in the
        // aisles. For each component, print the number of task_locs and the
        // locations.
        unordered_set<int> visited;
        for (const auto& task_loc : this->task_locations) {
            if (visited.find(task_loc) != visited.end()) {
                continue;  // Already visited this task_loc.
            }

            // Start a new component.
            set<int> component;
            component.insert(task_loc);
            visited.insert(task_loc);

            // Perform BFS to find all connected task_locs in the aisle.
            queue<int> q;
            q.push(task_loc);
            while (!q.empty()) {
                int current = q.front();
                q.pop();

                for (const auto& n : this->G.get_neighbors(current)) {
                    // Not visited and is a task location
                    if (visited.find(n) == visited.end() &&
                        std::find(this->task_locations.begin(),
                                  this->task_locations.end(),
                                  n) != this->task_locations.end()) {
                        visited.insert(n);
                        component.insert(n);
                        q.push(n);
                    }
                }
            }

            // Take the node in the compoent with the smallest and largest
            // id (which should be the "north/south most" task_loc) as the
            // landmarks.
            int s_landmark = *max_element(component.begin(), component.end());
            int n_landmark = *min_element(component.begin(), component.end());
            // this->landmarks.push_back(s_landmark);
            this->landmarks.push_back(n_landmark);

            // // Print the component information.
            // spdlog::info("Found task location component with {} locations. "
            //              "Northmost landmark: {}, Southmost landmark: {}.",
            //              component.size(), n_landmark, s_landmark);
        }
    } else {
        throw std::invalid_argument("Unsupported landmark selection method: " +
                                    landmark_selection);
    }

    std::cout << "Selected landmarks: ";
    for (int lm : this->landmarks) {
        std::cout << lm << ", ";
    }
    std::cout << std::endl;
}