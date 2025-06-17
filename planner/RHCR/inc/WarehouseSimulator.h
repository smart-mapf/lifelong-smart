#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "KivaSystem.h"
#include "SortingSystem.h"
#include "OnlineSystem.h"
#include "BeeSystem.h"
#include "ManufactureSystem.h"
#include "ID.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>
#include <memory>
using json = nlohmann::json;

namespace py = pybind11;


class WarehouseSimulator{
    public:
        WarehouseSimulator(py::kwargs kwargs);
        ~WarehouseSimulator();
        std::string warmup();
        std::string update_gg_and_step(bool optimize_wait, std::vector<double> weights);
    private:
        BasicSystem* system;
        // KivaSystem* system;
        BasicGraph* G;
        // KivaGrid G;
        MAPFSolver* solver;
        std::string output;
        int warmup_time;
        int update_gg_interval;

};