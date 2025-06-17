#pragma once
#include <vector>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "common.h"
#include "States.h"
#include "BasicSystem.h"
#include "BasicGraph.h"
#include "MAPFSolver.h"
// #include<bits/stdc++.h>

namespace py = pybind11;

namespace helper {
    std::tuple<double, double> mean_std(std::vector<double> v);
    void divide(std::vector<double> &v, double factor);
    double sum(std::vector<double> v);
    vector<int> longest_common_subpath(
        vector<Path> &paths, int simulation_time);
    void set_parameters(BasicSystem& system, const py::kwargs& kwargs);
    MAPFSolver* set_solver(const BasicGraph& G, const py::kwargs& kwargs);
    std::tuple<string, vector<int>, vector<double>> setup_packages(
        const py::kwargs& kwargs);
    map<int, vector<int>> setup_chute_mapping(const py::kwargs& kwargs);
    std::tuple<string, vector<double>> setup_task_assign(
        const py::kwargs& kwargs);
}