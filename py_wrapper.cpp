#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "RVO2/src/RVO.h"
#include "RVO2/src/RVOSimulator.h"
#include  "Eigen/Dense"
#include <memory>
namespace py = pybind11;
using namespace RVO;
PYBIND11_MODULE(pyrvo2,m){
    m.doc()="pybind for     rvo2";
    py::class_<RVOSimulator,std::shared_ptr<RVOSimulator>>(m,"RVOSimulator")
    .def(py::init<>())
    .def("setTimeStep",&RVOSimulator::setTimeStep)
    .def("step",&RVOSimulator::doStep)
    .def("setAgentDefaults",&RVOSimulator::setAgentDefaults,
    py::arg("neighborDist"), py::arg("maxNeighbors"), py::arg("timeHorizon"),  py::arg("timeHorizonObst"),  py::arg("radius"),   py::arg("maxSpeed"),  py::arg("velocity"))
    .def("addAgent",static_cast<size_t (RVOSimulator::*)(Eigen::Vector2f &)>(&RVOSimulator::addAgent),py::arg("position"))
    .def("addObstacle",&RVOSimulator::addObstacle,py::arg("obstacle_vertices"))
    .def("processObstacle",&RVOSimulator::processObstacles)
    .def("getAgentPosition",&RVOSimulator::getAgentPosition,py::arg("agentNo"))
    .def("getAgentVelocity",&RVOSimulator::getAgentVelocity,py::arg("agentNo"))
    .def("setAgentVelocity",&RVOSimulator::setAgentVelocity,py::arg("agentNo"),py::arg("velocity"))
    .def("setAgentPosition",&RVOSimulator::setAgentPosition,py::arg("agentNo"),py::arg("position"))
    .def("getAgentRadius",&RVOSimulator::getAgentRadius,py::arg("agentNo"))
    .def("setAgentRadius",&RVOSimulator::setAgentRadius,py::arg("agentNo"),py::arg("radius"))
    .def("setAgentPrefVelocity",&RVOSimulator::setAgentPrefVelocity,py::arg("agentNo"),py::arg("prefVelocity"))
    .def("getAgentPrefVelocity",&RVOSimulator::getAgentPrefVelocity,py::arg("agentNo"))
    .def("getAgentTimeHorizon",&RVOSimulator::getAgentTimeHorizon,py::arg("agentNo"))
    .def("getAgentTimeHorizon",&RVOSimulator::setAgentTimeHorizon,py::arg("agentNo"),py::arg("timeHorizon"))
    .def("getAgentTimeHorizonObst",&RVOSimulator::setAgentTimeHorizonObst,py::arg("agentNo"),py::arg("timeHorizon"))
    .def("getAgentTimeHorizonObst",&RVOSimulator::getAgentTimeHorizonObst,py::arg("agentNo"))
    .def("getNumAgents",&RVOSimulator::getNumAgents)
    ;
}