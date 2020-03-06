#include "ArchitectureSearch.h"
#include "ArchitectureSearchAnymal.h"
#include "ArchitectureSearchDummy.h"
#include "ArchitectureSearchStarleth.h"
#include "PythonTools.h"
#include <boost/python.hpp>

using namespace boost::python;

class PlannerEvaluatorAnymalPy : public arch::PlannerEvaluatorAnymal
{
public:
  void evaluatePy(const boost::python::list& vector) {
    std::vector<double> vec = toStdVector<double>(vector);
    evaluate(vec);
  }
  void savePy(const boost::python::list& vector, const std::string& filename, const std::string& extension) {
    std::vector<double> vec = toStdVector<double>(vector);
    save(vec, filename, extension);
  }
  boost::python::list getLowerBoundsPy() const {
    return toPythonList<double>(lb_);
  }
  boost::python::list getUpperBoundsPy() const {
    return toPythonList<double>(ub_);
  }
};

class PlannerEvaluatorDummyPy : public arch::PlannerEvaluatorDummy
{
public:
  void evaluatePy(const boost::python::list& vector) {
    std::vector<double> vec = toStdVector<double>(vector);
    evaluate(vec);
  }
  void savePy(const boost::python::list& vector, const std::string& filename, const std::string& extension) {
    std::vector<double> vec = toStdVector<double>(vector);
    save(vec, filename, extension);
  }
  boost::python::list getLowerBoundsPy() const {
    return toPythonList<double>(lb_);
  }
  boost::python::list getUpperBoundsPy() const {
    return toPythonList<double>(ub_);
  }
};

class PlannerEvaluatorStarlethPy : public arch::PlannerEvaluatorStarleth
{
public:
  void evaluatePy(const boost::python::list& vector) {
    std::vector<double> vec = toStdVector<double>(vector);
    evaluate(vec);
  }
  void savePy(const boost::python::list& vector, const std::string& filename, const std::string& extension) {
    std::vector<double> vec = toStdVector<double>(vector);
    save(vec, filename, extension);
  }
  boost::python::list getLowerBoundsPy() const {
    return toPythonList<double>(lb_);
  }
  boost::python::list getUpperBoundsPy() const {
    return toPythonList<double>(ub_);
  }
};

BOOST_PYTHON_MODULE(libArchitectureSearch)
{
  class_<PlannerEvaluatorAnymalPy>("PlannerEvaluatorAnymal")
    .def("setEnvironment",                &PlannerEvaluatorAnymalPy::setEnvironment)
    .def("setParamsBaseline",             &PlannerEvaluatorAnymalPy::setParamsBaseline)
    .def("setDebug",                      &PlannerEvaluatorAnymalPy::setDebug)
    .def("setRecast",                     &PlannerEvaluatorAnymalPy::setRecast)
    .def("setSeed",                       &PlannerEvaluatorAnymalPy::setSeed)
    .def("evaluate",                      &PlannerEvaluatorAnymalPy::evaluatePy)
    .def("save",                          &PlannerEvaluatorAnymalPy::savePy)
    .def("saveLast",                      &PlannerEvaluatorAnymalPy::saveLast)
    .def("generateRandomProblems",        &PlannerEvaluatorAnymalPy::generateRandomProblems)
    .def("getLowerBounds",                &PlannerEvaluatorAnymalPy::getLowerBoundsPy)
    .def("getUpperBounds",                &PlannerEvaluatorAnymalPy::getUpperBoundsPy)
    .def("getDimension",                  &PlannerEvaluatorAnymalPy::getDimension)
    .def("getLastEvaluationTime",         &PlannerEvaluatorAnymalPy::getLastEvaluationTime)
    .def("getLastEvaluationTimeMeasured", &PlannerEvaluatorAnymalPy::getLastEvaluationTimeMeasured)
    .def("getLastEvaluationCost",         &PlannerEvaluatorAnymalPy::getLastEvaluationCost)
    .def("getLastEvaluationSuccessRate",  &PlannerEvaluatorAnymalPy::getLastEvaluationSuccessRate)
  ;
  class_<PlannerEvaluatorDummyPy>("PlannerEvaluatorDummy")
    .def("setDebug",                      &PlannerEvaluatorDummyPy::setDebug)
    .def("evaluate",                      &PlannerEvaluatorDummyPy::evaluatePy)
    .def("save",                          &PlannerEvaluatorDummyPy::savePy)
    .def("saveLast",                      &PlannerEvaluatorDummyPy::saveLast)
    .def("generateRandomProblems",        &PlannerEvaluatorDummyPy::generateRandomProblems)
    .def("getLowerBounds",                &PlannerEvaluatorDummyPy::getLowerBoundsPy)
    .def("getUpperBounds",                &PlannerEvaluatorDummyPy::getUpperBoundsPy)
    .def("getDimension",                  &PlannerEvaluatorDummyPy::getDimension)
    .def("getLastEvaluationTime",         &PlannerEvaluatorDummyPy::getLastEvaluationTime)
    .def("getLastEvaluationTimeMeasured", &PlannerEvaluatorDummyPy::getLastEvaluationTimeMeasured)
    .def("getLastEvaluationCost",         &PlannerEvaluatorDummyPy::getLastEvaluationCost)
    .def("getLastEvaluationSuccessRate",  &PlannerEvaluatorDummyPy::getLastEvaluationSuccessRate)
  ;
  class_<PlannerEvaluatorStarlethPy>("PlannerEvaluatorStarleth")
    .def("setEnvironment",                &PlannerEvaluatorStarlethPy::setEnvironment)
    .def("setParamsBaseline",             &PlannerEvaluatorStarlethPy::setParamsBaseline)
    .def("setDebug",                      &PlannerEvaluatorStarlethPy::setDebug)
    .def("setRecast",                     &PlannerEvaluatorStarlethPy::setRecast)
    .def("setSeed",                       &PlannerEvaluatorStarlethPy::setSeed)
    .def("evaluate",                      &PlannerEvaluatorStarlethPy::evaluatePy)
    .def("save",                          &PlannerEvaluatorStarlethPy::savePy)
    .def("saveLast",                      &PlannerEvaluatorStarlethPy::saveLast)
    .def("generateRandomProblems",        &PlannerEvaluatorStarlethPy::generateRandomProblems)
    .def("getLowerBounds",                &PlannerEvaluatorStarlethPy::getLowerBoundsPy)
    .def("getUpperBounds",                &PlannerEvaluatorStarlethPy::getUpperBoundsPy)
    .def("getDimension",                  &PlannerEvaluatorStarlethPy::getDimension)
    .def("getLastEvaluationTime",         &PlannerEvaluatorStarlethPy::getLastEvaluationTime)
    .def("getLastEvaluationTimeMeasured", &PlannerEvaluatorStarlethPy::getLastEvaluationTimeMeasured)
    .def("getLastEvaluationCost",         &PlannerEvaluatorStarlethPy::getLastEvaluationCost)
    .def("getLastEvaluationSuccessRate",  &PlannerEvaluatorStarlethPy::getLastEvaluationSuccessRate)
  ;
}

