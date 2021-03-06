#include <iostream>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/kinematics.hpp>
#include <fstream>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 10.
#define RUNS_PER_EPS 40


bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q)
{
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
	return true;
}

void createLUAScript(string fileName, QPath aPath)
{
    if(aPath.size() > 0) //Do not create a file for an empy path!
    {
        double sleepTime = 10.0 / aPath.size(); //Complete visualisation should take ~10s

        ofstream output(fileName);

        if(output.is_open())
        {
            output << "wc = rws.getRobWorkStudio():getWorkCell()" << endl;
            output << "state = wc:getDefaultState()" << endl;
            output << "device = wc:findDevice (\"KukaKr16\")" << endl;
            output << "gripper = wc:findFrame(\"Tool\");" << endl;
            output << "bottle = wc:findFrame(\"Bottle\");" << endl;
            output << "table = wc:findFrame(\"Table\");" << endl;
            output << endl;

            output << "function setQ(q)" << endl;
            output << "\t qq = rw.Q (#q , q[1] , q[2] , q[3] , q[4] , q[5] , q[6])" << endl;
            output << "\t device:setQ(qq , state )" << endl;
            output << "\t rws.getRobWorkStudio():setState(state)" << endl;
            output << "\t rw.sleep(" << sleepTime << ")" << endl;
            output << "end" << endl;
            output << endl;

            output << "function attach(obj, tool)" << endl;
            output << "rw.gripFrame(obj, tool, state)" << endl;
            output << "rws.getRobWorkStudio():setState(state)" << endl;
            output << "rw.sleep(0.5)" << endl;
            output << "end" << endl;
            output << endl;

            output << "setQ({" << aPath[0][0] << ", "  << aPath[0][1] << ", "  << aPath[0][2] << ", "  << aPath[0][3] << ", "  << aPath[0][4] << ", "  << aPath[0][5] << "})" << endl;
            output << "attach(bottle,gripper)" << endl;

            for(int i = 1; i < aPath.size()-1; i++)
            {
                output << "setQ({" << aPath[i][0] << ", "  << aPath[i][1] << ", "  << aPath[i][2] << ", "  << aPath[i][3] << ", "  << aPath[i][4] << ", "  << aPath[i][5] << "})" << endl;
            }

            output << "setQ({" << aPath[aPath.size()-1][0] << ", "  << aPath[aPath.size()-1][1] << ", "  << aPath[aPath.size()-1][2] << ", "  << aPath[aPath.size()-1][3] << ", "  << aPath[aPath.size()-1][4] << ", "  << aPath[aPath.size()-1][5] << "})" << endl;
            output << "attach(bottle,table)" << endl;

            output << endl;
            output.close();
        }
    }
}

void workerFunc(double eps, unsigned int aSeed)
{
    cout << "Worker with eps = " << eps << " started!" << endl;
    ofstream out("../data/" + to_string(eps) + "data.csv"); //For exporting data

    rw::math::Math::seed(aSeed); //Set the seed

    const string wcFile = "../Kr16WallWorkCell/Scene.wc.xml";
    const string deviceName = "KukaKr16";


    WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
    Device::Ptr device = wc->findDevice(deviceName);
    /*const*/ State state = wc->getDefaultState(); //Cannot be const, then bottle can't be grasped!

    //Get the bottle-frame:
    Frame* bottle = wc->findFrame("Bottle");

    //Get the gripper-frame:
    Frame* gripper = wc->findFrame("Tool");
    //Attach bottle to gripper:
    Kinematics::gripFrame(bottle, gripper, state);

    //Construct planner
    CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,device, state);

    /** More complex way: allows more detailed definition of parameters and methods */
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    double extend = eps;
    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

    Q from(6,-3.142, -0.827, -3.002, -3.143, 0.099, -1.573); //Pick
    Q to(6, 1.571, 0.006, 0.030, 0.153, 0.762, 4.490); //Place

    QPath path;
    Timer t;
    string filename;
    //Run a number of times for the given eps:
    for(int i = 0; i < RUNS_PER_EPS; i++)
    {
        t.resetAndResume();
        planner->query(from,to,path,MAXTIME);
        t.pause();

        out << i+1 << ", " << path.size() << ", " << t.getTime() << endl; //Export results to file
        filename = "../data/" + to_string(i) + "_eps" + to_string(eps) + ".lua";
        createLUAScript("../data/" + filename, path); //Create script for verifying path (in case of doubt)
        path.clear();
    }

    out.close();
    cout << "Worker with eps = " << eps << " done!" << endl;
}

int main(int argc, char** argv)
{
    unsigned int seeds[10] = {427897842, 563758642, 645370379, 436567765, 234567865, 236677678, 562896554, 120058763, 424242425, 629326};
    int seedIndex;

    //Stage 1:
    for(double eps = 1.0; eps <= 10.0; eps += 1.0)
    {
        workerFunc(eps, seeds[(int)eps-1]);
    }

    //Stage 2:
    for(double eps = 0.5; eps <= 1.5; eps += 0.1)
    {
        seedIndex = (eps - 0.5) / 0.1;
        cout << "Using seed no. " << seedIndex << endl;
        workerFunc(eps, seeds[seedIndex]);
    }

	return 0;
}
