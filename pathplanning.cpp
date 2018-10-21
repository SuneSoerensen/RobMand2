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

int main(int argc, char** argv)
{
    //rw::math::Math::seed(427897842); //Seed 1
    rw::math::Math::seed(563758642); //Seed 2

    const string wcFile = "../Kr16WallWorkCell/Scene.wc.xml";
	const string deviceName = "KukaKr16";
	cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;

	WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
	Device::Ptr device = wc->findDevice(deviceName);
	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}
    /*const*/ State state = wc->getDefaultState();

    //Get the bottle-frame:
    Frame* bottle = wc->findFrame("Bottle");
    if(bottle == NULL)
        cout << "Bottle not found!" << endl;

    //Get the gripper-frame:
    Frame* gripper = wc->findFrame("Tool");
    if(gripper == NULL)
        cout << "Gripper not found!" << endl;


    //Attach bottle to gripper:
    Kinematics::gripFrame(bottle, gripper, state);

	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,device, state);

	/** Most easy way: uses default parameters based on given device
		sampler: QSampler::makeUniform(device)
		metric: PlannerUtil::normalizingInfinityMetric(device->getBounds())
        extend: 0.05 */
    //QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, device, RRTPlanner::RRTConnect);

	/** More complex way: allows more detailed definition of parameters and methods */
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    double extend = 0.1; //Originally = 0.1
    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

    Q from(6,-3.142, -0.827, -3.002, -3.143, 0.099, -1.573); //Pick
	//Q to(6,1.7,0.6,-0.8,0.3,0.7,-0.5); // Very difficult for planner
    Q to(6, 1.571, 0.006, 0.030, 0.153, 0.762, 4.490); //Place

	if (!checkCollisions(device, state, detector, from))
		return 0;
	if (!checkCollisions(device, state, detector, to))
		return 0;

	cout << "Planning from " << from << " to " << to << endl;
	QPath path;
	Timer t;
	t.resetAndResume();
	planner->query(from,to,path,MAXTIME);
	t.pause();
	cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << endl;
	if (t.getTime() >= MAXTIME) {
		cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
	}

    /*for (QPath::iterator it = path.begin(); it < path.end(); it++) {
		cout << *it << endl;
    }*/

    cout << "First element of path: " << path[0] << endl;
    cout << "Creating LUA script" << endl;
    createLUAScript("myLUAScript.lua", path);

	cout << "Program done." << endl;
	return 0;
}
