/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Arren.Glover@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "armtracing.h"
#include "yarp/math/Math.h"

using namespace yarp::math;
using namespace ev;

int main(int argc, char * argv[])
{

    /* initialize yarp network */
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    /* prepare and configure the resource finder */
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext( "eventdriven" );
    rf.setDefaultConfigFile( "armtracing.ini" );
    rf.configure( argc, argv );

    /* create the module */
    vArmTraceModule zmodule;

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    return zmodule.runModule(rf);

}

/*//////////////////////////////////////////////////////////////////////////////
  VBOTTLE READER/PROCESSOR
  ////////////////////////////////////////////////////////////////////////////*/

vArmTraceController::vArmTraceController()
{

    //inital gaze
    xrobref.resize(3);
    xrobref[0]=-0.4; //x = -0.4 (distance infront -ive)
    xrobref[1]=0; //y = 0 (left-right)
    xrobref[2]=0.3; //z = 0.3 (up/down)
    px.resize(2);
    px[0] = medy;
    px[1] = 127 - medx;

}


/******************************************************************************/
bool vArmTraceController::open(const std::string &name)
{
    //and open the input port

    this->useCallback();

    std::string vInPortName = name + "/vBottle:i";
    if(!yarp::os::BufferedPort<ev::vBottle>::open(vInPortName)) {
        std::cerr << "Could not open: " << vInPortName << std::endl;
        return false;
    }

    yarp::os::Property options;
    options.put("device", "gazecontrollerclient");
    options.put("local", name);
    options.put("remote", "/iKinGazeCtrl");
    gazedriver.open(options);
    if(gazedriver.isValid())
        gazedriver.view(gazecontrol);
    else {
        yError() << "Gaze Driver not opened and will not be used";
        return false;
    }
    options.put("device","cartesiancontrollerclient");
    // left arm
    //    options.put("remote","/icubSim/cartesianController/left_arm");
    //    options.put("local","/cartesian_client/left_arm");
    //right arm
    options.put("remote","/icub/cartesianController/right_arm");
    options.put("local","/cartesian_client/right_arm");

    // let's give the controller some time to warm up
    bool ok=false;
    double t0=yarp::os::Time::now();
    while (yarp::os::Time::now()-t0<10.0)
    {
        // this might fail if controller
        // is not connected to solver yet
        if (client.open(options))
        {
            ok=true;
            break;
        }

        yarp::os::Time::delay(1.0);
    }

    if (!ok)
    {
        yError()<<"Unable to open the Cartesian Controller";
        return false;
    }

    // open the view
    client.view(arm);

    // latch the controller context in order to preserve
    // it after closing the module
    // the context contains the dofs status, the tracking mode,
    // the resting positions, the limits and so on.
    arm->storeContext(&startup_context_id);

    // set trajectory time
    arm->setTrajTime(1.0);

    // get the torso dofs
    yarp::sig::Vector newDof, curDof;
    arm->getDOF(curDof);
    newDof=curDof;

    // enable the torso yaw and pitch
    // disable the torso roll
    newDof[0]=0;
    newDof[1]=0;
    newDof[2]=0;

    // send the request for dofs reconfiguration
    arm->setDOF(newDof,curDof);

    // impose some restriction on the torso pitch
    limitTorsoPitch();

    xd.resize(3);
    od.resize(4);

    yInfo()<<"Thread started successfully";

    return true;
}

void vArmTraceController::interrupt()
{
    std::cout << "Interrupting Manager" << std::endl;
    yarp::os::BufferedPort<ev::vBottle>::interrupt();
    std::cout << "Interrupted Manager" << std::endl;
}

void vArmTraceController::close()
{
    // we require an immediate stop
    // before closing the client for safety reason
    arm->stopControl();

    // it's a good rule to restore the controller
    // context as it was before opening the module
    arm->restoreContext(startup_context_id);

    client.close();
    std::cout << "Closing Event Manager" << std::endl;
    yarp::os::BufferedPort<ev::vBottle>::close();
    std::cout << "Closed Event Manager" << std::endl;
}

void vArmTraceController::limitTorsoPitch()
{
    int axis=0; // pitch joint
    double min, max;

    // sometimes it may be helpful to reduce
    // the range of variability of the joints;
    // for example here we don't want the torso
    // to lean out more than 30 degrees forward

    // we keep the lower limit
    arm->getLimits(axis,&min,&max);
    arm->setLimits(axis,min,MAX_TORSO_PITCH);
}

//void vArmTraceController::printStatus()
//{
//    if (t-t1>=PRINT_STATUS_PER)
//    {
//        yarp::sig::Vector x,o,xdhat,odhat,qdhat;

//        // we get the current arm pose in the
//        // operational space
//        if (!arm->getPose(x,o))
//            return;

//        // we get the final destination of the arm
//        // as found by the solver: it differs a bit
//        // from the desired pose according to the tolerances
//        if (!arm->getDesired(xdhat,odhat,qdhat))
//            return;

//        double e_x=norm(xdhat-x);
//        double e_o=norm(odhat-o);

//        yInfo()<<"+++++++++";
//        yInfo()<<"xd          [m] = "<<xd.toString();
//        yInfo()<<"xdhat       [m] = "<<xdhat.toString();
//        yInfo()<<"x           [m] = "<<x.toString();
//        yInfo()<<"od        [rad] = "<<od.toString();
//        yInfo()<<"odhat     [rad] = "<<odhat.toString();
//        yInfo()<<"o         [rad] = "<<o.toString();
//        yInfo()<<"norm(e_x)   [m] = "<<e_x;
//        yInfo()<<"norm(e_o) [rad] = "<<e_o;
//        yInfo()<<"---------";

//        t1=t;
//    }
//}



/******************************************************************************/
void vArmTraceController::onRead(vBottle &inputBottle)
{

    yarp::os::Stamp st;
    this->getEnvelope(st);

    //we just need to get our updated TS
    vQueue q = inputBottle.get<AE>();
    if(q.empty()) return;

    auto vc = is_event<AE>(q.back());

    px[0] = 303 - vc->x;
    px[1] = 239 - vc->y;
    //turn u/v into xyz
    if(gazedriver.isValid()) {
        gazecontrol->get3DPoint(1, px, 0.3, xrobref);
        std::cout << px.toString() << " " << xrobref.toString() << std::endl;
    }


    // we need to add an offset and possibly a scaling factor to x, to keep thehand in the reaching space of the arm without moving the torso so much


    // we need to add an offset to off-centre the movement to avoid interference with the torso and the movement backward

    // we keep the orientation of the left arm constant:
    // we want the middle finger to point forward (end-effector x-axis)
    // with the palm turned between down and right (end-effector y-axis points leftward);
    // to achieve that it is enough to rotate the root frame of pi around z-axis

    // left hand rotation
    // od[0]=0.0; od[1]=-0.5; od[2]=1.0; od[3]=M_PI;

    // right hand rotation
    od[0]=0.0; od[1]=-1.5; od[2]=1.0; od[3]=M_PI;

    // go to the target :)
    // (in streaming)
    //arm->goToPose(xrobref,od);
    arm->goToPosition(xrobref);

    // some verbosity
    // printStatus();

//    if(gazedriver.isValid() && dogaze && demo == graspdemo && gazingActive) {
//    //if(gazedriver.isValid() && demo == graspdemo && gazingActive) {

//        //this is the eye pose
//        yarp::sig::Vector xeye,oeye;
//        gazecontrol->getLeftEyePose(xeye,oeye);

//        //this does the transformation
//        yarp::sig::Matrix T=yarp::math::axis2dcm(oeye);
//        T(0,3)=xeye[0];
//        T(1,3)=xeye[1];
//        T(2,3)=xeye[2];
//        //std::cout << "initial rotation matrix" << std::endl;
//        //std::cout << T.toString() << std::endl;

//        //std::cout << "initial translations" << std::endl;
//        //std::cout << xeye.toString() << std::endl;

//        yarp::sig::Matrix Ti = yarp::math::SE3inv(T);
//        //std::cout << "inverted rotation matrix" << std::endl;
//        //std::cout << Ti.toString() << std::endl;


//        //this was the target in eye coordinates
//        yarp::sig::Vector fp(4);
//        fp[0]=xrobref[0];
//        fp[1]=xrobref[1];
//        fp[2]=xrobref[2];
//        fp[3]=1.0;

//        //std::cout << "Multiplied by" << std::endl;
//        //std::cout << fp.toString() << std::endl;


//        yarp::sig::Vector tp=Ti*fp;


//    }


}

/*//////////////////////////////////////////////////////////////////////////////
  MODULE
  ////////////////////////////////////////////////////////////////////////////*/

bool vArmTraceModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName = rf.check("name", yarp::os::Value("/vArmTracing")).asString();

    std::string rpcportname = moduleName + "/control";
    if(!rpcPort.open(rpcportname)) {
        std::cerr << "Could not open RPC port" << std::endl;
    }
    this->attach(rpcPort);

    if(!tracecontrol.open(moduleName)) {
        std::cerr << "Could Not Open arm tracer controller" << std::endl;
        return false;
    }

    return true ;
}

/******************************************************************************/
bool vArmTraceModule::interruptModule()
{
    tracecontrol.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/******************************************************************************/
bool vArmTraceModule::close()
{
    tracecontrol.close();
    yarp::os::RFModule::close();
    return true;
}

/******************************************************************************/
bool vArmTraceModule::updateModule()
{
    return true;
}

/******************************************************************************/
double vArmTraceModule::getPeriod()
{
    return 1;
}

bool vArmTraceModule::respond(const yarp::os::Bottle &command,
                                  yarp::os::Bottle &reply)
{
    reply.clear();

    if(command.get(0).asString() == "start") {
        reply.addString("starting");
        //this->vTrackToRobot.startGazing();
    } else if(command.get(0).asString() == "stop") {
        reply.addString("stopping");
        //this->vTrackToRobot.stopGazing();
    } else {
        return false;
    }

    return true;
}

