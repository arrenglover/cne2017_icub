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

    std::string vInPortName = "/" + name + "/vBottle:i";
    if(!yarp::os::BufferedPort<ev::vBottle>::open(vInPortName)) {
        std::cerr << "Could not open: " << vInPortName << std::endl;
        return false;
    }

    yarp::os::Property options;
    options.put("device", "gazecontrollerclient");
    options.put("local", "/" + name);
    options.put("remote", "/iKinGazeCtrl");
    gazedriver.open(options);
    if(gazedriver.isValid())
        gazedriver.view(gazecontrol);
    else
        std::cerr << "Gaze Driver not opened and will not be used" << std::endl;

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
    std::cout << "Closing Event Manager" << std::endl;
    yarp::os::BufferedPort<ev::vBottle>::close();
    std::cout << "Closed Event Manager" << std::endl;
}

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
        gazecontrol->get3DPoint(1, px, 1.0, xrobref);
        std::cout << px.toString() << " " << xrobref.toString() << std::endl;
    }

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
    std::string moduleName = rf.check("name", yarp::os::Value("vTrackToRobot")).asString();

    std::string rpcportname = "/" + moduleName + "/control";
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

