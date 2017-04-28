/*
 * Copyright (C) 2017 iCub Facility, IIT
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


#ifndef __ICUB_ARMTRACE__
#define __ICUB_ARMTRACE__

#include <yarp/os/all.h>
#include <iCub/eventdriven/all.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
#include <deque>

/*//////////////////////////////////////////////////////////////////////////////
  VBOTTLE READER/PROCESSOR
  ////////////////////////////////////////////////////////////////////////////*/

class vArmTraceController : public yarp::os::BufferedPort<ev::vBottle>
{
private:

    double medx;
    double medy;
    yarp::sig::Vector xrobref; //this stores the gaze position in eye ref frame
    yarp::sig::Vector px; //the pixel position to make a gaze

    yarp::dev::PolyDriver gazedriver;
    yarp::dev::IGazeControl *gazecontrol;

public:

    vArmTraceController();

    bool open(const std::string &name);
    void onRead(ev::vBottle &bot);
    void interrupt();
    void close();

};

/*//////////////////////////////////////////////////////////////////////////////
  MODULE
  ////////////////////////////////////////////////////////////////////////////*/

class vArmTraceModule : public yarp::os::RFModule
{
private:

    //the event bottle input and output handler
    vArmTraceController      tracecontrol;

    //the remote procedure port
    yarp::os::RpcServer     rpcPort;

public:

    //the virtual functions that need to be overloaded
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();

    virtual bool respond(const yarp::os::Bottle &command,
                         yarp::os::Bottle &reply);

};

#endif
