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
    std::string vOutPortName = "/" + name + "/vBottle:o";
    if(!eventsOutPort.open(vOutPortName)) {
        std::cerr << "Could not open: " << vOutPortName << std::endl;
        return false;
    }

    std::string cartPortName = "/" + name + "/vCartOut:o";
    if(!cartOutPort.open(cartPortName)) {
        std::cerr << "Could not open: " << cartPortName << std::endl;
        return false;
    }

    std::string scopePortName = "/" + name + "/scope:o";
    if(!scopeOutPort.open(scopePortName)) {
        std::cerr << "Could not open: " << scopePortName << std::endl;
        return false;
    }

    std::string positionPortName = "/" + name + "/posdump:o";
    if(!positionOutPort.open(positionPortName)) {
        std::cerr << "Could not open: " << positionPortName << std::endl;
        return false;
    }


    //if(method != fromgaze) return true;

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

    auto vc = is_event<GaussianAE>(q.back());

    //get the stamp
    bestts = vc->stamp;

        //get radius
        //p_eyez = (-2.5 * vc->getXSigma2() + 70)/100.0;
        p_eyez = vc->sigxy;
        //p_eyez = std::min(p_eyez, 16.0);

        //update our window
//        ev::event<ev::AddressEvent> v = ev::event<ev::AddressEvent>(new ev::AddressEvent());
//        v->setChannel(vc->getChannel());
//        v->setPolarity(vc->getChannel());
//        v->setX(vc->getXCog());
//        v->setY(vc->getYCog());
//        FIFO.addEvent(v);

//        //and then get everything in the current window
//        q = FIFO.getSurf();
//        n = q.size();

//        //compute the median
//        std::vector<int> xs, ys;
//        xs.resize(n); ys.resize(n);
//        for(int i = 0; i < n; i++) {
//            ev::event<ev::AddressEvent> vtw = ev::as_event<ev::AddressEvent>(q[i]);
//            xs[i] = 303 - vtw->x;
//            ys[i] = 239 - vtw->y;
//            //p_eyez = std::max(p_eyez, (double)vtw->getXSigma2());
//            //p_eyez = std::min(p_eyez, 16.0);
//        }

//        std::sort(xs.begin(), xs.end());
//        std::sort(ys.begin(), ys.end());

//        medx = xs[n / 2];
//        medy = ys[n / 2];

        //do error check for too much noise
//        double medstdx = 0, medstdy = 0;
//        for(int i = 0; i < n; i++) {
//            medstdx += pow(xs[i] - medx, 2.0);
//            medstdy += pow(ys[i] - medy, 2.0);
//        }
//        medstdx = sqrt(medstdx / n);
//        medstdy = sqrt(medstdy / n);

        //if(std::abs(vc->getXCog() - medx) < medstdx && std::abs(vc->getYCog() - medy) < medstdy) {
            //std::cout << "current observation within 1 std" << std::endl;
            //std::cout << medstdx << " " << medstdy << " " << n << std::endl;
            //if(medstdx < 10 && medstdy < 10 && n > 5) {
                dogaze = true;
                lastdogazetime = yarp::os::Time::now();
                px[0] = 303 - vc->x;
                px[1] = 239 - vc->y;
                //turn u/v into xyz
                if(gazedriver.isValid()) {
                    //gazecontrol->get3DPoint(0, px, (-2.4 * p_eyez + 70)/100.0, xrobref);
                    double zpos = -0.02 * p_eyez + 0.8;
                    zpos = std::min(zpos, 0.5);
                    zpos = std::max(zpos, 0.3);

                    gazecontrol->get3DPoint(1, px, zpos, xrobref);
                    std::cout << px.toString() << " " << xrobref.toString() << std::endl;
                }
            //}
        //}

    }


    //DO GAZE
    //find the median position in xyz space and gaze there
    //yarp::sig::Vector px(2);        //pixel in uv
    //yarp::sig::Vector x(3); x = 0;  //position in xyz (iCub ref frame)
    //px[0] = medy;
    //px[1] = 127 - medx;
    if(gazedriver.isValid() && dogaze) {

        //if we use the gaze controller to gaze then go ahead
        if(demo == gazedemo && gazingActive)
            gazecontrol->lookAtFixationPoint(xrobref);
    }

    if(gazedriver.isValid() && dogaze && demo == graspdemo && gazingActive) {
    //if(gazedriver.isValid() && demo == graspdemo && gazingActive) {

        //this is the eye pose
        yarp::sig::Vector xeye,oeye;
        gazecontrol->getLeftEyePose(xeye,oeye);

        //this does the transformation
        yarp::sig::Matrix T=yarp::math::axis2dcm(oeye);
        T(0,3)=xeye[0];
        T(1,3)=xeye[1];
        T(2,3)=xeye[2];
        //std::cout << "initial rotation matrix" << std::endl;
        //std::cout << T.toString() << std::endl;

        //std::cout << "initial translations" << std::endl;
        //std::cout << xeye.toString() << std::endl;

        yarp::sig::Matrix Ti = yarp::math::SE3inv(T);
        //std::cout << "inverted rotation matrix" << std::endl;
        //std::cout << Ti.toString() << std::endl;


        //this was the target in eye coordinates
        yarp::sig::Vector fp(4);
        fp[0]=xrobref[0];
        fp[1]=xrobref[1];
        fp[2]=xrobref[2];
        fp[3]=1.0;

        //std::cout << "Multiplied by" << std::endl;
        //std::cout << fp.toString() << std::endl;


        yarp::sig::Vector tp=Ti*fp;
        //std::cout << "Equals:" << std::endl;
        //std::cout << tp.toString() << std::endl;
        if(cartOutPort.getOutputCount()) {
            yarp::os::Bottle &cartcoords = cartOutPort.prepare();
            cartcoords.clear();
            //    //add the XYZ position
            cartcoords.add(tp[0]); cartcoords.add(tp[1]); cartcoords.add(tp[2]);
            //cartcoords.add(-1.0); cartcoords.add(0.0); cartcoords.add(-0.3);
            //    //add some buffer ints
            cartcoords.add(0.5); cartcoords.add(px[0]); cartcoords.add(px[1]);
            //    //flag that the object is detected
            cartcoords.add(1.0);

            //std::cout << "Bottle: " << cartcoords.toString() << std::endl;
            //targetPos in the eye reference frame
            //std::cout << "2D point: " << px.toString() << std::endl;
            //std::cout << "3D point: " << x.toString() << std::endl;
            cartOutPort.write();
        }

    }


    //DUMP POSITIONS
    //find the position of the eyes in the current position
    yarp::sig::Vector cpx(2); cpx[0] = 64; cpx[1] = 64;
    yarp::sig::Vector cx(3); cx = 0;  //position in xyz (eye ref frame)
    if(gazedriver.isValid()) {
        gazecontrol->get3DPoint(0, cpx, (-2.4 * p_eyez + 70)/100.0, cx);
    }
    if(positionOutPort.getOutputCount()) {
        yarp::os::Bottle &posdump = positionOutPort.prepare();
        posdump.clear();
        posdump.addInt(bestts);
        posdump.addDouble(cx[0]); posdump.addDouble(cx[1]); posdump.addDouble(cx[2]);
        posdump.addDouble(xrobref[0]); posdump.addDouble(xrobref[1]); posdump.addDouble(xrobref[2]);
        positionOutPort.setEnvelope(st);
        positionOutPort.write();
    }

    return;

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

