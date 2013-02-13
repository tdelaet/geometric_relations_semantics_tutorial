#ifndef OROCOS_GEOMETRIC_SEMANTICS_COMPONENT_TUTORIAL_SUBSCRIBER_HPP
#define OROCOS_GEOMETRIC_SEMANTICS_COMPONENT_TUTORIAL_SUBSCRIBER_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <iostream>


#include <Base/Primitives.h>
#include <Pose/Pose.h>
#include <Twist/Twist.h>
#include <Wrench/Wrench.h>
#include <Pose/PoseSemantics.h>
#include <Pose/PoseCoordinatesSemantics.h>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>

#include <geometric_semantics_tf_msgs/typekit/PoseTF.h>

#include "geometric_semantics_tf_msgs_conversions.h"


using namespace geometric_semantics;
using namespace RTT;
using namespace KDL;
using namespace std;

class Geometric_semantics_component_tutorial_subscriber
    : public RTT::TaskContext
{
 public:

    double propDouble;

    /****************************
      PRIMITIVES
      **************************/
  
    /****************************
     Pose
     **************************/
    //Pose
    InputPort<geometric_semantics_tf_msgs::PoseTF> inPortPose;
    geometric_semantics::PoseCoordinatesSemantics propPoseSemantics_expected;

    /****************************
     transform
     ***************************/
    tf::Transform transform;
    tf::Transform object_coordinates_Pose;

    geometric_semantics::PoseCoordinatesSemantics object_PoseTFCoordinatesSemantics;
    geometric_semantics::Pose<tf::Pose> object_PoseTF;
    geometric_semantics_tf_msgs::PoseTF object_PoseTF_msg;


    Geometric_semantics_component_tutorial_subscriber(string const& name)
        : TaskContext(name)
    {
        /****************************
          Pose
         **************************/
        // Pose
        this->addEventPort( "inPortPose", inPortPose ).doc( "Input Port for Pose that raises an event" );
        this->addProperty( "propPoseSemantics_expected", propPoseSemantics_expected).doc( "Property of expected semantics of received Pose." );

        log(Debug) << "Geometric_semantics_component_tutorial_subscriber constructed !" <<endlog();

    }

    bool configureHook() {
        log(Debug) << "Geometric_semantics_component_tutorial_subscriber configured !" <<endlog();
        return true;
    }

    bool startHook() {

        log(Debug) << "Geometric_semantics_component_tutorial_subscriber started !" <<endlog();
        return true;
    }

    void updateHook() {
        if ( inPortPose.read(object_PoseTF_msg) == NewData )
        {
            object_PoseTF= conversions_geometric_msgs::PoseTFFromMsg(object_PoseTF_msg);
            object_PoseTFCoordinatesSemantics = object_PoseTF.getSemantics();
        }
        log(Debug) << "Geometric_semantics_component_tutorial_subscriber executes updateHook !" <<endlog();
    }

    void stopHook() {
        log(Debug) << "Geometric_semantics_component_tutorial_subscriber executes stopping !" <<endlog();
    }

    void cleanupHook() {
        log(Debug) << "Geometric_semantics_component_tutorial_subscriber cleaning up !" <<endlog();
    }
};

#endif
