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

            if( object_PoseTFCoordinatesSemantics.getPoint().getName() != propPoseSemantics_expected.getPoint().getName() )
            {
                log(Error) << "The received pose with semantics " << object_PoseTFCoordinatesSemantics << " does not have expected point " << propPoseSemantics_expected.getPoint() <<endlog();
            }
            if( object_PoseTFCoordinatesSemantics.getOrientationFrame().getName() != propPoseSemantics_expected.getOrientationFrame().getName() )
            {
                log(Error) << "The received pose with semantics " << object_PoseTFCoordinatesSemantics << " does not have expected orientation frame " << propPoseSemantics_expected.getOrientationFrame() <<endlog();
            }
            if( object_PoseTFCoordinatesSemantics.getBody().getName() != propPoseSemantics_expected.getBody().getName() )
            {
                log(Error) << "The received pose with semantics " << object_PoseTFCoordinatesSemantics << " does not have expected body " << propPoseSemantics_expected.getBody() <<endlog();
            }
            if( object_PoseTFCoordinatesSemantics.getRefPoint().getName() != propPoseSemantics_expected.getRefPoint().getName() )
            {
                log(Error) << "The received pose with semantics " << object_PoseTFCoordinatesSemantics << " does not have expected reference point " << propPoseSemantics_expected.getRefPoint() <<endlog();
            }
            if( object_PoseTFCoordinatesSemantics.getRefOrientationFrame().getName() != propPoseSemantics_expected.getRefOrientationFrame().getName() )
            {
                log(Error) << "The received pose with semantics " << object_PoseTFCoordinatesSemantics << " does not have expected reference orientation frame " << propPoseSemantics_expected.getRefOrientationFrame() <<endlog();
            }
            if( object_PoseTFCoordinatesSemantics.getRefBody().getName() != propPoseSemantics_expected.getRefBody().getName() )
            {
                log(Error) << "The received pose with semantics " << object_PoseTFCoordinatesSemantics << " does not have expected reference body " << propPoseSemantics_expected.getRefBody() <<endlog();
            }
            if( object_PoseTFCoordinatesSemantics.getCoordinateFrame().getName() != propPoseSemantics_expected.getCoordinateFrame().getName() )
            {
                log(Error) << "The received pose with semantics " << object_PoseTFCoordinatesSemantics << " does not have expected coordinate frame " << propPoseSemantics_expected.getCoordinateFrame() <<endlog();
            }
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
