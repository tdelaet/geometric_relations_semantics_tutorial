#ifndef OROCOS_GEOMETRIC_SEMANTICS_COMPONENT_TUTORIAL_PUBLISHER_HPP
#define OROCOS_GEOMETRIC_SEMANTICS_COMPONENT_TUTORIAL_PUBLISHER_HPP

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

class Geometric_semantics_component_tutorial_publisher
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
    OutputPort<geometric_semantics_tf_msgs::PoseTF> outPortPose;
    geometric_semantics_tf_msgs::PoseTF propPose;

    /****************************
     transform
     ***************************/
    tf::Transform transform;
    tf::Transform object_coordinates_Pose;

    geometric_semantics::PoseCoordinatesSemantics object_PoseCoordinatesSemantics;
    geometric_semantics::Pose<tf::Pose> object_PoseTF;
    geometric_semantics_tf_msgs::PoseTF object_PoseTF_msg;

    double xpos, ypos, zpos;
    

    Geometric_semantics_component_tutorial_publisher(string const& name)
        : TaskContext(name)
        , object_PoseCoordinatesSemantics("a","a","A","b","b","B","b")
        , xpos(0)
        , ypos(0)
        , zpos(0)
    {
        object_PoseCoordinatesSemantics = PoseCoordinatesSemantics("e1","e1","E1","b1","b1","B1","b1");

        transform.setOrigin( tf::Vector3(1,2,3) );
        transform.setRotation(tf::createQuaternionFromRPY(0.2, 0, 0) );
        object_coordinates_Pose.setOrigin( tf::Vector3(1,2, 0.0) );
        object_coordinates_Pose.setRotation( tf::createQuaternionFromRPY(0.75, 0, 0) );

        //geometric_semantics::Pose<tf::Pose> object_PoseCoordinatesTF(conversions_geometric_msgs::PoseTFFromMsg(*msg))
        object_PoseTF = geometric_semantics::Pose<tf::Pose> (object_PoseCoordinatesSemantics, object_coordinates_Pose);

        propPose= conversions_geometric_msgs::PoseTFToMsg(object_PoseTF);

        /****************************
          Pose
         **************************/
        // Pose
        this->addPort( "outPortPose", outPortPose).doc( "Output Port for Pose." );
        this->addProperty( "propPose", propPose).doc( "Property for Pose." );

        log(Debug) << "Geometric_semantics_component_tutorial_publisher constructed !" <<endlog();

    }

    bool configureHook() {
        object_coordinates_Pose.setOrigin( tf::Vector3(1,2, 0.0) );
        object_coordinates_Pose.setRotation( tf::createQuaternionFromRPY(0.75, 0, 0) );
        object_PoseTF = geometric_semantics::Pose<tf::Pose> (object_PoseCoordinatesSemantics, object_coordinates_Pose);
        object_PoseTF_msg = conversions_geometric_msgs::PoseTFToMsg(object_PoseTF);

        outPortPose.setDataSample(object_PoseTF_msg);
        log(Debug) << "Geometric_semantics_component_tutorial_publisher configured !" <<endlog();
        return true;
    }

    bool startHook() {

        log(Debug) << "Geometric_semantics_component_tutorial_publisher started !" <<endlog();
        return true;
    }

    void updateHook() {
        xpos = xpos + 0.001;
        ypos = ypos + 0.001;
        zpos = zpos + 0.000;
        object_coordinates_Pose.setOrigin( tf::Vector3(xpos,ypos,zpos) );
        object_PoseTF = geometric_semantics::Pose<tf::Pose> (object_PoseCoordinatesSemantics, object_coordinates_Pose);
        object_PoseTF_msg = conversions_geometric_msgs::PoseTFToMsg(object_PoseTF);

        outPortPose.write(object_PoseTF_msg);
        log(Debug) << "Geometric_semantics_component_tutorial_publisher executes updateHook !" <<endlog();
    }

    void stopHook() {
        log(Debug) << "Geometric_semantics_component_tutorial_publisher executes stopping !" <<endlog();
    }

    void cleanupHook() {
        log(Debug) << "Geometric_semantics_component_tutorial_publisher cleaning up !" <<endlog();
    }
};

#endif
