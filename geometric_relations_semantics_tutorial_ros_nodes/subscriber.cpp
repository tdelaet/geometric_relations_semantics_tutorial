#include "ros/ros.h"

#include <geometric_semantics_tf_msgs_conversions.h>

#include <sstream>

//the incoming message
geometric_semantics::Pose<tf::Pose> T_in;

//the outcoming message
geometric_semantics::Pose<tf::Pose> T_out;

// allocate the geometric relations that you need
geometric_semantics::Pose<tf::Pose> T_interim1;
tf::Transform T_interim_coord1;
geometric_semantics::PoseCoordinatesSemantics T_interim_sem1;

geometric_semantics::Pose<tf::Pose> T_interim2;
tf::Transform T_interim_coord2;
geometric_semantics::PoseCoordinatesSemantics T_interim_sem2;

geometric_semantics::Pose<tf::Pose> T_interim3;
tf::Transform T_interim_coord3;
geometric_semantics::PoseCoordinatesSemantics T_interim_sem3;

geometric_semantics::Pose<tf::Pose> T_interim4;
tf::Transform T_interim_coord4;
geometric_semantics::PoseCoordinatesSemantics T_interim_sem4;


geometric_semantics::Pose<tf::Pose> T_interim5;
geometric_semantics::Pose<tf::Pose> T_interim6;
geometric_semantics::Pose<tf::Pose> T_interim7;
geometric_semantics::Pose<tf::Pose> T_interim8;
geometric_semantics::Pose<tf::Pose> T_interim9;

// the outgoing message - you should fill in this one
geometric_semantics_tf_msgs::PoseTF msg_PoseTF_out;

geometric_semantics::Pose<tf::Pose>   doGeometricSemanticsCalculations(const geometric_semantics::Pose<tf::Pose> pose)
{
    std::cout << "=============================================" << std::endl;
    T_interim5 = T_interim3.inverse2();
    std::cout << "T_interim5 " <<T_interim5.getSemantics() << std::endl;
    T_interim6 = compose(T_interim5,T_interim4);
    std::cout << "T_interim6 " <<T_interim6.getSemantics() << std::endl;
    T_interim7 = compose(T_interim6,T_interim2);
    std::cout << "T_interim7 " <<T_interim7.getSemantics() << std::endl;
    T_interim8 = compose(T_interim7,T_in);
    std::cout << "T_interim8 " <<T_interim8.getSemantics() << std::endl;
    T_interim9 = T_interim1.inverse2();
    std::cout << "T_interim9 " <<T_interim9.getSemantics() << std::endl;
    T_out = compose(T_interim8,T_interim9);
    std::cout << "=============================================" << std::endl;
    return T_out;
}

void chatterPoseTFCallback(const geometric_semantics_tf_msgs::PoseTF::ConstPtr& msg)
{
    T_in = geometric_semantics::Pose<tf::Pose>(conversions_geometric_msgs::PoseTFFromMsg(*msg));
    std::cout << " I heard: " << T_in.getSemantics() << std::endl;
    T_out = doGeometricSemanticsCalculations(T_in);
    msg_PoseTF_out= conversions_geometric_msgs::PoseTFToMsg(T_out);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_geometric_semantics_calculations");

    ros::NodeHandle n;

    // define the geometric relations that you need
    T_interim_coord1.setOrigin( tf::Vector3(1,2, 0.0) );
    T_interim_coord1.setRotation( tf::createQuaternionFromRPY(0.75, 0, 0) );
    T_interim_sem1 = geometric_semantics::PoseCoordinatesSemantics("b2","b2","B2","b1","b1","B1","b1");
    T_interim1 = geometric_semantics::Pose<tf::Pose> (T_interim_sem1, T_interim_coord1);

    // define the geometric relations that you need
    T_interim_coord2.setOrigin( tf::Vector3(1,2, 0.0) );
    T_interim_coord2.setRotation( tf::createQuaternionFromRPY(0.75, 0, 0) );
    T_interim_sem2 = geometric_semantics::PoseCoordinatesSemantics("o1","o1","O1","e1","e1","E1","e1");
    T_interim2 = geometric_semantics::Pose<tf::Pose> (T_interim_sem2, T_interim_coord2);

    // define the geometric relations that you need
    T_interim_coord3.setOrigin( tf::Vector3(1,2, 0.0) );
    T_interim_coord3.setRotation( tf::createQuaternionFromRPY(0.75, 0, 0) );
    T_interim_sem3 = geometric_semantics::PoseCoordinatesSemantics("o2","o2","O2","e2","e2","E2","e2");
    T_interim3 = geometric_semantics::Pose<tf::Pose> (T_interim_sem3, T_interim_coord3);

    // define the geometric relations that you need
    T_interim_coord4.setOrigin( tf::Vector3(1,2, 0.0) );
    T_interim_coord4.setRotation( tf::createQuaternionFromRPY(0.75, 0, 0) );
    T_interim_sem4 = geometric_semantics::PoseCoordinatesSemantics("o2","o2","O2","o1","o1","O1","o1");
    T_interim4 = geometric_semantics::Pose<tf::Pose> (T_interim_sem4, T_interim_coord4);

    //loop rate
    ros::Rate loop_rate(1);

    // subscriber
    ros::Subscriber subPoseTF = n.subscribe("/geometric_semantics_tutorial/pose", 1001, chatterPoseTFCallback);
    // publisher
    ros::Publisher pubPoseTF = n.advertise<geometric_semantics_tf_msgs::PoseTF>("/geometric_semantics_tutorial/pose_result", 1000);

    int count = 0;
    while (ros::ok())
    {
        pubPoseTF.publish(msg_PoseTF_out);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


