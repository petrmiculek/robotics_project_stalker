#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include "std_msgs/Bool.h"

#define waiting_for_a_person 0
#define observing_the_person 1
#define rotating_to_the_person 2
#define moving_to_the_person 3
#define interacting_with_the_person 4
#define rotating_to_the_base 5
#define moving_to_the_base 6
#define resetting_orientation 7
#define interaction_distance 0.5
#define min_angle 0.1

// Numbers of ticks to advance the user interaction
#define frequency_expected 25           // unused
#define frequency_expected_observing 25 // ticks of having observed a person until we start rotating towards them
#define frequency_expected_rotating_to_person 10   // ticks when rotated towards the person until we start moving towards them
#define frequency_expected_moving_to_person 5 // 
// note: interacting with the person has no timeout
#define frequency_expected_rotating_to_base 5 // ticks when rotated towards the base until we start moving towards it
#define frequency_expected_trasnlation_to_base 5
#define frequency_expected_orientation_at_base 5 // ticks when rotated towards the base until we start moving towards it

// Numbers of ticks to give up the user interaction

#define frequency_giveup_rotated 10           // person lost during ROTATING towards them
#define frequency_giveup_moving_to_person 5   // person lost during MOVING TOWARDS them
#define frequency_giveup_interacting_with_person 20   // person lost or walking away during INTERACTING with them

#define frequency_min 0
// #define frequency_max 25 // value not important, just prevent counting to infinity - overflows

#define max_base_distance 6 // in meters


// Transofrm the position of the base in the cartesian local frame of robot
// Input: base_position: position of the base in the map frame
//        current_position: position of the robot in the map frame
//        orientation: orientation of the robot in the map frame
// Output: new_base_point: position of the base in the local frame of the robot
geometry_msgs::Point transformPoint(geometry_msgs::Point &base, geometry_msgs::Point &current_pos, float orientation)
{
    geometry_msgs::Point new_base_point;
    float x = base.x - current_pos.x;
    float y = base.y - current_pos.y;

    new_base_point.x = x * cos(orientation) + y * sin(orientation);
    new_base_point.y = -x * sin(orientation) + y * cos(orientation);
    return new_base_point;
}

// Keep the value passed as parameter withing the unit circle
// Input: orientation of the robot in the map frame
// Output: orientation of the robot in the map frame, within the unit circle 
float clamp(float orientation)
{
    if (orientation > M_PI)
        return orientation - 2 * M_PI * (int)(orientation / (2 * M_PI));

    if (orientation < -M_PI)
        return orientation + 2 * M_PI * (int)(orientation / (2 * M_PI));

    return orientation;
}

class decision_node
{
private:
    ros::NodeHandle n;

    // communication with datmo_node
    ros::Subscriber sub_person_position;
    bool new_person_position, person_tracked;
    geometry_msgs::Point person_position;

    // communication with robot_moving_node
    ros::Subscriber sub_robot_moving;
    bool robot_moving;

    // communication with rotation_action
    ros::Publisher pub_rotation_to_do;
    float rotation_to_person;

    // communication with action_node
    ros::Publisher pub_goal_to_reach;
    float translation_to_person;

    // communication with localization
    ros::Subscriber sub_localization;
    bool new_localization;
    bool init_localization;
    geometry_msgs::Point current_position;
    float current_orientation;
    float translation_to_base;
    float rotation_to_base;
    geometry_msgs::Point local_base_position;

    int current_state, previous_state;
    int frequency;
    /*
    uses of frequency:
    - observing_the_person: if frequency > frequency_expected_observing, we start rotating towards the person
    - rotating_to_the_person: if frequency > frequency_expected_rotating_to_person, we start moving towards the person
    -
    */
    int frequency_giveup; // person lost
    // int frequency_rot_person_giveup;
    geometry_msgs::Point base_position;
    float base_orientation;
    geometry_msgs::Point robot_position; // renamed from origin_position, always [0, 0]
    bool state_has_changed;

public:
    decision_node()
    {

        // communication with datmo_node
        sub_person_position = n.subscribe("person_position", 1, &decision_node::person_positionCallback, this);

        // communication with rotation_node
        pub_rotation_to_do = n.advertise<std_msgs::Float32>("rotation_to_do", 0);

        // communication with action_node
        pub_goal_to_reach = n.advertise<geometry_msgs::Point>("goal_to_reach", 1); // Preparing a topic to publish the position of the person

        // communication with robot_moving_node
        sub_robot_moving = n.subscribe("robot_moving", 1, &decision_node::robot_movingCallback, this);

        // communication with localization node
        sub_localization = n.subscribe("localization", 1, &decision_node::localizationCallback, this);

        current_state = waiting_for_a_person;
        previous_state = -1;

        new_person_position = false;
        state_has_changed = false;

        // TO DEFINE according to the position of the base/initial position in the map
        base_position.x = 0;
        base_position.y = 0;
        base_orientation = 0;

        robot_position.x = 0;
        robot_position.y = 0;

        person_tracked = false;

        // INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
        ros::Rate r(10); // this node will work at 10hz
        while (ros::ok())
        {
            ros::spinOnce(); // each callback is called once
            update();
            r.sleep(); // we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
        }
    }

    // UPDATE: main processing of laser data
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void update()
    {
        /*
        // MOCK LOCALIZATION - use only for local testing
        const geometry_msgs::PointConstPtr mock_position = boost::make_shared<geometry_msgs::Point>(robot_position);
        localizationCallback(mock_position);
        */

        if (init_localization)
        {

            update_variables();

            // DO NOT FORGET that if robair is too far from its base (ie, its distance to the base is higher than max_base_distance),
            // robair should stop interacting with the moving person and go back to its base
            switch (current_state)
            {
            case waiting_for_a_person:
                process_waiting_for_a_person();
                break;

            case observing_the_person:
                process_observing_the_person();
                break;

            case rotating_to_the_person:
                process_rotating_to_the_person();
                break;

            case moving_to_the_person:
                process_moving_to_the_person();
                break;

            case interacting_with_the_person:
                process_interacting_with_the_person();
                break;

            case rotating_to_the_base:
                process_rotating_to_the_base();
                break;

            case moving_to_the_base:
                process_moving_to_the_base();
                break;

            case resetting_orientation:
                process_resetting_orientation();
                break;
            }

            new_localization = false;
            new_person_position = false;

            state_has_changed = current_state != previous_state;
            previous_state = current_state;
        }
        else
            ROS_WARN("Initialize localization");

    } // update

    void update_variables()
    {

        if ( new_person_position )
        {
            translation_to_person = distancePoints(robot_position, person_position);

            if ( translation_to_person > 0 )
            {
                rotation_to_person = acos( person_position.x / translation_to_person );
                if ( person_position.y < 0 )
                    rotation_to_person *=-1;
            }
            else
                rotation_to_person = 0;

            person_tracked = person_position.x != 0 || person_position.y != 0;
        }        

        if ( new_localization )
        {
            // when we receive a new position(x, y, o) of robair in the map, we update:
            // translation_to_base: the translation that robair has to do to reach its base
            // rotation_to_base: the rotation that robair has to do to reach its base
            // local_base_position: the position of the base in the cartesian local frame of robot

            translation_to_base = distancePoints(current_position, base_position);
            local_base_position = transformPoint(base_position, current_position, current_orientation);
            
            rotation_to_base = acos(local_base_position.x / translation_to_base) ;
            if ( local_base_position.y < 0 )
                rotation_to_base *= -1;

        }

    }

    void process_waiting_for_a_person()
    {

        if (state_has_changed)
        {
            ROS_INFO("current_state: waiting_for_a_person");
            ROS_INFO("press enter to continue");
            // getchar();
        }

        // Processing of the state
        // as soon as we detect a moving person, we switch to the state "observing_the_person"
        if (person_tracked)
            current_state = observing_the_person;
    }

    /**
     * @brief process_observing_the_person is the state where robair only observes and tracks the moving person
     *
     *
     */
    void process_observing_the_person()
    {

        if (state_has_changed)
        {
            ROS_INFO("current_state: observing_the_person");
            ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
            ROS_INFO("press enter to continue");
            // getchar();
            frequency = 0;
        }

        // Processing of the state
        // Robair only observes and tracks the moving person
        // if the moving person does not move during a while (use frequency), we switch to the state "rotating_to_the_person"
        if (person_tracked)
        {
            ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
            frequency = std::min(frequency_expected_observing, frequency + 1);
            if (frequency == frequency_expected_observing)
            {
                current_state = rotating_to_the_person;
            }
        }
        else
        {
            frequency = std::max(frequency_min, frequency - 1);
            // what should robair do if it loses the moving person ?
            if (frequency == frequency_min)
            {
                current_state = waiting_for_a_person;
                // rotation was not changed, so no need for resetting_orientation
            }
        }
    }

    void process_rotating_to_the_person()
    {
        if (state_has_changed)
        {
            ROS_INFO("current_state: rotating_to_the_person");
            ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
            ROS_INFO("press enter to continue");
            getchar();
            frequency = 0;
            frequency_giveup = 0;
        }

        // Processing of the state
        // Robair rotates to be face to the moving person
        // if robair is facing towards the moving person and the moving person does not move during a while,
        // we switch to the state "moving_to_the_person"
        if (person_tracked)
        {
            ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
            if (fabs(rotation_to_person) < min_angle)
            {
                frequency = std::min(frequency_expected_rotating_to_person, frequency + 1);
                if (frequency == frequency_expected_rotating_to_person)
                {
                    current_state = moving_to_the_person;
                }
            }
            else
            {
                // rotate towards the person
                std_msgs::Float32 rot_msg = std_msgs::Float32();
                rot_msg.data = rotation_to_person;
                pub_rotation_to_do.publish(rot_msg);

                // what if we rotate, but the person keeps moving back and forth 
                // this situation currently not handled
                // suggestion - have another timer to give up if the person doesn't stop moving
            }
        }
        else
        {

            // person lost - timeout until we give up
            frequency_giveup = std::min(frequency_giveup_rotated, frequency_giveup + 1);
            if (frequency_giveup == frequency_giveup_rotated)
            {
                // give up, reset orientation
                current_state = resetting_orientation;
            }
        }
    }

    void process_moving_to_the_person()
    {

        if (state_has_changed)
        {
            ROS_INFO("current_state: moving_to_the_person");
            ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
            ROS_INFO("press enter to continue");
            // getchar();
            frequency = 0;
            frequency_giveup = 0;
        }

        // what should robair do if it is too far from home ?
        if (distancePoints(current_position, base_position) > max_base_distance)
        {
            current_state = rotating_to_the_base;
            return;
            // note: changed from `resetting_orientation` - that happens when we're home
        }

        // Processing of the state
        // Robair moves to be close to the moving person
        // if robair is close to the moving person and the moving person does not move during a while 
        // (use frequency), we switch to the state "interacting_with_the_person"
        if (person_tracked)
        {
            ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);

            // set goal to reach - the person
            pub_goal_to_reach.publish(person_position);

            if (fabs(translation_to_person) < interaction_distance)
            {
                frequency = std::min(frequency_expected_moving_to_person, frequency + 1);
                if (frequency == frequency_expected_moving_to_person)
                {
                    current_state = interacting_with_the_person;
                }
            }
            // what should robair do if it is close to the moving person but the moving person does not stop ?
            // currently not handled
            // suggestion - give up after: a) timeout, or b) distance to person is increasing
        }
        else
        {
            // what should robair do if it loses the moving person ?
            frequency_giveup = std::min(frequency_giveup_moving_to_person, frequency + 1);
            if (frequency_giveup == frequency_giveup_moving_to_person)
            {
                // give up, go home
                current_state = rotating_to_the_base;
            }
        }
    }

    void process_interacting_with_the_person()
    {
        if (state_has_changed)
        {
            ROS_INFO("current_state: interacting_with_the_person");
            ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
            ROS_INFO("press enter to continue");
            // getchar();
            frequency = 0;
            frequency_giveup = 0;
        }

        // Processing of the state
        // Robair does not move and interacts with the moving person until the moving person goes away from robair
        // if the person goes away from robair, after a while (use frequency), we switch to the state "rotating_to_the_base"
        if (person_tracked)
        {
            ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
            if (fabs(translation_to_person) <= interaction_distance)
            {
                // interaction going on - do nothing
            }
            else {
                // person walking away
                frequency_giveup = std::min(frequency_giveup_interacting_with_person, frequency_giveup + 1);
                if (frequency_giveup == frequency_giveup_interacting_with_person)
                {
                    // person gone, go home
                    current_state = rotating_to_the_base;
                }
            }
        }
        else {
            // what should robair do if it loses the moving person ?

            // the timeout is currently shared for person walking away and completely lost
            frequency_giveup = std::min(frequency_giveup_interacting_with_person, frequency_giveup + 1);
            if (frequency_giveup == frequency_giveup_interacting_with_person)
            {
                // give up, go home
                current_state = rotating_to_the_base;
            }
        }
    }

    void process_rotating_to_the_base()
    {

        if (state_has_changed)
        {
            ROS_INFO("current_state: rotating_to_the_base");
            ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation * 180 / M_PI);
            ROS_INFO("press enter to continue");
            // getchar();
            frequency = 0;
        }

        // Processing of the state
        // Robair rotates to be face to its base
        // if robair is face to its base and does not move, after a while (use frequency), we switch to the state "moving_to_the_base"
        if (new_localization)
        {
            ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation * 180 / M_PI);
        }
        ROS_INFO("Position of base: (%f, %f, %f)" ,local_base_position.x , local_base_position.y,rotation_to_base* 180 / M_PI );
        //0.1 rad ~= 5.7 deg
        if (fabs(rotation_to_base) < min_angle)
        {
            frequency = std::min(frequency_expected_rotating_to_base, frequency + 1);
            if (frequency == frequency_expected_rotating_to_base)
            {
                current_state = moving_to_the_base;
            }
        }
        else
        {
            std_msgs::Float32 rot_msg = std_msgs::Float32();
            rot_msg.data = rotation_to_base;
            pub_rotation_to_do.publish(rot_msg);
        }
    }

    void process_moving_to_the_base()
    {

        if (state_has_changed)
        {
            ROS_INFO("current_state: moving_to_the_base");
            ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation * 180 / M_PI);
            ROS_INFO("press enter to continue");
            // getchar();
            frequency = 0;
        }

        // Processing of the state
        // Robair moves to its base
        // if robair is close to its base and does not move, after a while (use frequency), we switch to the state "resetting_orientation"

        if ( new_localization )
        {
            ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation*180/M_PI);
        }
        //If the translation to do is less than or equal to 5cm, this indicates that the robot is close
        if(translation_to_base <= 0.005){
            frequency = std::min(frequency_expected_trasnlation_to_base, frequency + 1);
            //After a while, if the translation to do is less than or equal to 5cm, then we note that robair is not moving and we change state
            if (frequency == frequency_expected_trasnlation_to_base)
            {
                current_state = resetting_orientation;
            }
        }
        else{
            pub_goal_to_reach.publish(local_base_position);
        }
        
    }

    void process_resetting_orientation()
    {

        if (state_has_changed)
        {
            ROS_INFO("current_state: initializing_rotation");
            ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation * 180 / M_PI);
            ROS_INFO("press enter to continue");
            // getchar();
            frequency = 0;
        }

        // Processing of the state
        // Robair rotates to its initial orientation
        // if robair is close to its initial orientation and does not move, after a while (use frequency), we switch to the state "waiting_for_a_person"
        if (new_localization)
        {
            ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation * 180 / M_PI);
        }
        
        if(fabs(current_orientation - base_orientation) <= min_angle){
            frequency = std::min(frequency_expected_orientation_at_base, frequency + 1);
            if (frequency == frequency_expected_orientation_at_base)
            {
                current_state = waiting_for_a_person;
            }
        }
        else{
            std_msgs::Float32 rot_msg = std_msgs::Float32();
            rot_msg.data = rotation_to_base;
            pub_rotation_to_do.publish(rot_msg);
        }

    }

    // CALLBACKS
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void person_positionCallback(const geometry_msgs::Point::ConstPtr &g)
    {
        // process the goal received from moving_persons detector

        new_person_position = true;
        person_position.x = g->x;
        person_position.y = g->y;
        
    }

    void robot_movingCallback(const std_msgs::Bool::ConstPtr &state)
    {

        robot_moving = state->data;

    } // robot_movingCallback

    void localizationCallback(const geometry_msgs::Point::ConstPtr &l)
    {
        // process the localization received from my localization
        if (base_position.x == 0 && base_position.y == 0)
        {
            base_position.x = l->x;
            base_position.y = l->y;
            base_orientation = l->z;
                ROS_INFO("position of robair in the map: (%f, %f, %f)", base_position.x, base_position.y, base_orientation * 180 / M_PI);
        }

        new_localization = true;
        init_localization = true;
        current_position = *l;
        current_orientation = l->z;
    }

    // Distance between two points
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb)
    {

        return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
    }
};

int main(int argc, char **argv)
{

    ROS_INFO("(decision_node) waiting for a /person_position");
    ros::init(argc, argv, "decision_node");

    decision_node bsObject;

    ros::spin();

    return 0;
}