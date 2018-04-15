#include "ros/ros.h"
#include <cstdlib>
#include <memory>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>

#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_msgs/TFMessage.h>

namespace eurobot{
namespace params{

    std::string colour;                         // to be set at the start of the node
    tf2::Transform T_BC;
    tf2::Transform T_CB;

    tf2::Transform lookupTagTransform(int id){
        // Gives T_WT, the transformation matrix of a detected tag relative to the world
    
        tf2::Vector3 r;
        tf2::Quaternion q;
        float ex, ey, ez; //Euler angles (radians) for easier calculation
    
        if(!std::strcmp(eurobot::params::colour.c_str(), "orange")){
        // Note: z-axis of tag is coming out of the page, y-axis points to the top
            switch(id){
                case  0: r.setValue(-0.044, 0.040, 0.310); ex = 1.57; ey = 0.00; ez =  1.57; break;
                case  1: r.setValue(-0.044, 0.040, 0.230); ex = 1.57; ey = 0.00; ez =  1.57; break;
                case  2: r.setValue(-0.044, 0.040, 0.150); ex = 1.57; ey = 0.00; ez =  1.57; break;
                case  3: r.setValue(-0.084, 0.080, 0.310); ex = 1.57; ey = 0.00; ez =  3.14; break;
                case  4: r.setValue(-0.084, 0.080, 0.230); ex = 1.57; ey = 0.00; ez =  3.14; break;
                case  5: r.setValue(-0.084, 0.080, 0.150); ex = 1.57; ey = 0.00; ez =  3.14; break;
                case  6: r.setValue( 3.084, 1.030, 0.310); ex = 1.57; ey = 0.00; ez =  3.14; break;
                case  7: r.setValue( 3.084, 1.030, 0.230); ex = 1.57; ey = 0.00; ez =  3.14; break;
                case  8: r.setValue( 3.084, 1.030, 0.150); ex = 1.57; ey = 0.00; ez =  3.14; break;
                case  9: r.setValue( 3.044, 0.990, 0.310); ex = 1.57; ey = 0.00; ez = -1.57; break;
                case 10: r.setValue( 3.044, 0.990, 0.230); ex = 1.57; ey = 0.00; ez = -1.57; break;
                case 11: r.setValue( 3.044, 0.990, 0.150); ex = 1.57; ey = 0.00; ez = -1.57; break;
                case 12: r.setValue( 3.084, 0.950, 0.310); ex = 1.57; ey = 0.00; ez =  0.00; break;
                case 13: r.setValue( 3.084, 0.950, 0.230); ex = 1.57; ey = 0.00; ez =  0.00; break;
                case 14: r.setValue( 3.084, 0.950, 0.150); ex = 1.57; ey = 0.00; ez =  0.00; break;
                case 15: r.setValue(-0.084, 1.900, 0.310); ex = 1.57; ey = 0.00; ez =  0.00; break;
                case 16: r.setValue(-0.084, 1.900, 0.230); ex = 1.57; ey = 0.00; ez =  0.00; break;
                case 17: r.setValue(-0.084, 1.900, 0.150); ex = 1.57; ey = 0.00; ez =  0.00; break;
                case 18: r.setValue(-0.044, 1.940, 0.310); ex = 1.57; ey = 0.00; ez =  1.57; break;
                case 19: r.setValue(-0.044, 1.940, 0.230); ex = 1.57; ey = 0.00; ez =  1.57; break;
                case 20: r.setValue(-0.044, 1.940, 0.150); ex = 1.57; ey = 0.00; ez =  1.57; break;
                default: r.setValue( 0.000, 0.000, 0.000); ex = 0.00; ey = 0.00; ez =  0.00; break;
            }
        } else {
            switch(id){
                case  0: r.setValue( 3.084, 0.080, 0.310); ex = 1.57; ey = 0.00; ez =  3.14; break; 
                case  1: r.setValue( 3.084, 0.080, 0.230); ex = 1.57; ey = 0.00; ez =  3.14; break; 
                case  2: r.setValue( 3.084, 0.080, 0.150); ex = 1.57; ey = 0.00; ez =  3.14; break;
                case  3: r.setValue( 3.044, 0.040, 0.310); ex = 1.57; ey = 0.00; ez = -1.57; break; 
                case  4: r.setValue( 3.044, 0.040, 0.230); ex = 1.57; ey = 0.00; ez = -1.57; break; 
                case  5: r.setValue( 3.044, 0.040, 0.150); ex = 1.57; ey = 0.00; ez = -1.57; break; 
                case  6: r.setValue(-0.084, 0.950, 0.310); ex = 1.57; ey = 0.00; ez =  0.00; break; 
                case  7: r.setValue(-0.084, 0.950, 0.230); ex = 1.57; ey = 0.00; ez =  0.00; break;
                case  8: r.setValue(-0.084, 0.950, 0.150); ex = 1.57; ey = 0.00; ez =  0.00; break; 
                case  9: r.setValue(-0.044, 0.990, 0.310); ex = 1.57; ey = 0.00; ez =  1.57; break; 
                case 10: r.setValue(-0.044, 0.990, 0.230); ex = 1.57; ey = 0.00; ez =  1.57; break; 
                case 11: r.setValue(-0.044, 0.990, 0.150); ex = 1.57; ey = 0.00; ez =  1.57; break; 
                case 12: r.setValue(-0.084, 1.030, 0.310); ex = 1.57; ey = 0.00; ez =  3.14; break;
                case 13: r.setValue(-0.084, 1.030, 0.230); ex = 1.57; ey = 0.00; ez =  3.14; break; 
                case 14: r.setValue(-0.084, 1.030, 0.150); ex = 1.57; ey = 0.00; ez =  3.14; break; 
                case 15: r.setValue( 3.044, 1.940, 0.310); ex = 1.57; ey = 0.00; ez = -1.57; break; 
                case 16: r.setValue( 3.044, 1.940, 0.230); ex = 1.57; ey = 0.00; ez = -1.57; break; 
                case 17: r.setValue( 3.044, 1.940, 0.150); ex = 1.57; ey = 0.00; ez = -1.57; break;
                case 18: r.setValue( 3.084, 1.900, 0.310); ex = 1.57; ey = 0.00; ez =  0.00; break; 
                case 19: r.setValue( 3.084, 1.900, 0.230); ex = 1.57; ey = 0.00; ez =  0.00; break; 
                case 20: r.setValue( 3.084, 1.900, 0.150); ex = 1.57; ey = 0.00; ez =  0.00; break; 
                default: r.setValue( 0.000, 0.000, 0.000); ex = 0.00; ey = 0.00; ez =  0.00; break; 
            }
        }
        
        q.setRPY(ex,ey,ez);
    
        return tf2::Transform(q,r);
    }

    tf2::Transform lookupEnemyTransform(int id){
        // Gives T_TE, enemy base frame relative to detected tag
        tf2::Vector3 r;
        tf2::Quaternion q;
        float ex, ey, ez; //Euler angles (radians) for easier calculation

        switch(id){
            // Values for T_ET, detected tag relative to enemy base
            case 21: r.setValue(0.039, 0.000, 0.469);  ex = 1.57; ey = 0.00; ez = 1.57; break;
            case 22: r.setValue(0.000, 0.039, 0.469);  ex = 1.57; ey = 0.00; ez =-1.57; break; 
            case 23: r.setValue(-0.039, 0.000, 0.469); ex = 1.57; ey = 0.00; ez = 1.57; break; 
            case 24: r.setValue(0.000, -0.039, 0.469); ex = 1.57; ey = 0.00; ez = 0.00; break; 
            default: r.setValue(0.00, 0.00, 0.00); ex = 0.00; ey = 0.00; ez = 0.00; break; 
        }
        q.setRPY(ex,ey,ez);
    
        return tf2::Transform(q,r).inverse(); // invert T_ET to give T_TE
    }

} //params
} //eurobot


class Subscriber{
public:
    Subscriber(std::string name) : cam_name(name) {}

    void setSelfPub(std::shared_ptr<ros::Publisher> ptr){
        self_pub = ptr;
    }
    void setEnemyPub(std::shared_ptr<ros::Publisher> ptr){
        enemy_pub = ptr;
    }

    void callback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr &msg){
        static tf::TransformBroadcaster br;        
        tf2_msgs::TFMessage self_pos;
        tf2_msgs::TFMessage enemy_pos;
    
        for(auto tag : msg->detections){
            // variables to be used for final result
            geometry_msgs::TransformStamped temp;
            tf2::Transform T_res;
  
            // extract T_TC from Apriltag reading
            auto r = tag.pose.pose.pose.position;
            auto q = tag.pose.pose.pose.orientation;
            int id = tag.id[0];

            // because screw datatypes
            tf2::Vector3 r_CT(r.x, r.y, r.z);
            tf2::Quaternion q_CT;
            q_CT.setValue(q.x, q.y, q.z, q.w);
            
            tf2::Transform T_CT;
            T_CT.setOrigin(r_CT);
            T_CT.setRotation(q_CT);
            
            if(id <= 20 && id >= 0){    
                // if beacon tag detected
                // Calculate T_WB = T_WT * T_TC * T_CB
                tf2::Transform T_TC = T_CT.inverse();
                tf2::Transform T_WT = eurobot::params::lookupTagTransform(id);
                T_res = T_WT * T_TC * eurobot::params::T_CB;
            
                // set the proper links for the result
                temp.header.frame_id = "world";
                temp.child_frame_id = "base_tag" + std::to_string(id) + "(" + cam_name + ")";
            } else if(id > 20 && id < 25) {
                // if enemy tag detected
                tf2::Transform T_TE = eurobot::params::lookupEnemyTransform(id);

                // calculate T_BE, enemy base relative to our robot
                // T_BE = T_BC * T_CT * T_TE
                T_res = eurobot::params::T_BC * T_CT * T_TE;

                // set proper links
                temp.header.frame_id = "base"; 
                temp.child_frame_id = "enemy_"+ std::to_string(id);
            }

            tf2::Vector3 r_res = T_res.getOrigin();
            tf2::Quaternion q_res = T_res.getRotation();
            
            // Form the transform message 
            temp.header.stamp = msg->header.stamp;
    
            temp.transform.translation.x = r_res.x();
            temp.transform.translation.y = r_res.y();
            temp.transform.translation.z = r_res.z();
            
            temp.transform.rotation.x = q_res.x();
            temp.transform.rotation.y = q_res.y();
            temp.transform.rotation.z = q_res.z();
            temp.transform.rotation.w = q_res.w();


            if(id <= 20 && id >= 0){
                self_pos.transforms.push_back(temp);
                br.sendTransform(temp);
            } else if(id > 20 && id < 25){
                enemy_pos.transforms.push_back(temp);
            }


        }
        self_pub->publish(self_pos);
        enemy_pub->publish(enemy_pos);
    }

private:
    std::string cam_name;
    std::shared_ptr<ros::Publisher> self_pub;
    std::shared_ptr<ros::Publisher> enemy_pub;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "localisation_node");
    ros::NodeHandle nh;

    if(!nh.getParam("localisation_node/colour", eurobot::params::colour)){
        eurobot::params::colour = "green";
    }

    std::string cam_name;
    double rx, ry, rz, ex, ey, ez;
    nh.getParam("localisation_node/name" , cam_name);
    nh.getParam("localisation_node/rx"   , rx);
    nh.getParam("localisation_node/ry"   , ry);
    nh.getParam("localisation_node/rz"   , rz);
    nh.getParam("localisation_node/roll" , ex);
    nh.getParam("localisation_node/pitch", ey);
    nh.getParam("localisation_node/yaw"  , ez);
    
    tf2::Vector3 r(rx, ry, rz);
    tf2::Quaternion q;
    q.setRPY(ex, ey, ez);
    eurobot::params::T_BC = tf2::Transform(q, r);
    eurobot::params::T_CB = eurobot::params::T_BC.inverse();



    Subscriber subscriber(cam_name);
    ros::Subscriber sub = nh.subscribe("tag_detections", 10, &Subscriber::callback, &subscriber);
    ros::Publisher self_pub = nh.advertise<tf2_msgs::TFMessage>("/eurobot/state/visual_measurements", 10);
    ros::Publisher enemy_pub = nh.advertise<tf2_msgs::TFMessage>("/eurobot/enemy/relative_position", 10);
    auto self_sp = std::make_shared<ros::Publisher>(self_pub);
    auto enemy_sp = std::make_shared<ros::Publisher>(enemy_pub);
    
    subscriber.setSelfPub(self_sp);
    subscriber.setEnemyPub(enemy_sp);

    ros::spin();
    return 0;
}
