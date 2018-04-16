#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cstdlib>
#include <cstdio>
#include <vector>
#include <string>
#include <map>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>

static std::string colour;

static std::map<std::string, int> init_map(){
    // mapping of tag frame to an id number
    
    std::map<std::string, int> m;
    m["tag00"] =  0;
    m["tag01"] =  1;
    m["tag02"] =  2;
    m["tag03"] =  3;
    m["tag04"] =  4;
    m["tag05"] =  5;
    m["tag06"] =  6;
    m["tag07"] =  7;
    m["tag08"] =  8;
    m["tag09"] =  9;
    m["tag10"] = 10;
    m["tag11"] = 11;
    m["tag12"] = 12;
    m["tag13"] = 13;
    m["tag14"] = 14;
    m["tag15"] = 15;
    m["tag16"] = 16;
    m["tag17"] = 17;
    m["tag18"] = 18;
    m["tag19"] = 19;
    m["tag20"] = 20;
    m["tag21"] = 21;
    m["tag22"] = 22;
    m["tag23"] = 23;
    m["tag24"] = 24;
    m["tag25"] = 25;
    m["tag26"] = 26;
    m["tag27"] = 27;
    m["tag28"] = 28;

    return m;
}

tf2::Transform lookupTagTransform(int id){
    // Gives T_WT, the transformation matrix of a detected tag relative to the world

    tf2::Vector3 r;
    tf2::Quaternion q;
    float ex, ey, ez; //Euler angles (radians) for easier calculation

    if(!std::strcmp(colour.c_str(), "orange")){
    // Note: z-axis of tag is coming out of the page, y-axis points to the top
        switch(id){
            case  0: r.setValue(-0.0440, 0.0475, 0.3675); ex = 1.571; ey = 0.000; ez = 1.571; break; 
            case  1: r.setValue(-0.0440, 0.0475, 0.4625); ex = 1.571; ey = 0.000; ez = 1.571; break; 
            case  2: r.setValue(-0.0440, 0.0475, 0.2725); ex = 1.571; ey = 0.000; ez = 1.571; break;
            case  3: r.setValue(-0.0440, 0.0475, 0.1775); ex = 1.571; ey = 0.000; ez = 1.571; break; 
            case  4: r.setValue(-0.0915, 0.0950, 0.4625); ex = 1.571; ey = 0.000; ez = 3.142; break; 
            case  5: r.setValue(-0.0915, 0.0950, 0.3675); ex = 1.571; ey = 0.000; ez = 3.142; break; 
            case  6: r.setValue(-0.0915, 0.0950, 0.2725); ex = 1.571; ey = 0.000; ez = 3.142; break; 
            case  7: r.setValue(-0.0915, 0.0950, 0.1775); ex = 1.571; ey = 0.000; ez = 3.142; break;
            case  8: r.setValue( 3.0915, 1.0450, 0.4625); ex = 1.571; ey = 0.000; ez = 3.142; break; 
            case  9: r.setValue( 3.0915, 1.0450, 0.3675); ex = 1.571; ey = 0.000; ez = 3.142; break; 
            case 10: r.setValue( 3.0915, 1.0450, 0.2725); ex = 1.571; ey = 0.000; ez = 3.142; break; 
            case 11: r.setValue( 3.0915, 1.0450, 0.1775); ex = 1.571; ey = 0.000; ez = 3.142; break; 
            case 12: r.setValue( 3.0440, 0.9975, 0.4625); ex = 1.571; ey = 0.000; ez =-1.571; break;
            case 13: r.setValue( 3.0440, 0.9975, 0.3675); ex = 1.571; ey = 0.000; ez =-1.571; break; 
            case 14: r.setValue( 3.0440, 0.9975, 0.2725); ex = 1.571; ey = 0.000; ez =-1.571; break; 
            case 15: r.setValue( 3.0440, 0.9975, 0.1775); ex = 1.571; ey = 0.000; ez =-1.571; break; 
            case 16: r.setValue( 3.0915, 0.9500, 0.4625); ex = 1.571; ey = 0.000; ez = 0.000; break; 
            case 17: r.setValue( 3.0915, 0.9500, 0.3675); ex = 1.571; ey = 0.000; ez = 0.000; break;
            case 18: r.setValue( 3.0915, 0.9500, 0.2725); ex = 1.571; ey = 0.000; ez = 0.000; break; 
            case 19: r.setValue( 3.0915, 0.9500, 0.1775); ex = 1.571; ey = 0.000; ez = 0.000; break; 
            case 20: r.setValue(-0.0915, 1.9000, 0.4625); ex = 1.571; ey = 0.000; ez = 0.000; break; 
            case 21: r.setValue(-0.0915, 1.9000, 0.3675); ex = 1.571; ey = 0.000; ez = 0.000; break; 
            case 22: r.setValue(-0.0915, 1.9000, 0.2725); ex = 1.571; ey = 0.000; ez = 0.000; break;
            case 23: r.setValue(-0.0915, 1.9000, 0.1775); ex = 1.571; ey = 0.000; ez = 0.000; break; 
            case 24: r.setValue(-0.0440, 1.9475, 0.4625); ex = 1.571; ey = 0.000; ez = 1.571; break; 
            case 25: r.setValue(-0.0440, 1.9475, 0.3675); ex = 1.571; ey = 0.000; ez = 1.571; break; 
            case 26: r.setValue(-0.0440, 1.9475, 0.2725); ex = 1.571; ey = 0.000; ez = 1.571; break; 
            case 27: r.setValue(-0.0440, 1.9475, 0.1775); ex = 1.571; ey = 0.000; ez = 1.571; break;
            default: r.setValue(0.000, 0.000, 0.000); ex = 0.000; ey = 0.000; ez = 0.000; break; 
        }
    } else {
        switch(id){
            case  0:  r.setValue( 3.0915, 0.0950, 0.3675); ex = 1.571; ey = 0.000; ez = 3.142; break; 
            case  1:  r.setValue( 3.0915, 0.0950, 0.4625); ex = 1.571; ey = 0.000; ez = 3.142; break; 
            case  2:  r.setValue( 3.0915, 0.0950, 0.2725); ex = 1.571; ey = 0.000; ez = 3.142; break;
            case  3:  r.setValue( 3.0915, 0.0950, 0.1775); ex = 1.571; ey = 0.000; ez = 3.142; break; 
            case  4:  r.setValue( 3.0440, 0.0475, 0.4625); ex = 1.571; ey = 0.000; ez =-1.571; break; 
            case  5:  r.setValue( 3.0440, 0.0475, 0.3675); ex = 1.571; ey = 0.000; ez =-1.571; break; 
            case  6:  r.setValue( 3.0440, 0.0475, 0.2725); ex = 1.571; ey = 0.000; ez =-1.571; break; 
            case  7:  r.setValue( 3.0440, 0.0475, 0.1775); ex = 1.571; ey = 0.000; ez =-1.571; break;
            case  8:  r.setValue(-0.0915, 0.9500, 0.4625); ex = 1.571; ey = 0.000; ez = 0.000; break; 
            case  9:  r.setValue(-0.0915, 0.9500, 0.3675); ex = 1.571; ey = 0.000; ez = 0.000; break; 
            case 10:  r.setValue(-0.0915, 0.9500, 0.2725); ex = 1.571; ey = 0.000; ez = 0.000; break; 
            case 11:  r.setValue(-0.0915, 0.9500, 0.1775); ex = 1.571; ey = 0.000; ez = 0.000; break; 
            case 12:  r.setValue(-0.0440, 0.9975, 0.4625); ex = 1.571; ey = 0.000; ez = 1.571; break;
            case 13:  r.setValue(-0.0440, 0.9975, 0.3675); ex = 1.571; ey = 0.000; ez = 1.571; break; 
            case 14:  r.setValue(-0.0440, 0.9975, 0.2725); ex = 1.571; ey = 0.000; ez = 1.571; break; 
            case 15:  r.setValue(-0.0440, 0.9975, 0.1775); ex = 1.571; ey = 0.000; ez = 1.571; break; 
            case 16:  r.setValue(-0.0915, 1.0450, 0.4625); ex = 1.571; ey = 0.000; ez = 3.142; break; 
            case 17:  r.setValue(-0.0915, 1.0450, 0.3675); ex = 1.571; ey = 0.000; ez = 3.142; break;
            case 18:  r.setValue(-0.0915, 1.0450, 0.2725); ex = 1.571; ey = 0.000; ez = 3.142; break; 
            case 19:  r.setValue(-0.0915, 1.0450, 0.1775); ex = 1.571; ey = 0.000; ez = 3.142; break; 
            case 20:  r.setValue( 3.0440, 1.9475, 0.4625); ex = 1.571; ey = 0.000; ez =-1.571; break; 
            case 21:  r.setValue( 3.0440, 1.9475, 0.3675); ex = 1.571; ey = 0.000; ez =-1.571; break; 
            case 22:  r.setValue( 3.0440, 1.9475, 0.2725); ex = 1.571; ey = 0.000; ez =-1.571; break;
            case 23:  r.setValue( 3.0440, 1.9475, 0.1775); ex = 1.571; ey = 0.000; ez =-1.571; break; 
            case 24:  r.setValue( 3.0915, 1.9000, 0.4625); ex = 1.571; ey = 0.000; ez = 0.000; break; 
            case 25:  r.setValue( 3.0915, 1.9000, 0.3675); ex = 1.571; ey = 0.000; ez = 0.000; break; 
            case 26:  r.setValue( 3.0915, 1.9000, 0.2725); ex = 1.571; ey = 0.000; ez = 0.000; break; 
            case 27:  r.setValue( 3.0915, 1.9000, 0.1775); ex = 1.571; ey = 0.000; ez = 0.000; break;
            default: r.setValue(0.000, 0.000, 0.000); ex = 0.000; ey = 0.000; ez = 0.000; break; 
        }
    }
    q.setRPY(ex,ey,ez);

    return tf2::Transform(q,r);
}

void meanCallback(const geometry_msgs::TransformStamped::ConstPtr &data){
    static tf2_ros::TransformBroadcaster br;      
    br.sendTransform(*data);
}


int main(int argc, char **argv){

    ros::init(argc, argv, "eurobot_tf_broadcaster");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/eurobot/state/kalman_mean", 10, meanCallback);

    static tf2_ros::StaticTransformBroadcaster static_bc;
    auto name_map = init_map();

    if(!nh.getParam("/eurobot_arena_tf_generator/colour", colour)){
        colour= "green";
    }

    for(auto& kv : name_map){
        int id = kv.second;
        tf2::Transform T = lookupTagTransform(id);
        tf2::Vector3 r = T.getOrigin();
        tf2::Quaternion q = T.getRotation();
        
        // Form the transform message 
        geometry_msgs::TransformStamped temp;
        temp.header.stamp = ros::Time::now();
        temp.header.frame_id = "world";
        temp.child_frame_id = kv.first;
    
        temp.transform.translation.x = r.x();
        temp.transform.translation.y = r.y();
        temp.transform.translation.z = r.z();
        
        temp.transform.rotation.x = q.x();
        temp.transform.rotation.y = q.y();
        temp.transform.rotation.z = q.z();
        temp.transform.rotation.w = q.w();
        static_bc.sendTransform(temp);
    }

    ros::spin();
    return 0;

}
