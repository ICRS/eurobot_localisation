#include <ros/ros.h>
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
            case  0:  r.setValue(-0.044, 0.040, 0.310); ex = 1.57; ey = 0.00; ez = 1.57; break; 
            case  1:  r.setValue(-0.044, 0.040, 0.230); ex = 1.57; ey = 0.00; ez = 1.57; break; 
            case  2:  r.setValue(-0.044, 0.040, 0.150); ex = 1.57; ey = 0.00; ez = 1.57; break;
            case  3:  r.setValue(-0.084, 0.080, 0.310); ex = 1.57; ey = 0.00; ez = 3.14; break; 
            case  4:  r.setValue(-0.084, 0.080, 0.230); ex = 1.57; ey = 0.00; ez = 3.14; break; 
            case  5:  r.setValue(-0.084, 0.080, 0.150); ex = 1.57; ey = 0.00; ez = 3.14; break; 
            case  6:  r.setValue(3.084, 1.030, 0.310); ex = 1.57; ey = 0.00; ez = 3.14; break; 
            case  7:  r.setValue(3.084, 1.030, 0.230); ex = 1.57; ey = 0.00; ez = 3.14; break;
            case  8:  r.setValue(3.084, 1.030, 0.150); ex = 1.57; ey = 0.00; ez = 3.14; break; 
            case  9:  r.setValue(3.044, 0.990, 0.310); ex = 1.57; ey = 0.00; ez = -1.57; break; 
            case 10:  r.setValue(3.044, 0.990, 0.230); ex = 1.57; ey = 0.00; ez = -1.57; break; 
            case 11:  r.setValue(3.044, 0.990, 0.150); ex = 1.57; ey = 0.00; ez = -1.57; break; 
            case 12:  r.setValue(3.084, 0.950, 0.310); ex = 1.57; ey = 0.00; ez = 0.00; break;
            case 13:  r.setValue(3.084, 0.950, 0.230); ex = 1.57; ey = 0.00; ez = 0.00; break; 
            case 14:  r.setValue(3.084, 0.950, 0.150); ex = 1.57; ey = 0.00; ez = 0.00; break; 
            case 15:  r.setValue(-0.084, 1.900, 0.310); ex = 1.57; ey = 0.00; ez = 0.00; break; 
            case 16:  r.setValue(-0.084, 1.900, 0.230); ex = 1.57; ey = 0.00; ez = 0.00; break; 
            case 17:  r.setValue(-0.084, 1.900, 0.150); ex = 1.57; ey = 0.00; ez = 0.00; break;
            case 18:  r.setValue(-0.044, 1.940, 0.310); ex = 1.57; ey = 0.00; ez = 1.57; break; 
            case 19:  r.setValue(-0.044, 1.940, 0.230); ex = 1.57; ey = 0.00; ez = 1.57; break; 
            case 20:  r.setValue(-0.044, 1.940, 0.150); ex = 1.57; ey = 0.00; ez = 1.57; break; 
            default: r.setValue(0.00, 0.00, 0.00); ex = 0.00; ey = 0.00; ez = 0.00; break; 
        }
    } else {
        switch(id){
            case  0:  r.setValue(3.084, 0.080, 0.310); ex = 1.57; ey = 0.00; ez = 3.14; break; 
            case  1:  r.setValue(3.084, 0.080, 0.230); ex = 1.57; ey = 0.00; ez = 3.14; break; 
            case  2:  r.setValue(3.084, 0.080, 0.150); ex = 1.57; ey = 0.00; ez = 3.14; break;
            case  3:  r.setValue(3.044, 0.040, 0.310); ex = 1.57; ey = 0.00; ez = -1.57; break; 
            case  4:  r.setValue(3.044, 0.040, 0.230); ex = 1.57; ey = 0.00; ez = -1.57; break; 
            case  5:  r.setValue(3.044, 0.040, 0.150); ex = 1.57; ey = 0.00; ez = -1.57; break; 
            case  6:  r.setValue(-0.084, 0.950, 0.310); ex = 1.57; ey = 0.00; ez = 0.00; break; 
            case  7:  r.setValue(-0.084, 0.950, 0.230); ex = 1.57; ey = 0.00; ez = 0.00; break;
            case  8:  r.setValue(-0.084, 0.950, 0.150); ex = 1.57; ey = 0.00; ez = 0.00; break; 
            case  9:  r.setValue(-0.044, 0.990, 0.310); ex = 1.57; ey = 0.00; ez = 1.57; break; 
            case 10:  r.setValue(-0.044, 0.990, 0.230); ex = 1.57; ey = 0.00; ez = 1.57; break; 
            case 11:  r.setValue(-0.044, 0.990, 0.150); ex = 1.57; ey = 0.00; ez = 1.57; break; 
            case 12:  r.setValue(-0.084, 1.030, 0.310); ex = 1.57; ey = 0.00; ez = 3.14; break;
            case 13:  r.setValue(-0.084, 1.030, 0.230); ex = 1.57; ey = 0.00; ez = 3.14; break; 
            case 14:  r.setValue(-0.084, 1.030, 0.150); ex = 1.57; ey = 0.00; ez = 3.14; break; 
            case 15:  r.setValue(3.044, 1.940, 0.310); ex = 1.57; ey = 0.00; ez = -1.57; break; 
            case 16:  r.setValue(3.044, 1.940, 0.230); ex = 1.57; ey = 0.00; ez = -1.57; break; 
            case 17:  r.setValue(3.044, 1.940, 0.150); ex = 1.57; ey = 0.00; ez = -1.57; break;
            case 18:  r.setValue(3.084, 1.900, 0.310); ex = 1.57; ey = 0.00; ez = 0.00; break; 
            case 19:  r.setValue(3.084, 1.900, 0.230); ex = 1.57; ey = 0.00; ez = 0.00; break; 
            case 20:  r.setValue(3.084, 1.900, 0.150); ex = 1.57; ey = 0.00; ez = 0.00; break; 
            default: r.setValue(0.00, 0.00, 0.00); ex = 0.00; ey = 0.00; ez = 0.00; break; 
        }
    }
    q.setRPY(ex,ey,ez);

    return tf2::Transform(q,r);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "eurobot_tf_broadcaster");
    ros::NodeHandle nh;

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
