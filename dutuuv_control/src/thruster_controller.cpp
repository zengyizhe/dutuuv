#include "dutuuv_control/thruster_controller.h"

ThrusterController::ThrusterController() : nh("~"){
    //Load parameters from .yaml files or launch files
    ThrusterController::LoadParam<string>("properties_file", properties_file);
    properties = YAML::LoadFile(properties_file);
    ThrusterController::LoadVehicleProperties();
    ThrusterController::SetThrusterCoeffs();
    weightLoad_eig.setZero();
    isSubmerged = false;

    for(int i = 0; i < 6; i++){
        weightLoad[i] = 0;
        transportThm[i] = 0;
        command[i] = 0;
        //solver_forces[i] = 0;
    }
    
    for(int i = 0; i < numThrusters; i++){
        solver_forces[i] = 0;
    }

    for(int i = 0; i < 3; i++){
        solver_cob[i] = 0;
        Fb_vector[i] = 0;
    }

    // odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &ThrusterController::OdomCB, this);
    cmd_sub = nh.subscribe<dutuuv_msgs::NetLoad>("/command/net_load", 1, &ThrusterController::NetLoadCB, this);
    thrust_pub = nh.advertise<dutuuv_msgs::ThrustStamped>("/command/thrust", 1);
    cob_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/properties/cob", 1);

    // ThrusterController::InitDynamicReconfigure();
    ThrusterController::InitThrustMsg();

    //EOM problem
    problemEOM.AddResidualBlock(new ceres::AutoDiffCostFunction<EOM, 6, 8>(new EOM(numThrusters, thrustCoeffs, weightLoad, transportThm, command)), NULL, solver_forces);
    optionsEOM.max_num_iterations = 100;
    optionsEOM.linear_solver_type = ceres::DENSE_QR;

    //Buoyancy Problem
    // problemBuoyancy.AddResidualBlock(new ceres::AutoDiffCostFunction<FindCoB, 3, 3>(new FindCoB(numThrusters, thrustCoeffs, Fb_vector, solver_forces)), NULL, solver_cob);
    // optionsBuoyancy.max_num_iterations = 100;
    // optionsBuoyancy.linear_solver_type = ceres::DENSE_QR;
}

//Load parameter from namespace
template<typename T>
void ThrusterController::LoadParam(std::string param, T &var){
    try{
        if(!nh.getParam(param, var)){
            throw 0;
        }
    }
    catch (int e){
        std::string ns = nh.getNamespace();
        ROS_ERROR("Thruster Controller Namespace: %s", ns.c_str());
        ROS_ERROR("Critical! Param \"%s/%s\" does not exist or iss not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
        ros::shutdown();
    }
}

void ThrusterController::LoadVehicleProperties(){
    mass = properties["properties"]["mass"].as<double>();

    double comX = properties["properties"]["center_of_mass"][0].as<double>();
    double comY = properties["properties"]["center_of_mass"][1].as<double>();
    double comZ = properties["properties"]["center_of_mass"][2].as<double>();
    center_of_mass[0] = comX;
    center_of_mass[1] = comY;
    center_of_mass[2] = comZ;
    //重力
    Fg = mass * GRAVITY;
    depth_fully_submerged = properties["properties"]["depth_fully_submerged"].as<double>();

    Ixx = properties["properties"]["inertia"][0].as<double>();
    Iyy = properties["properties"]["inertia"][1].as<double>();
    Izz = properties["properties"]["inertia"][2].as<double>();

    // inertia[0] = mass;
    // inertia[1] = mass;
    // inertia[2] = mass;
    // inertia[3] = Ixx;
    // inertia[4] = Iyy;
    // inertia[5] = Izz;
    //std::cout<<"the mass is "<<mass<<" !"<<std::endl;
}

// void ThrusterController::InitDynamicReconfigure(){
//     //Reset server
//     param_reconfig_server.reset(new DynamicReconfigServer(param_reconfig_mutex, nh));
//     //Set the callback
//     param_reconfig_callback = boost::bind(&ThrusterController::DynamicReconfigCallback, this, _1, _2);
//     param_reconfig_server->setCallback(param_reconfig_callback);
// }

void ThrusterController::SetThrusterCoeffs(){
    //推进器个数
    numThrusters = properties["properties"]["thrusters"].size();
    for(int i =0; i < numThrusters; i++){
        bool enabled = properties["properties"]["thrusters"][i]["enable"].as<bool>();
        thrustersEnabled.push_back((int)enabled);
    }

    //Each COLUMN contains a thruster's info
    int numThrustParams = properties["properties"]["thrusters"][0]["pose"].size();
    thrusters.resize(numThrustParams, numThrusters);
    thrustCoeffs.resize(6, numThrusters);
    thrusters.setZero();
    thrustCoeffs.setZero();

    for(int i = 0; i < numThrusters; i++){
        if(thrustersEnabled[i]){
            for(int j = 0; j < 5; j++){
                //Transform X, Y, Z to COM reference frame
                if(j < 3){
                    thrusters(j, i) = properties["properties"]["thrusters"][i]["pose"][j].as<double>() - center_of_mass[j];
                }
                else{
                    thrusters(j, i) = properties["properties"]["thrusters"][i]["pose"][j].as<double>();
                }
            }
        }
    }

    for(int i = 0; i < numThrusters; i++){
        if(thrustersEnabled[i]){
            //rotate around z, y axis
            float psi = thrusters(3, i) * PI / 180;
            float theta = thrusters(4, i) * PI / 180;
            thrustCoeffs(0, i) = cos(psi) * cos(theta);  //Effective contribution along X-axis
            thrustCoeffs(1, i) = sin(psi) * cos(theta);  //Effective contribution along Y-axis
            thrustCoeffs(2, i) = -sin(theta);            //Effective contribution along Z-axis

            //cross-product
            //Determine the effective moment arms for each thruster about the B-frame axes
            thrustCoeffs.block<3, 1>(3, i) = thrusters.block<3, 1>(0, i).cross(thrustCoeffs.block<3, 1>(0, i));
        }
    }
}

void ThrusterController::InitThrustMsg(){
    thrust_msg.header.stamp = ros::Time::now();
    thrust_msg.thrust.horizontal_port = 0;
    thrust_msg.thrust.horizontal_stbd = 0;
    thrust_msg.thrust.vertical_port = 0;
    thrust_msg.thrust.vertical_stbd = 0;


    thrust_pub.publish(thrust_msg);
}

//Callback for dynamic reconfigure
// void ThrusterController::DynamicReconfigCallback(cabin_controllers::VehiclePropertiesConfig &config, uint32_t levels){
//     CoB(0) = config.Buoyancy_X_POS;
//     CoB(1) = config.Buoyancy_Y_POS;
//     CoB(2) = config.Buoyancy_Z_POS;
//     Fb = config.Buoyant_Force;

//     isSubmerged = config.IS_Submerged;
// }

// void ThrusterController::OdomCB(const nav_msgs::Odometry::ConstPtr &odom_msg){



//     // isSubmerged = (bool)(odom_msg->pose.pose.position.z < depth_fully_submerged);
//     isSubmerged = true;

//     tf2::Quaternion quat;
//     tf2::fromMsg(odom_msg->pose.pose.orientation, quat);
//     double yaw, pitch, roll;
//     tf2::Matrix3x3 mat(quat);
//     mat.getRPY(roll, pitch, yaw);
    
//     Vector3d angular_vel;
//     angular_vel[0] = odom_msg->twist.twist.angular.x;
//     angular_vel[1] = odom_msg->twist.twist.angular.y;
//     angular_vel[2] = odom_msg->twist.twist.angular.z;

//     transportThm[3] = -angular_vel[1] * angular_vel[2] * (Izz - Iyy);
//     transportThm[4] = -angular_vel[0] * angular_vel[2] * (Ixx - Izz);
//     transportThm[5] = -angular_vel[0] * angular_vel[1] * (Iyy - Ixx);

    

//     Vector3d Fb_eig;
//     Fb_eig(0) = -Fb * sin(pitch);
//     Fb_eig(1) = Fb * sin(roll) * cos(pitch);
//     Fb_eig(2) = Fb * cos(roll) * cos(pitch);

//     weightLoad_eig(0) = (Fg - Fb) * sin(pitch);
//     weightLoad_eig(1) = -(Fg - Fb) * sin(roll) * cos(pitch);
//     weightLoad_eig(2) = -(Fg - Fb) * cos(roll) * cos(pitch);
//     weightLoad_eig.segment<3>(3) = CoB.cross(Fb_eig);
//     weightLoad_eig = weightLoad_eig * ((int)(isSubmerged));

//     // Convert Eigen::VectorXd to c++ double[X]
//     Map<RowMatrixXd>(&weightLoad[0], weightLoad_eig.rows(), weightLoad_eig.cols()) = weightLoad_eig;
//     Map<Vector3d>(&Fb_vector[0], Fb_eig.rows(), Fb_eig.cols()) = Fb_eig;
// }

void ThrusterController::NetLoadCB(const dutuuv_msgs::NetLoad::ConstPtr &netload_msg){
    
    command[0] = netload_msg->force.x;
    command[1] = netload_msg->force.y;
    command[2] = netload_msg->force.z;       
    command[3] = netload_msg->moment.x;
    command[4] = netload_msg->moment.y;
    command[5] = netload_msg->moment.z;

    //These initial guesses don't make much of a difference
    for(int i = 0; i < numThrusters; i++){
        solver_forces[i] = 0.0;
    }

    //solve all my problems
    ceres::Solve(optionsEOM, &problemEOM, &summaryEOM);

    thrust_msg.header.stamp = ros::Time::now();
    thrust_msg.thrust.horizontal_port = solver_forces[thrust_msg.thrust.HP];
    thrust_msg.thrust.horizontal_stbd = solver_forces[thrust_msg.thrust.HS];
    thrust_msg.thrust.vertical_port = solver_forces[thrust_msg.thrust.VP];
    thrust_msg.thrust.vertical_stbd = solver_forces[thrust_msg.thrust.VS];


    thrust_pub.publish(thrust_msg);
}

void ThrusterController::Loop(){
    ros::Rate rate(50);
    while(!ros::isShuttingDown()){
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "thruster_controller");
    ThrusterController tc;
    tc.Loop();
}
