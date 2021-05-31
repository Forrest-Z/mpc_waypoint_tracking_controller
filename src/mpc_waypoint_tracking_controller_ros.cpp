/*
 * mpc_ros
 * Copyright (c) 2021, Geonhee Lee
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 * Authors : Geonhee Lee, Balamurugan Kandan
 */

#include "mpc_waypoint_tracking_controller/mpc_waypoint_tracking_controller_ros.h"
#include <pluginlib/class_list_macros.h>
#include <math.h>

using namespace std;
using namespace Eigen;

PLUGINLIB_EXPORT_CLASS(mpc_waypoint_tracking_controller::MPCWayPointTrackerROS, nav_core::BaseLocalPlanner)

namespace mpc_waypoint_tracking_controller {

    MPCWayPointTrackerROS::MPCWayPointTrackerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}
	MPCWayPointTrackerROS::MPCWayPointTrackerROS(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false) {
        // initialize planner
        initialize(name, tf, costmap_ros);
    }

	MPCWayPointTrackerROS::~MPCWayPointTrackerROS() {}

	void MPCWayPointTrackerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {

        ros::NodeHandle private_nh("~/" + name);
        g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
        l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

		tf_ = tf;
		costmap_ros_ = costmap_ros;
        //initialize the copy of the costmap the controller will use
        costmap_ = costmap_ros_->getCostmap();
        global_frame_ = costmap_ros_->getGlobalFrameID();
        robot_base_frame_ = costmap_ros_->getBaseFrameID();
        footprint_spec_ = costmap_ros_->getRobotFootprint();
        
        planner_util_.initialize(tf, costmap_, costmap_ros_->getGlobalFrameID());
        
        if( private_nh.getParam( "odom_frame", _odom_frame )) {
            odom_helper_.setOdomTopic( _odom_frame );
        }

        //Assuming this planner is being run within the navigation stack, we can
        //just do an upward search for the frequency at which its being run. This
        //also allows the frequency to be overwritten locally.
        ros::NodeHandle nh_;
        std::string controller_frequency_param_name;
        double controller_frequency = 0;
        if(!nh_.searchParam("move_base/controller_frequency", controller_frequency_param_name)) {
            ROS_WARN("controller_frequency_param_name doesn't exits");
        } else {
            nh_.param(controller_frequency_param_name, controller_frequency, 20.0);
            
            if(controller_frequency > 0) {
            } else {
                ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
            }
        }
        //private_nh.param("vehicle_Lf", _Lf, 0.290); // distance between the front of the vehicle and its center of gravity
        _dt = double(1.0/controller_frequency); // time step duration dt in s 

        //Parameter for topics & Frame name
        private_nh.param<std::string>("map_frame", _map_frame, "map" ); 
        private_nh.param<std::string>("odom_frame", _odom_frame, "odom");
        private_nh.param<std::string>("base_frame", _base_frame, "base_footprint");

        //Publishers and Subscribers
        _sub_odom   = _nh.subscribe("odom", 1, &MPCWayPointTrackerROS::odomCB, this);
        _pub_mpctraj   = _nh.advertise<nav_msgs::Path>("mpc_trajectory", 1);// MPC trajectory output
        _pub_odompath  = _nh.advertise<nav_msgs::Path>("mpc_reference", 1); // reference path for MPC ///mpc_reference 

        //Init variables
        _throttle = 0.0; 
        _w = 0.0;
        _speed = 0.0;

        //_ackermann_msg = ackermann_msgs::AckermannDriveStamped();
        _twist_msg = geometry_msgs::Twist();
        _mpc_traj = nav_msgs::Path();

        _debug_info = false;
        
        dsrv_ = new dynamic_reconfigure::Server<MPCWayPointTrackerROSConfig>(private_nh);
        dynamic_reconfigure::Server<MPCWayPointTrackerROSConfig>::CallbackType cb = boost::bind(&MPCWayPointTrackerROS::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        initialized_ = true;
    }

  void MPCWayPointTrackerROS::reconfigureCB(MPCWayPointTrackerROSConfig &config, uint32_t level) {
    // update generic local planner params
    base_local_planner::LocalPlannerLimits limits;
    limits.max_vel_trans = config.max_vel_trans;
    limits.min_vel_trans = config.min_vel_trans;
    limits.max_vel_x = config.max_vel_x;
    limits.min_vel_x = config.min_vel_x;
    limits.max_vel_y = config.max_vel_y;
    limits.min_vel_y = config.min_vel_y;
    limits.max_vel_theta = config.max_vel_theta;
    limits.min_vel_theta = config.min_vel_theta;
    limits.acc_lim_x = config.acc_lim_x;
    limits.acc_lim_y = config.acc_lim_y;
    limits.acc_lim_theta = config.acc_lim_theta;
    limits.acc_lim_trans = config.acc_lim_trans;
    limits.xy_goal_tolerance = config.xy_goal_tolerance;
    limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
    limits.prune_plan = config.prune_plan;
    limits.trans_stopped_vel = config.trans_stopped_vel;
    limits.theta_stopped_vel = config.theta_stopped_vel;

    //Parameter for MPC solver
    _debug_info = config.debug_info;
    _delay_mode = config.delay_mode;
    _max_speed = config.max_speed;
    _waypointsDist = config.waypoints_dist;
    _pathLength = config.path_length;
    _mpc_steps = config.mpc_steps;
    _ref_cte = config.mpc_ref_cte;
    _ref_vel = config.mpc_ref_vel;
    _ref_etheta = config.mpc_ref_etheta;
    _w_cte = config.mpc_w_cte;
    _w_etheta = config.mpc_w_etheta;
    _w_vel = config.mpc_w_vel;
    _w_angvel = config.mpc_w_angvel;
    _w_angvel_d = config.mpc_w_angvel_d;
    _w_accel_d = config.mpc_w_accel_d;
    _w_accel = config.mpc_w_accel;
    _max_angvel = config.mpc_max_angvel;
    _max_throttle = config.mpc_max_throttle;
    _bound_value = config.mpc_bound_value;

    if(_debug_info) {
        //Display the parameters
        cout << "\n===== Parameters =====" << endl;
        cout << "debug_info: "  << _debug_info << endl;
        cout << "delay_mode: "  << _delay_mode << endl;
        cout << "max_speed: "  << _max_speed << endl;
        cout << "waypointsDist: "  << _waypointsDist << endl;
        cout << "pathLength: "  << _pathLength << endl;
        cout << "mpc_steps: "   << _mpc_steps << endl;
        cout << "ref_cte: "  << _ref_cte << endl;
        cout << "ref_vel: "  << _ref_vel << endl;
        cout << "ref_etheta: "  << _ref_etheta << endl;
        cout << "w_cte: "  << _w_cte << endl;
        cout << "w_etheta: "  << _w_etheta << endl;
        cout << "w_vel: "  << _w_vel << endl;
        cout << "w_angvel: "  << _w_angvel << endl;
        cout << "w_angvel_d: "  << _w_angvel_d << endl;
        cout << "w_accel_d: "  << _w_accel_d << endl;
        cout << "w_accel: "  << _w_accel << endl;
        //cout << "vehicle_Lf: "  << _Lf << endl;
        cout << "frequency: "   << _dt << endl;
        cout << "mpc_max_angvel: "  << _max_angvel << endl;
        cout << "max_throttle: "  << _max_throttle << endl;
        cout << "bound_value: "  << _bound_value << endl;
    }

    planner_util_.reconfigureCB(limits, false);
  }

    void MPCWayPointTrackerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, l_plan_pub_);
    }

    void MPCWayPointTrackerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, g_plan_pub_);
    }
  
	bool MPCWayPointTrackerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
        if( ! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        //Init parameters for MPC object
        _mpc_params["DT"] = _dt;
        //_mpc_params["LF"] = _Lf;
        _mpc_params["STEPS"]    = _mpc_steps;
        _mpc_params["REF_CTE"]  = _ref_cte;
        _mpc_params["REF_ETHETA"] = _ref_etheta;
        _mpc_params["REF_V"]    = _ref_vel;
        _mpc_params["W_CTE"]    = _w_cte;
        _mpc_params["W_EPSI"]   = _w_etheta;
        _mpc_params["W_V"]      = _w_vel;
        _mpc_params["W_ANGVEL"]  = _w_angvel;
        _mpc_params["W_A"]      = _w_accel;
        _mpc_params["W_DANGVEL"] = _w_angvel_d;
        _mpc_params["W_DA"]     = _w_accel_d;
        _mpc_params["ANGVEL"]   = _max_angvel;
        _mpc_params["MAXTHR"]   = _max_throttle;
        _mpc_params["BOUND"]    = _bound_value;
        _mpc.LoadParams(_mpc_params);

        latchedStopRotateController_.resetLatching();
        planner_util_.setPlan(orig_global_plan);
        
    }

    bool MPCWayPointTrackerROS::pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose, 
                                        std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot) {
        if (global_plan.empty())
            return true;
      
        try {
            // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
            geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, 
                                                                                        global_pose.header.frame_id, ros::Time(0));
            geometry_msgs::PoseStamped robot;
            tf2::doTransform(global_pose, robot, global_to_plan_transform);

            double dist_thresh_sq = dist_behind_robot*dist_behind_robot;

            // iterate plan until a pose close the robot is found
            std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
            std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
            while (it != global_plan.end()) {
              double dx = robot.pose.position.x - it->pose.position.x;
              double dy = robot.pose.position.y - it->pose.position.y;
              double dist_sq = dx * dx + dy * dy;
              if (dist_sq < dist_thresh_sq) {
                 erase_end = it;
                 break;
              }
              ++it;
            }
            if (erase_end == global_plan.end())
              return false;

            if (erase_end != global_plan.begin())
              global_plan.erase(global_plan.begin(), erase_end);

        } catch (const tf::TransformException& ex) {
            ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
            return false;
        }
        return true;
    }

    void MPCWayPointTrackerROS::updatePlanAndLocalCosts(
        const geometry_msgs::PoseStamped& global_pose,
        const std::vector<geometry_msgs::PoseStamped>& new_plan,
        const std::vector<geometry_msgs::Point>& footprint_spec) {
        
        global_plan_.resize(new_plan.size());
        for (unsigned int i = 0; i < new_plan.size(); ++i) {
            global_plan_[i] = new_plan[i];
        }

        pruneGlobalPlan(*tf_, global_pose, global_plan_, 0.5);
    }

	bool MPCWayPointTrackerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
        // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
        if ( ! costmap_ros_->getRobotPose(current_pose_)) {
            ROS_ERROR("Could not get robot pose");
            return false;
        }
        // Transform the global plan
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
            ROS_ERROR("Could not get local plan");
            return false;
        }
        //if the global plan passed in is empty... we won't do anything
        if(transformed_plan.empty()) {
            ROS_WARN_NAMED("mpc_planner", "Received an empty transformed plan.");
            return false;
        }
        ROS_DEBUG_NAMED("mpc_planner", "Received a transformed plan with %zu points.", transformed_plan.size());
        //ROS_INFO("**** Received a transformed plan with %zu points. ****", transformed_plan.size());
        updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());

        if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {
            //publish an empty plan because we've reached our goal position
            std::vector<geometry_msgs::PoseStamped> local_plan;
            std::vector<geometry_msgs::PoseStamped> transformed_plan;
            publishGlobalPlan(transformed_plan);
            publishLocalPlan(local_plan);
            base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
            ROS_WARN_NAMED("mpc_ros", "Reached the goal!!!.");
            return true;
        }
        bool isOk = mpcComputeVelocityCommands(current_pose_, cmd_vel);
        if (isOk) {
            publishGlobalPlan(transformed_plan);
        } else {
            ROS_WARN_NAMED("mpc_ros", "MPC Planner failed to produce path.");
            std::vector<geometry_msgs::PoseStamped> empty_plan;
            publishGlobalPlan(empty_plan);
        }
        return isOk;
    }

    // Timer: Control Loop (closed loop nonlinear MPC)
    bool MPCWayPointTrackerROS::mpcComputeVelocityCommands(geometry_msgs::PoseStamped global_pose, geometry_msgs::Twist& cmd_vel) {         
        // dynamic window sampling approach to get useful velocity commands
        if(! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        geometry_msgs::PoseStamped robot_vel;
        odom_helper_.getRobotVel(robot_vel);

        //compute what trajectory to drive along
        geometry_msgs::PoseStamped drive_cmds;
        drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();

        // call with updated footprint
        base_local_planner::Trajectory path = findBestPath(global_pose, robot_vel, drive_cmds);
        //base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
        //ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

        //pass along drive commands
        cmd_vel.linear.x = drive_cmds.pose.position.x;
        cmd_vel.linear.y = drive_cmds.pose.position.y;
        cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

        //if we cannot move... tell someone
        std::vector<geometry_msgs::PoseStamped> local_plan;
        if(path.cost_ < 0) {
            ROS_DEBUG_NAMED("mpc_ros",
                "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
            local_plan.clear();
            publishLocalPlan(local_plan);
            return false;
        }

        ROS_DEBUG_NAMED("mpc_ros", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", 
                        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

        // Fill out the local plan
        for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
            double p_x, p_y, p_th;
            path.getPoint(i, p_x, p_y, p_th);

            geometry_msgs::PoseStamped p;
            p.header.frame_id = costmap_ros_->getGlobalFrameID();
            p.header.stamp = ros::Time::now();
            p.pose.position.x = p_x;
            p.pose.position.y = p_y;
            p.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, p_th);
            tf2::convert(q, p.pose.orientation);
            local_plan.push_back(p);
        }

        //publish information to the visualizer
        publishLocalPlan(local_plan);
        return true;
    }

    base_local_planner::Trajectory MPCWayPointTrackerROS::findBestPath(const geometry_msgs::PoseStamped& global_pose,
                                                                       const geometry_msgs::PoseStamped& global_vel,
                                                                       geometry_msgs::PoseStamped& drive_velocities) {
        base_local_planner::Trajectory result_traj_;

        Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
        Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));
        geometry_msgs::PoseStamped goal_pose = global_plan_.back();
        Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
        base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
        result_traj_.cost_ = 1;

        /*
        *
        *  MPC Control Loop
        * 
        */
        //copy over the odometry information
        nav_msgs::Odometry base_odom = _odom;

        // Update system states: X=[x, y, theta, v]
        const double px = base_odom.pose.pose.position.x; //pose: odom frame
        const double py = base_odom.pose.pose.position.y;
        tf::Pose pose;
        tf::poseMsgToTF(base_odom.pose.pose, pose);
        double theta = tf::getYaw(pose.getRotation());
        const double v = base_odom.twist.twist.linear.x; //twist: body fixed frame
        // Update system inputs: U=[w, throttle]
        const double w = _w; // steering -> w
        //const double steering = _steering;  // radian
        const double throttle = _throttle; // accel: >0; brake: <0
        const double dt = _dt;

        //Update path waypoints (conversion to odom frame)
        nav_msgs::Path odom_path = nav_msgs::Path();
        try {
            double total_length = 0.0;
            int sampling = _downSampling;

            //find waypoints distance
            if(_waypointsDist <= 0.0) {        
                double dx = global_plan_[1].pose.position.x - global_plan_[0].pose.position.x;
                double dy = global_plan_[1].pose.position.y - global_plan_[0].pose.position.y;
                _waypointsDist = hypot(dx,dy);
                _downSampling = int(_pathLength/10.0/_waypointsDist);
            }            

            // Cut and downsampling the path
            for(int i =0; i< global_plan_.size(); i++) {
                if(total_length > _pathLength) {
                    break;
                }
                if(sampling == _downSampling) {
                    geometry_msgs::PoseStamped tempPose;
                    tf2_ros::TransformListener tfListener(*tf_);
                    geometry_msgs::TransformStamped odom_transform;
                    odom_transform = tf_->lookupTransform(_odom_frame, _map_frame, ros::Time(0), ros::Duration(1.0) );
                    tf2::doTransform(global_plan_[i], tempPose, odom_transform); // robot_pose is the PoseStamp                     
                    odom_path.poses.push_back(tempPose);  
                    sampling = 0;
                }
                total_length = total_length + _waypointsDist; 
                sampling = sampling + 1;;  
            }
           
            if(odom_path.poses.size() > 3) {
                // publish odom path
                odom_path.header.frame_id = _odom_frame;
                odom_path.header.stamp = ros::Time::now();
                _pub_odompath.publish(odom_path);
            } else {
                ROS_DEBUG_NAMED("mpc_ros", "Failed to path generation since small down-sampling path.");
                _waypointsDist = -1;
                result_traj_.cost_ = -1;
                return result_traj_;
            }
            //DEBUG      
            if(_debug_info) {
                cout << endl << "odom_path: " << odom_path.poses.size()
                << ", path[0]: " << odom_path.poses[0]
                << ", path[N]: " << odom_path.poses[odom_path.poses.size()-1] << endl;
            }  
        } catch(tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        
        // Waypoints related parameters
        const int N = odom_path.poses.size(); // Number of waypoints
        const double costheta = cos(theta);
        const double sintheta = sin(theta);
        if(_debug_info) {
            cout << "px, py : " << px << ", "<< py << ", theta: " << theta << " , N: " << N << endl;
        }
        // Convert to the vehicle coordinate system
        VectorXd x_veh(N);
        VectorXd y_veh(N);
        for(int i = 0; i < N; i++) {
            const double dx = odom_path.poses[i].pose.position.x - px;
            const double dy = odom_path.poses[i].pose.position.y - py;
            x_veh[i] = dx * costheta + dy * sintheta;
            y_veh[i] = dy * costheta - dx * sintheta;
            if(_debug_info)
                cout << "x_veh : " << x_veh[i]<< ", y_veh: " << y_veh[i] << endl;
        }

        // Fit waypoints
        auto coeffs = polyfit(x_veh, y_veh, 3); 
        const double cte  = polyeval(coeffs, 0.0);
        if(_debug_info) {
            cout << "coeffs : " << coeffs[0] << endl;
            cout << "pow : " << pow(0.0 ,0) << endl;
            cout << "cte : " << cte << endl;
        }
        double etheta = atan(coeffs[1]);

        // Global coordinate system about theta
        double gx = 0;
        double gy = 0;
        int N_sample = N * 0.3;
        for(int i = 1; i < N_sample; i++) {
            gx += odom_path.poses[i].pose.position.x - odom_path.poses[i-1].pose.position.x;
            gy += odom_path.poses[i].pose.position.y - odom_path.poses[i-1].pose.position.y;
        }   

        double temp_theta = theta;
        double traj_deg = atan2(gy,gx);
        //double PI = 3.141592;

        // Degree conversion -pi~pi -> 0~2pi(ccw) since need a continuity        
        if(temp_theta <= -M_PI + traj_deg) 
            temp_theta = temp_theta + 2 * M_PI;
        
        // Implementation about theta error more precisly
        if(gx && gy && temp_theta - traj_deg < 1.8 * M_PI)
            etheta = temp_theta - traj_deg;
        else
            etheta = 0;

        // Difference bewteen current position and goal position
        const double x_err = goal_pose.pose.position.x -  base_odom.pose.pose.position.x;
        const double y_err = goal_pose.pose.position.y -  base_odom.pose.pose.position.y;
        const double goal_err = sqrt(x_err*x_err + y_err*y_err);

        if(_debug_info) {
            cout << "etheta: "<< etheta << ", atan2(gy,gx): " << atan2(gy,gx) << ", temp_theta:" << traj_deg << endl;
            cout << "x_err:"<< x_err << ", y_err:"<< y_err  << endl;
        }

        VectorXd state(6);
        if(_delay_mode) {
            // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt)
            const double px_act = v * dt;
            const double py_act = 0;
            const double theta_act = w * dt; //(steering) theta_act = v * steering * dt / Lf;
            const double v_act = v + throttle * dt; //v = v + a * dt
            
            const double cte_act = cte + v * sin(etheta) * dt;
            const double etheta_act = etheta - theta_act;  
            
            state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
        } else {
            state << 0, 0, 0, v, cte, etheta;
        }

        // Solve MPC Problem
        ros::Time begin = ros::Time::now();
        vector<double> mpc_results = _mpc.Solve(state, coeffs);    
        ros::Time end = ros::Time::now();
            
        // MPC result (all described in car frame), output = (acceleration, w)        
        _w = mpc_results[0]; // radian/sec, angular velocity
        _throttle = mpc_results[1]; // acceleration

        _speed = v + _throttle * dt;  // speed
        if (_speed >= _max_speed)
            _speed = _max_speed;
        if(_speed <= 0.0)
            _speed = 0.0;

        if(_debug_info) {
            cout << "--------------- Duration: " << end.sec << "." << end.nsec << endl << begin.sec<< "."  << begin.nsec << endl;
            cout << "\n\nDEBUG" << endl;
            cout << "theta: " << theta << endl;
            cout << "V: " << v << endl;
            //cout << "odom_path: \n" << odom_path << endl;
            //cout << "x_points: \n" << x_veh << endl;
            //cout << "y_points: \n" << y_veh << endl;
            cout << "coeffs: \n" << coeffs << endl;
            cout << "_w: \n" << _w << endl;
            cout << "_throttle: \n" << _throttle << endl;
            cout << "_speed: \n" << _speed << endl;
        }
        // Display the MPC predicted trajectory
        _mpc_traj = nav_msgs::Path();
        _mpc_traj.header.frame_id = _base_frame; // points in car coordinate        
        _mpc_traj.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped tempPose;
        tf2::Quaternion myQuaternion;

        for(int i=0; i<_mpc.mpc_x.size(); i++) {
            tempPose.header = _mpc_traj.header;
            tempPose.pose.position.x = _mpc.mpc_x[i];
            tempPose.pose.position.y = _mpc.mpc_y[i];

            myQuaternion.setRPY( 0, 0, _mpc.mpc_theta[i] );  
            tempPose.pose.orientation.x = myQuaternion[0];
            tempPose.pose.orientation.y = myQuaternion[1];
            tempPose.pose.orientation.z = myQuaternion[2];
            tempPose.pose.orientation.w = myQuaternion[3];
                
            _mpc_traj.poses.push_back(tempPose); 
        }     

        if(result_traj_.cost_ < 0) {
            drive_velocities.pose.position.x = 0;
            drive_velocities.pose.position.y = 0;
            drive_velocities.pose.position.z = 0;
            drive_velocities.pose.orientation.w = 1;
            drive_velocities.pose.orientation.x = 0;
            drive_velocities.pose.orientation.y = 0;
            drive_velocities.pose.orientation.z = 0;
        } else {
            drive_velocities.pose.position.x = _speed;
            drive_velocities.pose.position.y = 0;
            drive_velocities.pose.position.z = 0;
            tf2::Quaternion q;
            q.setRPY(0, 0, _w);
            tf2::convert(q, drive_velocities.pose.orientation);
        }
        
        // publish the mpc trajectory
        _pub_mpctraj.publish(_mpc_traj);
        return result_traj_;
    }

	bool MPCWayPointTrackerROS::isGoalReached() {
        if( ! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if ( ! costmap_ros_->getRobotPose(current_pose_)) {
            ROS_ERROR("Could not get robot pose");
            return false;
        }
        if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
            ROS_INFO("Goal reached");
            return true;
        } else {
            return false;
        }
    }

    // Evaluate a polynomial.
    double MPCWayPointTrackerROS::polyeval(Eigen::VectorXd coeffs, double x) {
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); i++) {
            result += coeffs[i] * pow(x, i);
        }
        return result;
    }

    // Fit a polynomial.
    // Adapted from
    // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
    Eigen::VectorXd MPCWayPointTrackerROS::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
        assert(xvals.size() == yvals.size());
        assert(order >= 1 && order <= xvals.size() - 1);
        Eigen::MatrixXd A(xvals.size(), order + 1);

        for (int i = 0; i < xvals.size(); i++)
            A(i, 0) = 1.0;

        for (int j = 0; j < xvals.size(); j++) {
            for (int i = 0; i < order; i++) 
                A(j, i + 1) = A(j, i) * xvals(j);
        }
        auto Q = A.householderQr();
        auto result = Q.solve(yvals);
        return result;
    }

    // CallBack: Update odometry
    void MPCWayPointTrackerROS::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg) {
        _odom = *odomMsg;
    }
}