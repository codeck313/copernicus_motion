#include "dwa_planner/dwa_planner.h"

DWAPlanner::DWAPlanner(void)
    :local_nh("~"), scan_updated(false), odom_updated(false)
{
    init();
    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    candidate_trajectories_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);

    selected_trajectory_pub = local_nh.advertise<visualization_msgs::Marker>("selected_trajectory", 1);

    scan_sub = nh.subscribe("/scan", 1, &DWAPlanner::scan_callback, this);
    odom_sub = nh.subscribe("/odom", 1000, &DWAPlanner::odom_callback, this);
}

void DWAPlanner::scan_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
    scan = *msg;
    scan_updated = true;
}

void DWAPlanner::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    current_velocity = msg->twist.twist;
    current_pos.pose =  msg->pose.pose;
    // ROS_INFO_STREAM("$$ Current Loction"<<current_pos.pose.position.x);
    odom_updated = true;
}

DWAPlanner::Window DWAPlanner::calc_dynamic_window(const geometry_msgs::Twist &current_velocity)
{
    Window window(MIN_VELOCITY, MAX_VELOCITY, -MAX_YAWRATE, MAX_YAWRATE);
    window.min_velocity = std::max((current_velocity.linear.x - MAX_ACCELERATION * DT), MIN_VELOCITY);
    window.max_velocity = std::min((current_velocity.linear.x + MAX_ACCELERATION * DT), MAX_VELOCITY);
    window.min_yawrate = std::max((current_velocity.angular.z - MAX_D_YAWRATE * DT), -MAX_YAWRATE);
    window.max_yawrate = std::min((current_velocity.angular.z + MAX_D_YAWRATE * DT), MAX_YAWRATE);
    return window;
}

float DWAPlanner::calc_to_goal_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal)
{
    Eigen::Vector3d last_position(traj.back().x, traj.back().y, traj.back().yaw);
    return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}

float DWAPlanner::calc_goal_heading_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal)
{
    float xDiff = goal[0] - traj.back().x;
    float yDiff = goal[1] - traj.back().y;
    float targetCost = atan2(yDiff, xDiff) - traj.back().yaw;
    return abs(targetCost);
}

float DWAPlanner::calc_speed_cost(const std::vector<State> &traj, const float max_velocity)
{
    float cost = fabs(max_velocity - fabs(traj[traj.size() - 1].velocity));
    return cost;
}

float DWAPlanner::calc_obstacle_cost(const std::vector<State> &traj, const std::vector<std::vector<float>> &obs_list)
{
    float cost = 0.0;
    float min_dist = 1e3;
    for (const auto &state : traj)
    {
        for (const auto &obs : obs_list)
        {
            float dist = sqrt((state.x - obs[0]) * (state.x - obs[0]) + (state.y - obs[1]) * (state.y - obs[1]));
            if (dist <= local_map.info.resolution)
            {
                cost = 1e6;
                return cost;
            }
            min_dist = std::min(min_dist, dist);
            if (min_dist <= LENGTH_ROBOT)
            {
                min_dist = INFINITY;
            }
        }
    }
    if (isinf(min_dist))
    {
        return INFINITY;
    }
    cost = 1.0 / min_dist;
    return cost;
}

void DWAPlanner::motion(State &state, const double velocity, const double yawrate)
{
    state.yaw += yawrate * DT;
    state.x += velocity * std::cos(state.yaw) * DT;
    state.y += velocity * std::sin(state.yaw) * DT;
    state.velocity = velocity;
    state.yawrate = yawrate;
}

std::vector<std::vector<float>> DWAPlanner::scan_to_obs()
{
    std::vector<std::vector<float>> obs_list;
    float angle = scan.angle_min;
    for (auto r : scan.ranges)
    {
        float x = r * cos(angle);
        float y = r * sin(angle);
        std::vector<float> obs_state = {x, y};
        obs_list.push_back(obs_state);
        angle += scan.angle_increment;
    }
    return obs_list;
}

std::vector<DWAPlanner::State> DWAPlanner::dwa_planning(
        Window dynamic_window,
        Eigen::Vector3d goal,
        std::vector<std::vector<float>> obs_list)
{
    float min_cost = 1e6;
    float min_obs_cost = min_cost;
    float min_goal_cost = min_cost;
    float min_heading_cost = min_cost;
    float min_speed_cost = min_cost;

    std::vector<std::vector<State>> trajectories;
    std::vector<State> best_traj;

    for(float v=dynamic_window.min_velocity; v<=dynamic_window.max_velocity; v+=VELOCITY_RESOLUTION){
        for(float y=dynamic_window.min_yawrate; y<=dynamic_window.max_yawrate; y+=YAWRATE_RESOLUTION){
            State state(current_pos.pose.position.x, current_pos.pose.position.y, current_pos.pose.position.z, current_velocity.linear.x, current_velocity.angular.z);
            std::vector<State> traj;
            for(float t=0; t<=PREDICT_TIME; t+=DT){
                motion(state, v, y);
                traj.push_back(state);
            }
            trajectories.push_back(traj);

            float to_goal_cost = calc_to_goal_cost(traj, goal);
            float speed_cost = calc_speed_cost(traj, MAX_VELOCITY);
            float obstacle_cost = calc_obstacle_cost(traj, obs_list);
            float heading_cost = calc_goal_heading_cost(traj, goal);
            float final_cost = TO_GOAL_COST_GAIN * to_goal_cost + SPEED_COST_GAIN * speed_cost + OBSTACLE_COST_GAIN * obstacle_cost + HEADING_COST_GAIN * heading_cost;
            if(min_cost >= final_cost){
                min_goal_cost = TO_GOAL_COST_GAIN*to_goal_cost;
                min_heading_cost = HEADING_COST_GAIN * heading_cost;
                min_obs_cost = OBSTACLE_COST_GAIN*obstacle_cost;
                min_speed_cost = SPEED_COST_GAIN*speed_cost;
                min_cost = final_cost;
                best_traj = traj;
            }
        }
    }
    v_all_trajectories(trajectories, 0, 1, 0, candidate_trajectories_pub);
    ROS_INFO_STREAM("Cost: " << min_cost);
    ROS_INFO_STREAM("- Goal Distance cost: " << min_goal_cost);
    ROS_INFO_STREAM("- Heading cost: " << min_heading_cost);
    ROS_INFO_STREAM("- Obs cost: " << min_obs_cost);
    ROS_INFO_STREAM("- Speed cost: " << min_speed_cost);
    ROS_INFO_STREAM("num of trajectories: " << trajectories.size());
    ROS_INFO_STREAM("Current location : (" << current_pos.pose.position.x << "," << current_pos.pose.position.y << "," << current_pos.pose.position.z << ")" );
    if(min_cost == 1e6){
        std::vector<State> traj;
        State state(current_pos.pose.position.x, current_pos.pose.position.y, current_pos.pose.position.z, current_velocity.linear.x, current_velocity.angular.z);
        traj.push_back(state);
        best_traj = traj;
    }
    return best_traj;
}

void DWAPlanner::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        ROS_INFO("==========================================");
        double start = ros::Time::now().toSec();
        bool input_updated = false;
        if(scan_updated){
            input_updated = true;
        }
        if(input_updated &&  odom_updated){
            Window dynamic_window = calc_dynamic_window(current_velocity);
            // Eigen::Vector3d goal(local_goal.pose.position.x, local_goal.pose.position.y, tf::getYaw(local_goal.pose.orientation));
            Eigen::Vector3d goal(GOAL_X, GOAL_Y, 0.0);
            Eigen::Vector3d currentPOS(current_pos.pose.position.x, current_pos.pose.position.y, 0.0);
            ROS_INFO_STREAM("local goal: (" << goal[0] << "," << goal[1] << "," << goal[2]/M_PI*180 << ")");
            geometry_msgs::Twist cmd_vel;
            if ((goal.segment(0, 2) - currentPOS.segment(0, 2)).norm() > GOAL_THRESHOLD)
            {
                std::vector<std::vector<float>> obs_list;

                obs_list = scan_to_obs();
                scan_updated = false;

                std::vector<State> best_traj = dwa_planning(dynamic_window, goal, obs_list);
                v_best_trajectory(best_traj, 1, 0, 0, selected_trajectory_pub);

                cmd_vel.linear.x = best_traj[0].velocity;
                cmd_vel.angular.z = best_traj[0].yawrate;
            }

            else{
                cmd_vel.linear.x = 0.0;
                if(fabs(goal[2])>TURN_DIRECTION_THRESHOLD){
                    cmd_vel.angular.z = std::min(std::max(goal(2), -MAX_YAWRATE), MAX_YAWRATE);
                }
                else{
                    cmd_vel.angular.z = 0.0;
                }
            }
            ROS_INFO_STREAM("cmd_vel: (" << cmd_vel.linear.x << "[m/s], " << cmd_vel.angular.z << "[rad/s])");
            velocity_pub.publish(cmd_vel);

            odom_updated = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO_STREAM("loop time: " << ros::Time::now().toSec() - start << "[s]");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dwa_planner");
    DWAPlanner planner;
    planner.process();
    return 0;
}

DWAPlanner::State::State(double _x, double _y, double _yaw, double _velocity, double _yawrate)
    : x(_x), y(_y), yaw(_yaw), velocity(_velocity), yawrate(_yawrate)
{
}

DWAPlanner::Window::Window(void)
    : min_velocity(0.0), max_velocity(0.0), min_yawrate(0.0), max_yawrate(0.0)
{
}

DWAPlanner::Window::Window(const double min_v, const double max_v, const double min_y, const double max_y)
    : min_velocity(min_v), max_velocity(max_v), min_yawrate(min_y), max_yawrate(max_y)
{
}

void DWAPlanner::init(void)
{
    local_nh.param("HZ", HZ, {10});
    local_nh.param("MAX_VELOCITY", MAX_VELOCITY, {1.0});
    local_nh.param("MIN_VELOCITY", MIN_VELOCITY, {0.0});
    local_nh.param("MAX_YAWRATE", MAX_YAWRATE, {1.0});
    local_nh.param("MAX_ACCELERATION", MAX_ACCELERATION, {1.0});
    local_nh.param("MAX_D_YAWRATE", MAX_D_YAWRATE, {1.0});
    local_nh.param("VELOCITY_RESOLUTION", VELOCITY_RESOLUTION, {0.01});
    local_nh.param("YAWRATE_RESOLUTION", YAWRATE_RESOLUTION, {0.01});
    local_nh.param("PREDICT_TIME", PREDICT_TIME, {3.0});
    local_nh.param("TO_GOAL_COST_GAIN", TO_GOAL_COST_GAIN, {0.5});
    local_nh.param("HEADING_COST_GAIN", HEADING_COST_GAIN, {1.0});
    local_nh.param("SPEED_COST_GAIN", SPEED_COST_GAIN, {1.0});
    local_nh.param("OBSTACLE_COST_GAIN", OBSTACLE_COST_GAIN, {1.0});
    local_nh.param("GOAL_THRESHOLD", GOAL_THRESHOLD, {0.3});
    local_nh.param("TURN_DIRECTION_THRESHOLD", TURN_DIRECTION_THRESHOLD, {1.0});
    local_nh.param("LENGTH_ROBOT", LENGTH_ROBOT, {1.2});
    local_nh.param("WIDTH_ROBOT", WIDTH_ROBOT, {0.8});
    local_nh.param("GOAL_X", GOAL_X, {4.0});
    local_nh.param("GOAL_Y", GOAL_Y, {0.0});
    DT = 1.0 / HZ;

    ROS_INFO("=== DWA Planner ===");
    ROS_INFO_STREAM("HZ: " << HZ);
    ROS_INFO_STREAM("DT: " << DT);
    ROS_INFO_STREAM("MAX_VELOCITY: " << MAX_VELOCITY);
    ROS_INFO_STREAM("MIN_VELOCITY: " << MIN_VELOCITY);
    ROS_INFO_STREAM("MAX_YAWRATE: " << MAX_YAWRATE);
    ROS_INFO_STREAM("MAX_ACCELERATION: " << MAX_ACCELERATION);
    ROS_INFO_STREAM("MAX_D_YAWRATE: " << MAX_D_YAWRATE);
    ROS_INFO_STREAM("VELOCITY_RESOLUTION: " << VELOCITY_RESOLUTION);
    ROS_INFO_STREAM("YAWRATE_RESOLUTION: " << YAWRATE_RESOLUTION);
    ROS_INFO_STREAM("PREDICT_TIME: " << PREDICT_TIME);
    ROS_INFO_STREAM("TO_GOAL_COST_GAIN: " << TO_GOAL_COST_GAIN);
    ROS_INFO_STREAM("HEADING_COST_GAIN: " << HEADING_COST_GAIN);
    ROS_INFO_STREAM("SPEED_COST_GAIN: " << SPEED_COST_GAIN);
    ROS_INFO_STREAM("OBSTACLE_COST_GAIN: " << OBSTACLE_COST_GAIN);
    ROS_INFO_STREAM("GOAL_THRESHOLD: " << GOAL_THRESHOLD);
    ROS_INFO_STREAM("GOAL_X" << GOAL_X);
    ROS_INFO_STREAM("GOAL_Y" << GOAL_Y);
    ROS_INFO_STREAM("TURN_DIRECTION_THRESHOLD: " << TURN_DIRECTION_THRESHOLD);
}

void DWAPlanner::v_best_trajectory(const std::vector<State> &traj, const double r, const double g, const double b, const ros::Publisher &pub)
{
    visualization_msgs::Marker bestTrajectory;
    bestTrajectory.header.frame_id = "base_link";
    bestTrajectory.header.stamp = ros::Time::now();
    bestTrajectory.color.r = r;
    bestTrajectory.color.g = g;
    bestTrajectory.color.b = b;
    bestTrajectory.color.a = 0.8;
    bestTrajectory.ns = pub.getTopic();
    bestTrajectory.type = visualization_msgs::Marker::LINE_STRIP;
    bestTrajectory.action = visualization_msgs::Marker::ADD;
    bestTrajectory.lifetime = ros::Duration();
    bestTrajectory.scale.x = 0.05;
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    bestTrajectory.pose = pose;
    geometry_msgs::Point p;
    for (const auto &pose : traj)
    {
        p.x = pose.x;
        p.y = pose.y;
        bestTrajectory.points.push_back(p);
    }
    pub.publish(bestTrajectory);
}

void DWAPlanner::v_all_trajectories(const std::vector<std::vector<State>> &trajs, const double r, const double g, const double b, const ros::Publisher &pub)
{
    visualization_msgs::MarkerArray allTraj;
    int count = 0;
    const int size = trajs.size();
    for (; count < size; count += 10)
    {
        visualization_msgs::Marker v_traj;
        v_traj.header.frame_id = "base_link";
        v_traj.header.stamp = ros::Time::now();
        v_traj.color.r = r;
        v_traj.color.g = g;
        v_traj.color.b = b;
        v_traj.color.a = 0.5;
        v_traj.ns = pub.getTopic();
        v_traj.type = visualization_msgs::Marker::LINE_STRIP;
        v_traj.action = visualization_msgs::Marker::ADD;
        v_traj.lifetime = ros::Duration();
        v_traj.id = count;
        v_traj.scale.x = 0.02;
        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        v_traj.pose = pose;
        geometry_msgs::Point p;
        for (const auto &pose : trajs[count])
        {
            p.x = pose.x;
            p.y = pose.y;
            v_traj.points.push_back(p);
        }
        allTraj.markers.push_back(v_traj);
    }
    pub.publish(allTraj);
}
