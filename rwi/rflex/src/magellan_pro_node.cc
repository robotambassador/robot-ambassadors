#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <rflex/magellan_pro_driver.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>
#include <math.h>

/**
 *  \brief B21 Node for ROS
 *  By David Lu!! 2/2010
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
class MagellanProNode {
    private:
        MAGELLAN_PRO driver;

        ros::Subscriber subs[4];			///< Subscriber handles (cmd_vel, cmd_accel, cmd_sonar_power, cmd_brake_power)
        ros::Publisher sonar_pub;			///< Sonar Publisher for Sonars (sonar_cloud)
        ros::Publisher voltage_pub;			///< Voltage Publisher (voltage)
        ros::Publisher brake_power_pub;		///< Brake Power Publisher (brake_power)
        ros::Publisher sonar_power_pub;		///< Sonar Power Publisher (sonar_power)
        ros::Publisher odom_pub;			///< Odometry Publisher (odom)
        ros::Publisher plugged_pub;			///< Plugged In Publisher (plugged_in)
        ros::Publisher joint_pub; 			///< Joint State Publisher (state)
        //ros::Publisher bump_pub; 			///< Bump Publisher (bumps)
        tf::TransformBroadcaster broadcaster; ///< Transform Broadcaster (for odom)

        bool isSonarOn, isBrakeOn;
        float acceleration;

        // Odometry related variables
        float lastTrans, lastRot;
        ros::Time current_time, last_time;
        float oldx, x, oldy, y, oldth, th;
        float R, w;

        float cmdTranslation, cmdRotation;
        bool brake_dirty, sonar_dirty;
        bool initialized;
        float first_bearing;
        int updateTimer;
        int prev_bumps;
        bool sonar_just_on;

        void publishOdometry();
        void publishSonar();
        //void publishBumps();

    public:
        ros::NodeHandle n;
        MagellanProNode();
        ~MagellanProNode();
        int initialize(const char* port);
        void spinOnce();

        // Message Listeners
        void NewCommand      (const geometry_msgs::Twist::ConstPtr& msg);
        void SetAcceleration (const std_msgs::Float32   ::ConstPtr& msg);
        void ToggleSonarPower(const std_msgs::Bool      ::ConstPtr& msg);
        void ToggleBrakePower(const std_msgs::Bool      ::ConstPtr& msg);
};

MagellanProNode::MagellanProNode() : n ("~") {
    isSonarOn = isBrakeOn = false;
    brake_dirty = sonar_dirty = false;
    sonar_just_on = false;
    cmdTranslation = cmdRotation = 0.0;
    updateTimer = 99;
    initialized = false;
    prev_bumps = 0;
    subs[0] = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1,   &MagellanProNode::NewCommand, this);
    subs[1] = n.subscribe<std_msgs::Float32>("cmd_accel", 1,     &MagellanProNode::SetAcceleration, this);
    subs[2] = n.subscribe<std_msgs::Bool>("cmd_sonar_power", 1, &MagellanProNode::ToggleSonarPower, this);
    subs[3] = n.subscribe<std_msgs::Bool>("cmd_brake_power", 1, &MagellanProNode::ToggleBrakePower, this);
    acceleration = 0.7;
    //acceleration = 30000;

    sonar_pub = n.advertise<sensor_msgs::PointCloud>("sonar_cloud", 50);
    sonar_power_pub = n.advertise<std_msgs::Bool>("sonar_power", 1);
    brake_power_pub = n.advertise<std_msgs::Bool>("brake_power", 1);
    voltage_pub = n.advertise<std_msgs::Float32>("voltage", 1);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    plugged_pub = n.advertise<std_msgs::Bool>("plugged_in", 1);
    joint_pub = n.advertise<sensor_msgs::JointState>("state", 1);
    //bump_pub = n.advertise<sensor_msgs::PointCloud>("bump", 5);
}

int MagellanProNode::initialize(const char* port) {
    int ret = driver.initialize(port);
    if (ret < 0)
        return ret;

    driver.setOdometryPeriod(100); // in ms
    driver.setDigitalIoPeriod(1000);
    driver.motionSetDefaults();

    return 0;
}

MagellanProNode::~MagellanProNode() {
    driver.motionSetDefaults();
    driver.setOdometryPeriod(0);
    driver.setDigitalIoPeriod(0);
    driver.setSonarPower(false);
    driver.setIrPower(false);
}

/// cmd_vel callback
void MagellanProNode::NewCommand(const geometry_msgs::Twist::ConstPtr& msg) {
    cmdTranslation = msg->linear.x;
    cmdRotation = msg->angular.z;
}

/// cmd_acceleration callback
void MagellanProNode::SetAcceleration (const std_msgs::Float32::ConstPtr& msg) {
    acceleration = msg->data;
}

/// cmd_sonar_power callback
void MagellanProNode::ToggleSonarPower(const std_msgs::Bool::ConstPtr& msg) {
    isSonarOn=msg->data;
    sonar_dirty = true;
}

/// cmd_brake_power callback
void MagellanProNode::ToggleBrakePower(const std_msgs::Bool::ConstPtr& msg) {
    isBrakeOn = msg->data;
    brake_dirty = true;
}

void MagellanProNode::spinOnce() {
    // Sending the status command too often overwhelms the driver
    if (updateTimer>=100) {
        driver.sendSystemStatusCommand();
        updateTimer = 0;
    }
    updateTimer++;

    //printf("In SpinOnce\n");

    if (cmdTranslation != 0 || cmdRotation != 0)
        driver.setMovement(cmdTranslation, cmdRotation, acceleration);

    if (sonar_dirty) {
        driver.setSonarPower(isSonarOn);
        sonar_dirty = false;
        driver.sendSystemStatusCommand();
    }
    if (brake_dirty) {
        driver.setBrakePower(isBrakeOn);
        brake_dirty = false;
        updateTimer = 99;
    }

    std_msgs::Bool bmsg;
    bmsg.data = isSonarOn;
    sonar_power_pub.publish(bmsg);
    bmsg.data = driver.getBrakePower();
    brake_power_pub.publish(bmsg);
    bmsg.data = driver.isPluggedIn();
    plugged_pub.publish(bmsg);
    std_msgs::Float32 vmsg;
    vmsg.data = driver.getVoltage();
    voltage_pub.publish(vmsg);

    publishOdometry();
    publishSonar();
    //publishBumps();
}

/** Integrates over the lastest raw odometry readings from
 * the driver to get x, y and theta */
void MagellanProNode::publishOdometry() {
    if (!driver.isOdomReady()) {
        return;
    }

    if (!initialized) {
    	current_time  = ros::Time::now();
    	last_time = ros::Time::now();

    	lastTrans = 0.0;
    	lastRot = 0.0;

    	oldx = 0.0;
    	oldy = 0.0;
    	oldth = 0.0;

    	x = 0.0;
    	y = 0.0;
    	th = 0.0;

    	R = 0.0;
    	w = 0.0;

    	initialized = true;
    }
    else {

    	// Update time (s)
    	current_time = ros::Time::now();

    	// Update distances (m)
    	float trans = driver.getTransDistance();
    	// (rads)
    	float rot = driver.getRotDistance(); //rotation distance since startup

    	// Find distance differences (m)
    	float dTrans = trans - lastTrans;
    	float dRot = rot - lastRot;  //difference in rotation from last call

	if (dTrans < -2.0 || dTrans > 2.0) return;

    	// Find time difference (s)

    	float dTime = (current_time - last_time).toSec();

    	// Find th, x and y (m)
      /*
    	float dth = dRot / 0.14; // r = 0.14 (base radius)
    	
    	//th += dth; // and back to distance from startup
    	
      */
      

      // attempt to fix it accurately...doesnt work
      float radius = dTrans/dRot;
      float th = acos(x/radius);
      float alpha = dRot + th;
      /*
      x = radius * cos(alpha);
      y = radius * sin(alpha);
      */

      float dx = dTrans * cos(rot);
    	float dy = dTrans * sin(rot);
    	
    	//x += dx;
    	//y += dy;

    	x = x + radius*(cos(rot) - cos(lastRot));
    	y = y + radius*(cos(rot) - cos(lastRot));
         
           
    	//since all odometry is 6DOF we'll need a quaternion created from yaw
    	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(rot);

    	//first, we'll publish the transform over tf
    	geometry_msgs::TransformStamped odom_trans;
    	odom_trans.header.stamp = current_time;
    	odom_trans.header.frame_id = "odom";
    	odom_trans.child_frame_id = "base_link";

    	odom_trans.transform.translation.x = x;
    	odom_trans.transform.translation.y = y;
    	odom_trans.transform.translation.z = 0.0;
    	odom_trans.transform.rotation = odom_quat;

    	//send the transform
    	broadcaster.sendTransform(odom_trans);

    	//next, we'll publish the odometry message over ROS
    	nav_msgs::Odometry odom;
    	odom.header.stamp = current_time;
    	odom.header.frame_id = "odom";

    	double rotCovar = 1.0;
    	if (dRot == 0) {rotCovar = 0.00000000001;}

    	//set the position
    	odom.pose.pose.position.x = x;
    	odom.pose.pose.position.y = y;
    	odom.pose.pose.position.z = 0.0;
    	odom.pose.pose.orientation = odom_quat;
    	/*
	odom.pose.covariance.elems = {0.00001, 0, 0, 0, 0, 0,
    	                        0, 0.00001, 0, 0, 0, 0,
    	                        0, 0, 10.0000, 0, 0, 0,
    	                        0, 0, 0, 1.00000, 0, 0,
    	                        0, 0, 0, 0, 1.00000, 0,
    	                        0, 0, 0, 0, 0, rotCovar};
	*/

    	//set the velocity
    	odom.child_frame_id = "base_link";
    	odom.twist.twist.linear.x = dTrans / dTime;
    	odom.twist.twist.linear.y = 0;
    	odom.twist.twist.angular.z = dRot / dTime;

    	//publish the message
    	odom_pub.publish(odom);

    	lastTrans = trans;
    	lastRot = rot;

    	last_time = current_time;

    	//float dTrans = (dDistRight - dDistLeft)*0.07/2.0;
        //float dRot = (dDistRight - dDistLeft)*0.07/(2.0*0.32);

        /*//integrate latest motion into odometry
        x_odo += dTrans * cos(a_odo);
        y_odo += dTrans * sin(a_odo);
        a_odo += dTrans;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(a_odo);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x_odo;
        odom_trans.transform.translation.y = y_odo;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x_odo;
        odom.pose.pose.position.y = y_odo;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        float tvel = driver.getTranslationalVelocity();
        odom.twist.twist.linear.x = tvel*cos(a_odo);
        odom.twist.twist.linear.y = tvel*sin(a_odo);
        odom.twist.twist.angular.z = driver.getRotationalVelocity();

        //publish the message
        odom_pub.publish(odom);

        // finally, publish the joint state
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.set_name_size(1);
        joint_state.set_position_size(1);
        joint_state.name[0] = "godot_twist";
        joint_state.position[0] = 0;

        joint_pub.publish(joint_state);*/
    }
}

void MagellanProNode::publishSonar() {
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "magellan";
/*
    if (isSonarOn) {
        driver.getSonarPoints(&cloud);
        sonar_pub.publish(cloud);

    } else if (sonar_just_on) {
        sonar_pub.publish(cloud);
    }*/
}

/*void B21Node::publishBumps() {
    sensor_msgs::PointCloud cloud1, cloud2;
    cloud1.header.stamp = ros::Time::now();
    cloud2.header.stamp = ros::Time::now();
    cloud1.header.frame_id = "base";
    cloud2.header.frame_id = "body";
    int bumps = driver.getBaseBumps(&cloud1) +
                driver.getBodyBumps(&cloud2);

    if (bumps>0 || prev_bumps>0) {
        bump_pub.publish(cloud1);
        bump_pub.publish(cloud2);
    }
    prev_bumps = bumps;
}*/

int main(int argc, char** argv) {
    ros::init(argc, argv, "magellan_pro");
    MagellanProNode node;
    std::string port;
    node.n.param<std::string>("port", port, "/dev/ttyS1");
    ROS_INFO("Attempting to connect to %s", port.c_str());
    if (node.initialize(port.c_str())<0) {
        ROS_ERROR("Could not initialize RFLEX driver!\n");
        return 0;
    }
    ROS_INFO("Connected!");

    int hz;
    node.n.param("rate", hz, 10);
    ros::Rate loop_rate(hz);

    while (ros::ok()) {
        node.spinOnce();
        // Process a round of subscription messages
        ros::spinOnce();
        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}

