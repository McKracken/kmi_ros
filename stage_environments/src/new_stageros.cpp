/*
 *  stageros
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 */

/**

@mainpage

@htmlinclude manifest.html
**/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>


// libstage
#include <stage.hh>

// roscpp
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/LaserScan.h>
// #include <new_stage/Range.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <roslib/Clock.h>

#include "tf/transform_broadcaster.h"

#define USAGE "stageros <worldfile>"
#define ODOM "odom"
//#define BASE_SCAN "base_scan"
#define LASER_SCAN "laser_scan"
#define RANGER_SCAN "ranger_scan"
#define BASE_POSE_GROUND_TRUTH "base_pose_ground_truth"
#define CMD_VEL "cmd_vel"

// Our node
class StageNode
{
  private:
    // Messages that we'll send or receive
    //sensor_msgs::LaserScan *laserMsgs;
    //nav_msgs::Odometry *odomMsgs;
    //nav_msgs::Odometry *groundTruthMsgs;
    roslib::Clock clockMsg;

    // roscpp-related bookkeeping
    ros::NodeHandle n_;

    // A mutex to lock access to fields that are used in message callbacks
    boost::mutex msg_lock;

    // The models that we're interested in
    std::vector<Stg::ModelPosition *> positionmodels;
    std::vector<Stg::ModelLaser *> lasermodels;
	std::vector<Stg::ModelRanger *> rangermodels;
	
	struct StageRobot
	{
		Stg::ModelPosition * positionmodel;
		std::vector<Stg::ModelLaser *> lasermodels;
		std::vector<Stg::ModelRanger *> rangermodels;
		//ros::Publisher laser_pub_;
		//ros::Publisher range_pub_;
		std::vector<ros::Publisher> laser_pubs_;
		std::vector<ros::Publisher> range_pubs_;
		ros::Publisher odom_pub_;
		ros::Publisher ground_truth_pub_;
		ros::Subscriber cmdvel_sub_;
	};
    
    std::vector<struct StageRobot> robots;
    
    ros::Publisher clock_pub_;

    // A helper function that is executed for each stage model.  We use it
    // to search for models of interest.
    static void ghfunc(Stg::Model* mod, StageNode* node);

    static bool s_update(Stg::World* world, StageNode* node)
    {
      node->WorldCallback();
      // We return false to indicate that we want to be called again (an
      // odd convention, but that's the way that Stage works).
      return false;
    }

    // Appends the given robot ID to the given message name.  If omitRobotID
    // is true, an unaltered copy of the name is returned.
    const char *mapName(const char *name, size_t robotID);

    //  like mapName, but for lasers.
    // robotID: position model number
    // localLaserID: number of laser, respect to position model!
    const char *mapLaserName(const char *name, size_t localLaserID,size_t robotID);

    tf::TransformBroadcaster tf;

    // Last time that we received a velocity command
    ros::Time base_last_cmd;
    ros::Duration base_watchdog_timeout;

    // Current simulation time
    ros::Time sim_time;

  public:
    // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
    StageNode(int argc, char** argv, bool gui, const char* fname);
    ~StageNode();

    // Subscribe to models of interest.  Currently, we find and subscribe
    // to the first 'laser' model and the first 'position' model.  Returns
    // 0 on success (both models subscribed), -1 otherwise.
    int SubscribeModels();

    // Our callback
    void WorldCallback();
    
    // Do one update of the world.  May pause if the next update time
    // has not yet arrived.
    bool UpdateWorld();

    // Message callback for a MsgBaseVel message, which set velocities.
    void cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg);

    // The main simulator object
    Stg::World* world;
};

// since stageros is single-threaded, this is OK. revisit if that changes!
const char *
StageNode::mapName(const char *name, size_t robotID)
{
  if (positionmodels.size() > 1)
  {
    static char buf[100];
    snprintf(buf, sizeof(buf), "/robot_%u/%s", (unsigned int)robotID, name);
    return buf;
  }
  else
    return name;
}

// since stageros is single-threaded, this is OK. revisit if that changes!
const char *
StageNode::mapLaserName(const char *name, size_t localLaserID,size_t robotID)
{
	
	
	static char buf[100];
	//if (localLaserID > 0){
	   snprintf(buf, sizeof(buf), "%s_%u", name,(unsigned int)localLaserID );
	//} else {
	//   snprintf(buf, sizeof(buf), "%s", name);
	//}
		
    if (robotID > 0)
    {
      snprintf(buf, sizeof(buf), "/robot_%u/%s", (unsigned int)robotID, buf);    
    }
  
  //ROS_INFO("Getting name for laser #%d of robot #%d:");
  
    return buf;
}

void
StageNode::ghfunc(Stg::Model* mod, StageNode* node)
{
  if (dynamic_cast<Stg::ModelLaser *>(mod))
    node->lasermodels.push_back(dynamic_cast<Stg::ModelLaser *>(mod));
  if (dynamic_cast<Stg::ModelRanger *>(mod))
	node->rangermodels.push_back(dynamic_cast<Stg::ModelRanger *>(mod));
  if (dynamic_cast<Stg::ModelPosition *>(mod))
    node->positionmodels.push_back(dynamic_cast<Stg::ModelPosition *>(mod));
}

void
StageNode::cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg)
{
  boost::mutex::scoped_lock lock(msg_lock);
  this->positionmodels[idx]->SetSpeed(msg->linear.x, 
                                      msg->linear.y, 
                                      msg->angular.z);
  this->base_last_cmd = this->sim_time;
}

StageNode::StageNode(int argc, char** argv, bool gui, const char* fname)
{
  this->sim_time.fromSec(0.0);
  this->base_last_cmd.fromSec(0.0);
  double t;
  ros::NodeHandle localn("~");
  if(!localn.getParam("base_watchdog_timeout", t))
    t = 0.2;
  this->base_watchdog_timeout.fromSec(t);

  // We'll check the existence of the world file, because libstage doesn't
  // expose its failure to open it.  Could go further with checks (e.g., is
  // it readable by this user).
  struct stat s;
  if(stat(fname, &s) != 0)
  {
    ROS_FATAL("The world file %s does not exist.", fname);
    ROS_BREAK();
  }

  // initialize libstage
  Stg::Init( &argc, &argv );

  if(gui)
    this->world = new Stg::WorldGui(800, 700, "Stage (ROS)");
  else
    this->world = new Stg::World();

  // Apparently an Update is needed before the Load to avoid crashes on
  // startup on some systems.
  this->UpdateWorld();
  this->world->Load(fname);

  // We add our callback here, after the Update, so avoid our callback
  // being invoked before we're ready.
  this->world->AddUpdateCallback((Stg::stg_world_callback_t)s_update, this);

  this->world->ForEachDescendant((Stg::stg_model_callback_t)ghfunc, this);
  if (lasermodels.size() + rangermodels.size() < positionmodels.size())
  {
	ROS_FATAL("number of position models and laser/ranger models must be equal in "
				 "the world file.");
	ROS_BREAK();
  }
  size_t numRobots = positionmodels.size();

  ROS_INFO("found %u robot%s in the file", 
	(unsigned int)numRobots, (numRobots==1) ? "" : "s");
	
  //this->laserMsgs = new sensor_msgs::LaserScan[lasermodels.size()];
  //this->odomMsgs = new nav_msgs::Odometry[numRobots];
  //this->groundTruthMsgs = new nav_msgs::Odometry[numRobots];
}


// Subscribe to models of interest.  Currently, we find and subscribe
// to the first 'laser' model and the first 'position' model.  Returns
// 0 on success (both models subscribed), -1 otherwise.
//
// Eventually, we should provide a general way to map stage models onto ROS
// topics, similar to Player .cfg files.
int
StageNode::SubscribeModels()
{
  n_.setParam("/use_sim_time", true);
  //ROS_INFO("[new_stageros] we have  %d position models and %d laser models",this->positionmodels.size(),this->lasermodels.size() );

	for (size_t r = 0; r < this->positionmodels.size(); r++)
	{
		if(this->positionmodels[r])
		{
			this->positionmodels[r]->Subscribe();
			
			struct StageRobot new_robot;
			new_robot.positionmodel = positionmodels[r];
			new_robot.odom_pub_ = n_.advertise<nav_msgs::Odometry>(mapName(ODOM,r), 10);
			new_robot.ground_truth_pub_ = n_.advertise<nav_msgs::Odometry>(mapName(BASE_POSE_GROUND_TRUTH,r), 10);
			new_robot.cmdvel_sub_ = n_.subscribe<geometry_msgs::Twist>(mapName(CMD_VEL,r), 10, boost::bind(&StageNode::cmdvelReceived, this, r, _1));
			
			//odom_pubs_.push_back(n_.advertise<nav_msgs::Odometry>(mapName(ODOM,r), 10));
			//ground_truth_pubs_.push_back(n_.advertise<nav_msgs::Odometry>(mapName(BASE_POSE_GROUND_TRUTH,r), 10));
			//cmdvel_subs_.push_back(n_.subscribe<geometry_msgs::Twist>(mapName(CMD_VEL,r), 10, boost::bind(&StageNode::cmdvelReceived, this, r, _1)));
		
			
			for (size_t s = 0; s < this->lasermodels.size(); s++)
			{
				if(this->lasermodels[s])
				{
					this->lasermodels[s]->Subscribe();
					
					if(lasermodels[s]->Parent()==positionmodels[r])
					{
							//ROS_INFO("[new_stageros] el laser model %d pertenece al robot %d", s,r );						
						//if(new_robot.lasermodels.size() == 0){ 
						    //ROS_INFO("[new_stageros] adding laser publisher \"%s\" to robot %d",mapLaserName(LASER_SCAN,new_robot.lasermodels.size(),r),r);
							new_robot.laser_pubs_.push_back(n_.advertise<sensor_msgs::LaserScan>(mapLaserName(LASER_SCAN,new_robot.lasermodels.size(),r), 10));
							//new_robot.laser_pub_ = n_.advertise<sensor_msgs::LaserScan>(mapName(LASER_SCAN,r), 10);
						
						//laser_pubs_.push_back(n_.advertise<sensor_msgs::LaserScan>(mapName(LASER_SCAN,r), 10));
						//}
						new_robot.lasermodels.push_back(lasermodels[s]);
					} else{
						//ROS_INFO("[new_stageros] el laser model %d NO! pertenece al robot %d", s,r );						
					}
				}
				else
				{
					ROS_ERROR("no laser");
					return(-1);
				}
			}
			
			for (size_t s = 0; s < this->rangermodels.size(); s++)
			{
				if(this->rangermodels[s])
				{
					this->rangermodels[s]->Subscribe();
					if(rangermodels[s]->Parent()==positionmodels[r])
					{
						//if(new_robot.rangermodels.size() == 0) 
						  //ROS_INFO("[new_stageros] adding range publisher %s to robot %d",mapName(RANGER_SCAN,r),r);
						  new_robot.range_pubs_.push_back(n_.advertise<new_stage::Range>(mapName(RANGER_SCAN,r), 10));
						//new_robot.range_pub_ = n_.advertise<new_stage::Range>(mapName(RANGER_SCAN,r), 10);
					
						//range_pubs_.push_back(n_.advertise<new_stage::Range>(mapName(RANGER_SCAN,r), 10));
						
						new_robot.rangermodels.push_back(rangermodels[s]);
					}
				}
				else
				{
					ROS_ERROR("no ranger");
					return(-1);
				}
			}
			
			robots.push_back(new_robot);
		}
		else
		{
			ROS_ERROR("no position");
			return(-1);
		}
	}
	
	clock_pub_ = n_.advertise<roslib::Clock>("/clock",10);
	return(0);
}

StageNode::~StageNode()
{
  //delete[] laserMsgs;
  //delete[] odomMsgs;
  //delete[] groundTruthMsgs;
}

bool
StageNode::UpdateWorld()
{
  return this->world->UpdateAll();
}

void
StageNode::WorldCallback()
{
  boost::mutex::scoped_lock lock(msg_lock);

  this->sim_time.fromSec(world->SimTimeNow() / 1e6);
  // We're not allowed to publish clock==0, because it used as a special
  // value in parts of ROS, #4027.
  if(this->sim_time.sec == 0 && this->sim_time.nsec == 0)
  {
    ROS_DEBUG("Skipping initial simulation step, to avoid publishing clock==0");
    return;
  }

  // TODO make this only affect one robot if necessary
  if((this->base_watchdog_timeout.toSec() > 0.0) &&
      ((this->sim_time - this->base_last_cmd) >= this->base_watchdog_timeout))
  {
    for (size_t r = 0; r < this->positionmodels.size(); r++)
      this->positionmodels[r]->SetSpeed(0.0, 0.0, 0.0);
  }

  // Get latest laser data
  /*for (size_t r = 0; r < this->lasermodels.size(); r++)
  {
    std::vector<Stg::ModelLaser::Sample> samples = this->lasermodels[r]->GetSamples();
    if(samples.size())
    {
      // Translate into ROS message format and publish
      Stg::ModelLaser::Config cfg = this->lasermodels[r]->GetConfig();
      this->laserMsgs[r].angle_min = -cfg.fov/2.0;
      this->laserMsgs[r].angle_max = +cfg.fov/2.0;
      this->laserMsgs[r].angle_increment = cfg.fov/(double)(cfg.sample_count-1);
      this->laserMsgs[r].range_min = 0.0;
      this->laserMsgs[r].range_max = cfg.range_bounds.max;
      this->laserMsgs[r].ranges.resize(cfg.sample_count);
      this->laserMsgs[r].intensities.resize(cfg.sample_count);
      for(unsigned int i=0;i<cfg.sample_count;i++)
      {
        this->laserMsgs[r].ranges[i] = samples[i].range;
        this->laserMsgs[r].intensities[i] = (uint8_t)samples[i].reflectance;
      }

      this->laserMsgs[r].header.frame_id = mapName("base_laser_link", r);
      this->laserMsgs[r].header.stamp = sim_time;
      this->laser_pubs_[r].publish(this->laserMsgs[r]);
    }

    // Also publish the base->base_laser_link Tx.  This could eventually move
    // into being retrieved from the param server as a static Tx.
    Stg::Pose lp = this->lasermodels[r]->GetPose();
    tf::Quaternion laserQ;
    laserQ.setRPY(0.0, 0.0, lp.a);
    tf::Transform txLaser =  tf::Transform(laserQ,
                                            tf::Point(lp.x, lp.y, 0.15));
    tf.sendTransform(tf::StampedTransform(txLaser, sim_time,
                                          mapName("base_link", r),
                                          mapName("base_laser_link", r)));

	}

	// Get latest ranger data	
	for (size_t r = 0; r < this->rangermodels.size(); r++)
	{
		std::vector<Stg::ModelRanger::Sensor> sensors = this->rangermodels[r]->sensors;
		if(sensors.size())
		{
			// Translate into ROS message format and publish
			for(unsigned int i=0;i<sensors.size();i++)
			{
				new_stage::Range rangeMsg;
				rangeMsg.radiation_type = 0; // ULTRASOUND
				rangeMsg.field_of_view = sensors[i].fov;
				rangeMsg.min_range = sensors[i].bounds_range.min;
				rangeMsg.max_range = sensors[i].bounds_range.max;
				rangeMsg.range = sensors[i].range;

				char frame_name[128];
				sprintf(frame_name, "base_ranger%d_link", i);
				rangeMsg.header.frame_id = mapName(frame_name, r);
				rangeMsg.header.stamp = sim_time;

				// Also publish the base->base_laser_link Tx.  This could eventually move
				// into being retrieved from the param server as a static Tx.
				Stg::Pose lr = sensors[i].pose;
				tf::Quaternion rangerQ;
				rangerQ.setRPY(0.0, 0.0, lr.a);
				tf::Transform txRanger =  tf::Transform(rangerQ,
									tf::Point(lr.x, lr.y, 0.15));
				tf.sendTransform(tf::StampedTransform(txRanger, sim_time,
										mapName("base_link", r),
										mapName(frame_name, r)));
				
				this->range_pubs_[r].publish(rangeMsg);	
			}
		}
	}*/
	
	//ROS_DEBUG("[worldcallback] tenemos %d positionmodels",this->positionmodels.size());
	for (size_t r = 0; r < this->positionmodels.size(); r++)
	{
      //ROS_DEBUG("[worldcallback]  positionmodel %d tiene %d lasermodels",r,this->robots[r].lasermodels.size());
	  for (size_t s = 0; s < this->robots[r].lasermodels.size(); s++)
	  {
		std::vector<Stg::ModelLaser::Sample> samples = this->robots[r].lasermodels[s]->GetSamples();
		if(samples.size())
		{
		  sensor_msgs::LaserScan laserMsg;
		
		  // Translate into ROS message format and publish
		  Stg::ModelLaser::Config cfg = this->robots[r].lasermodels[s]->GetConfig();
		  laserMsg.angle_min = -cfg.fov/2.0;
		  laserMsg.angle_max = +cfg.fov/2.0;
		  laserMsg.angle_increment = cfg.fov/(double)(cfg.sample_count-1);
		  laserMsg.range_min = 0.0;
		  laserMsg.range_max = cfg.range_bounds.max;
		  laserMsg.ranges.resize(cfg.sample_count);
		  laserMsg.intensities.resize(cfg.sample_count);
		  for(unsigned int i=0;i<cfg.sample_count;i++)
		  {
			laserMsg.ranges[i] = samples[i].range;
			laserMsg.intensities[i] = (uint8_t)samples[i].reflectance;
		  }

		  std::string frame_id = mapName("base_laser_link", r);
		  if(this->robots[r].lasermodels.size() > 1)
		  {
		  	char ind[3];
		  	sprintf(ind, "_%d", s);
		  	frame_id.append(ind);
		  }
		  laserMsg.header.frame_id = frame_id;
		  laserMsg.header.stamp = sim_time;
		  //ROS_DEBUG("[new_stageros] robot %d publishing on laser %d",r,s);		  
		  this->robots[r].laser_pubs_[s].publish(laserMsg);
		  //this->robots[r].laser_pub_.publish(laserMsg);
		  
		  // Also publish the base->base_laser_link Tx.  This could eventually move
		  // into being retrieved from the param server as a static Tx.
		  Stg::Pose lp = this->robots[r].lasermodels[s]->GetPose();
		  tf::Quaternion laserQ;
		  laserQ.setRPY(0.0, 0.0, lp.a);
		  tf::Transform txLaser =  tf::Transform(laserQ,
										            tf::Point(lp.x, lp.y, 0.15));
		  tf.sendTransform(tf::StampedTransform(txLaser, sim_time,
										          mapName("base_link", r),
										          frame_id));

		  }
		}

		// Get latest ranger data	
		for (size_t s = 0; s < this->robots[r].rangermodels.size(); s++)
		{
			std::vector<Stg::ModelRanger::Sensor> sensors = this->robots[r].rangermodels[s]->sensors;
			if(sensors.size())
			{
				// Translate into ROS message format and publish
				for(unsigned int i=0;i<sensors.size();i++)
				{
					new_stage::Range rangeMsg;
					rangeMsg.radiation_type = 0; // ULTRASOUND
					rangeMsg.field_of_view = sensors[i].fov;
					rangeMsg.min_range = sensors[i].bounds_range.min;
					rangeMsg.max_range = sensors[i].bounds_range.max;
					rangeMsg.range = sensors[i].range;

					char frame_name[128];
					sprintf(frame_name, "base_ranger_link_%d", i);
					rangeMsg.header.frame_id = mapName(frame_name, r);
					rangeMsg.header.stamp = sim_time;

					// Also publish the base->base_laser_link Tx.  This could eventually move
					// into being retrieved from the param server as a static Tx.
					Stg::Pose lr = sensors[i].pose;
					tf::Quaternion rangerQ;
					rangerQ.setRPY(0.0, 0.0, lr.a);
					tf::Transform txRanger =  tf::Transform(rangerQ,
										tf::Point(lr.x, lr.y, 0.15));
					tf.sendTransform(tf::StampedTransform(txRanger, sim_time,
											mapName("base_link", r),
											mapName(frame_name, r)));
			
					//this->robots[r].range_pub_.publish(rangeMsg);	
					ROS_DEBUG("[new_stageros] robot %d publishing on range sensor %d",r,s);
					this->robots[r].range_pubs_[s].publish(rangeMsg);
				}
			}
		}

    // Send the identity transform between base_footprint and base_link
    tf::Transform txIdentity(tf::createIdentityQuaternion(),
                             tf::Point(0, 0, 0));
    tf.sendTransform(tf::StampedTransform(txIdentity,
                                          sim_time,
                                          mapName("base_footprint", r),
                                          mapName("base_link", r)));

	nav_msgs::Odometry odomMsg;
    // Get latest odometry data
    // Translate into ROS message format and publish
    odomMsg.pose.pose.position.x = this->positionmodels[r]->est_pose.x;
    odomMsg.pose.pose.position.y = this->positionmodels[r]->est_pose.y;
    odomMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->positionmodels[r]->est_pose.a);
    Stg::Velocity v = this->positionmodels[r]->GetVelocity();
    odomMsg.twist.twist.linear.x = v.x;
    odomMsg.twist.twist.linear.y = v.y;
    odomMsg.twist.twist.angular.z = v.a;

    //@todo Publish stall on a separate topic when one becomes available
    //this->odomMsgs[r].stall = this->positionmodels[r]->Stall();
    //
    odomMsg.header.frame_id = mapName("odom", r);
    odomMsg.header.stamp = sim_time;

    this->robots[r].odom_pub_.publish(odomMsg);

    // broadcast odometry transform
    tf::Quaternion odomQ;
    tf::quaternionMsgToTF(odomMsg.pose.pose.orientation, odomQ);
    tf::Transform txOdom(odomQ, 
                         tf::Point(odomMsg.pose.pose.position.x,
                                   odomMsg.pose.pose.position.y, 0.0));
    tf.sendTransform(tf::StampedTransform(txOdom, sim_time,
                                          mapName("odom", r),
                                          mapName("base_footprint", r)));

    // Also publish the ground truth pose and velocity
    Stg::Pose gpose = this->positionmodels[r]->GetGlobalPose();
    Stg::Velocity gvel = this->positionmodels[r]->GetGlobalVelocity();
    // Note that we correct for Stage's screwed-up coord system.
    tf::Quaternion q_gpose;
    q_gpose.setRPY(0.0, 0.0, gpose.a-M_PI/2.0);
    tf::Transform gt(q_gpose, tf::Point(gpose.y, -gpose.x, 0.0));
    tf::Quaternion q_gvel;
    q_gvel.setRPY(0.0, 0.0, gvel.a-M_PI/2.0);
    tf::Transform gv(q_gvel, tf::Point(gvel.y, -gvel.x, 0.0));

	nav_msgs::Odometry groundTruthMsg;
    groundTruthMsg.pose.pose.position.x     = gt.getOrigin().x();
    groundTruthMsg.pose.pose.position.y     = gt.getOrigin().y();
    groundTruthMsg.pose.pose.position.z     = gt.getOrigin().z();
    groundTruthMsg.pose.pose.orientation.x  = gt.getRotation().x();
    groundTruthMsg.pose.pose.orientation.y  = gt.getRotation().y();
    groundTruthMsg.pose.pose.orientation.z  = gt.getRotation().z();
    groundTruthMsg.pose.pose.orientation.w  = gt.getRotation().w();
    groundTruthMsg.twist.twist.linear.x = gv.getOrigin().x();
    groundTruthMsg.twist.twist.linear.y = gv.getOrigin().y();
    //this->groundTruthMsgs[r].twist.twist.angular.z = tf::getYaw(gv.getRotation());
    //this->groundTruthMsgs[r].twist.twist.linear.x = gvel.x;
    //this->groundTruthMsgs[r].twist.twist.linear.y = gvel.y;
    groundTruthMsg.twist.twist.angular.z = gvel.a;

    groundTruthMsg.header.frame_id = mapName("odom", r);
    groundTruthMsg.header.stamp = sim_time;

    this->robots[r].ground_truth_pub_.publish(groundTruthMsg);
  }

  this->clockMsg.clock = sim_time;
  this->clock_pub_.publish(this->clockMsg);
}

static bool quit = false;
void
sigint_handler(int num)
{
  quit = true;
}

int 
main(int argc, char** argv)
{ 
  if( argc < 2 )
  {
    puts(USAGE);
    exit(-1);
  }

  ros::init(argc, argv, "stageros");

  bool gui = true;
  for(int i=0;i<(argc-1);i++)
  {
    if(!strcmp(argv[i], "-g"))
      gui = false;
  }

  StageNode sn(argc-1,argv,gui,argv[argc-1]);

  if(sn.SubscribeModels() != 0)
    exit(-1);

  boost::thread t = boost::thread::thread(boost::bind(&ros::spin));

  // TODO: get rid of this fixed-duration sleep, using some Stage builtin
  // PauseUntilNextUpdate() functionality.
  ros::WallRate r(10.0);
  while(ros::ok() && !sn.world->TestQuit())
  {
    if(gui)
      Fl::wait(r.expectedCycleTime().toSec());
    else
    {
      sn.UpdateWorld();
      r.sleep();
    }
  }
  t.join();

  exit(0);
}

