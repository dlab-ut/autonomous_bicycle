//
// Based on gazebo_ros_diff_drive
//
#include <autonomous_bicycle/bicycle_interaction.h>

namespace gazebo {
    BicycleInteraction::BicycleInteraction() {}
    BicycleInteraction::~BicycleInteraction() {}

    // Load the controller
    void BicycleInteraction::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        this->parent = _parent;

        gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "BicycleInteraction"));

        // Make sure the ROS node for Gazebo has already been initialized
        gazebo_ros_->isInitialized();

        gazebo_ros_->getParameter<std::string>(topic_wheel, "topic_wheel", "wheel_torque");
        gazebo_ros_->getParameter<std::string>(topic_steering, "topic_steering", "steer_vel");
        gazebo_ros_->getParameter<double>(update_rate_, "updateRate", 100.0);
        gazebo_ros_->getParameter<double>(limit_torque, "limitTorque", 5.0);

        wheel_torque = -1;
        steer_velocity = 0;

        joint_front_wheel = gazebo_ros_->getJoint(parent, "joint_front_wheel", "_joint_front_wheel");
        joint_rear_wheel = gazebo_ros_->getJoint(parent, "joint_rear_wheel", "_joint_rear_wheel");
        joint_steering = gazebo_ros_->getJoint(parent, "joint_steering", "_joint_steering");

        joint_front_wheel->SetParam("fmax", 0, limit_torque);
        joint_rear_wheel->SetParam("fmax", 0, limit_torque);
        joint_steering->SetParam("fmax", 0, limit_torque);

        if (!joint_front_wheel) {
            ROS_ERROR("Not found: (joint_wheel)");
            return;
        }

        if (!joint_rear_wheel) {
            ROS_ERROR("Not found: (joint_wheel_90)");
            return;
        }

        if (!joint_steering) {
            ROS_ERROR("Not found: (joint_wheel_90)");
            return;
        }

        j2_controller = new physics::JointController(this->parent);

        // Initialize update rate stuff
        if (this->update_rate_ > 0.0) this->update_period_ = 1.0 / this->update_rate_;
        else this->update_period_ = 0.0;
        last_update_time_ = parent->GetWorld()->SimTime();

        restart_position = 0;

        alive_ = true;

        ros::SubscribeOptions so =
                ros::SubscribeOptions::create<std_msgs::Float32>(topic_wheel, 1,
                                                                    boost::bind(&BicycleInteraction::wheelForceCallback,
                                                                                this, _1),
                                                                    ros::VoidPtr(), &queue_);
        wheel_force_subscriber_ = gazebo_ros_->node()->subscribe(so);
        ros::SubscribeOptions so_ =
                ros::SubscribeOptions::create<std_msgs::Float32>(topic_steering, 1,
                                                                    boost::bind(&BicycleInteraction::steerVelCallback,
                                                                                this, _1),
                                                                    ros::VoidPtr(), &queue_);
        steer_vel_subscriber_ = gazebo_ros_->node()->subscribe(so_);

        // start custom queue for bicycle interaction
        this->callback_queue_thread_ =
                boost::thread(boost::bind(&BicycleInteraction::QueueThread, this));

        // listen to the update event (broadcast every simulation iteration)
        this->update_connection_ =
                event::Events::ConnectWorldUpdateBegin(boost::bind(&BicycleInteraction::UpdateChild, this));

        this->create_connection_ =
                event::Events::ConnectWorldReset(boost::bind(&BicycleInteraction::resetWorldEvent, this));

    }

    void BicycleInteraction::Reset()
    {
        last_update_time_ = parent->GetWorld()->SimTime();
    }

    void BicycleInteraction::UpdateChild()
    {
        common::Time current_time = parent->GetWorld()->SimTime();
        double seconds_since_last_update = (current_time - last_update_time_).Double();

        if(!initial_pose_saved){
            this->original_pose = this->parent->WorldPose();
            initial_pose_saved = true;
        }

        if (seconds_since_last_update > update_period_) {

            // if velocity < 0 reset model pose
            if(!init_simulation){
                if(wheel_torque < 0){
                    init_simulation = true;
                    joint_front_wheel->SetVelocity(0, 0);
                    joint_rear_wheel->SetVelocity(0, 0);
                    joint_steering->SetVelocity(0, 0);

                    joint_front_wheel->SetPosition(0, 0);
                    joint_rear_wheel->SetPosition(0, 0);
                    joint_steering->SetPosition(0, 0);

                    //this->original_pose.rot.y = 0.5;
                    this->parent->SetWorldPose(this->original_pose);
                }
            }else{
                joint_rear_wheel->SetForce(0, -wheel_torque);
                //joint_rear_wheel->SetVelocity(0, -30);
                joint_steering->SetVelocity(0, steer_velocity);
            }

            last_update_time_ += common::Time(update_period_);
        }
    }

    void BicycleInteraction::resetWorldEvent()
    {
        ROS_INFO("BicycleInteraction::resetWorldEvent()");
    }

    // Finalize the controller
    void BicycleInteraction::FiniChild()
    {
        alive_ = false;
        queue_.clear();
        queue_.disable();
        gazebo_ros_->node()->shutdown();
        callback_queue_thread_.join();
    }

    void BicycleInteraction::wheelForceCallback(const std_msgs::Float32::ConstPtr &wheel_msg)
    {
        boost::mutex::scoped_lock scoped_lock(lock);
        wheel_torque = wheel_msg->data;
    }

    void BicycleInteraction::steerVelCallback(const std_msgs::Float32::ConstPtr &steer_msg)
    {
        boost::mutex::scoped_lock scoped_lock(lock);
        steer_velocity = steer_msg->data;
    }

    void BicycleInteraction::QueueThread()
    {
        static const double timeout = 0.01;

        while (alive_ && gazebo_ros_->node()->ok()) {
            queue_.callAvailable(ros::WallDuration(timeout));
        }

        this->Reset();
        this->FiniChild();
    }

    GZ_REGISTER_MODEL_PLUGIN (BicycleInteraction)
}