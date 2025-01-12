#include "ColliderPosePublisher.hh"

#include <gz/msgs/pose_v.pb.h>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>

#include <iostream>

namespace lawn_robot_project
{
  ColliderPosePublisher::ColliderPosePublisher()
  {
  }

  void ColliderPosePublisher::Configure(
      const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm,
      gz::sim::EventManager &)
  {
    this->modelEntity = entity;

    if (sdf->HasElement("topic"))
    {
      this->topic = sdf->Get<std::string>("topic");
    }
    else
    {
      this->topic = "/collider_poses";
    }

    this->posePublisher = this->node.Advertise<gz::msgs::Pose_V>(this->topic);

    if (!this->posePublisher)
    {
      std::cerr << "Error advertising topic [" << this->topic << "]" << std::endl;
    }
  }

  void ColliderPosePublisher::PreUpdate(
      const gz::sim::UpdateInfo &,
      gz::sim::EntityComponentManager &ecm)
  {
    gz::msgs::Pose_V msg;

    ecm.Each<gz::sim::components::Collision, gz::sim::components::Pose>(
        [&](const gz::sim::Entity &entity,
            const gz::sim::components::Collision *,
            const gz::sim::components::Pose *poseComp) -> bool
        {
          auto pose = poseComp->Data();
          auto *newPose = msg.add_pose();
          newPose->set_name(std::to_string(entity));
          newPose->mutable_position()->set_x(pose.Pos().X());
          newPose->mutable_position()->set_y(pose.Pos().Y());
          newPose->mutable_position()->set_z(pose.Pos().Z());
          newPose->mutable_orientation()->set_x(pose.Rot().X());
          newPose->mutable_orientation()->set_y(pose.Rot().Y());
          newPose->mutable_orientation()->set_z(pose.Rot().Z());
          newPose->mutable_orientation()->set_w(pose.Rot().W());
          return true;
        });

    this->posePublisher.Publish(msg);
  }

  GZ_ADD_PLUGIN(ColliderPosePublisher,
                gz::sim::System,
                ColliderPosePublisher::ISystemConfigure,
                ColliderPosePublisher::ISystemPreUpdate)

  GZ_ADD_PLUGIN_ALIAS(ColliderPosePublisher, "lawn_robot_project::ColliderPosePublisher")
}
