#ifndef COLLIDER_POSE_PUBLISHER_HH_
#define COLLIDER_POSE_PUBLISHER_HH_

#include <gz/sensors/Sensor.hh>
#include <gz/sensors/SensorTypes.hh>
#include <gz/transport/Node.hh>

namespace lawn_robot_project
{
  class ColliderPosePublisher : public gz::sim::System,
                                public gz::sim::ISystemConfigure,
                                public gz::sim::ISystemPreUpdate
  {
  public:
    ColliderPosePublisher();
    ~ColliderPosePublisher() override = default;

    void Configure(const gz::sim::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   gz::sim::EntityComponentManager &ecm,
                   gz::sim::EventManager &eventMgr) override;

    void PreUpdate(const gz::sim::UpdateInfo &info,
                   gz::sim::EntityComponentManager &ecm) override;

  private:
    /// \brief Transport node for communication
    gz::transport::Node node;

    /// \brief Publisher for collider poses
    gz::transport::Node::Publisher posePublisher;

    /// \brief Topic for collider poses
    std::string topic;

    /// \brief Root entity in the simulation
    gz::sim::Entity modelEntity;
  };
}

#endif
