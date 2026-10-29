// Gazebo joint commands for moving a part with a gripper and seating it in 1V4.
#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Gravity.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace group4_project2_al {
using namespace ignition::gazebo;

class GraspSystem final : public System,
                          public ISystemConfigure,
                          public ISystemPreUpdate {
 public:
  void Configure(const Entity &, const std::shared_ptr<const sdf::Element> &,
                 EntityComponentManager &, EventManager &) override {
    node_.Subscribe("/project2/grasp/request", &GraspSystem::OnRequest, this);
    reply_ = node_.Advertise<ignition::msgs::StringMsg>("/project2/grasp/reply");
  }

  void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override {
    std::deque<std::string> queue;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue.swap(requests_);
    }
    for (const auto &request : queue) {
      Process(request, ecm);
    }
    // Parked carriers ignore gravity; apply their weight after the release command.
    if (!info.paused) {
      const auto world = ecm.EntityByComponents(components::World());
      const auto gravity = ecm.Component<components::Gravity>(world);
      for (const auto &entry : dropped_) {
        const auto inertia = ecm.Component<components::Inertial>(entry.second);
        if (gravity && inertia) {
          ignition::gazebo::Link(entry.second).AddWorldForce(
              ecm, gravity->Data() * inertia->Data().MassMatrix().Mass());
        }
      }
    }
  }

 private:
  void OnRequest(const ignition::msgs::StringMsg &message) {
    std::lock_guard<std::mutex> lock(mutex_);
    requests_.push_back(message.data());
  }

  void Reply(const std::string &id, const std::string &result) {
    ignition::msgs::StringMsg message;
    message.set_data(id + " " + result);
    reply_.Publish(message);
  }

  Entity Link(EntityComponentManager &ecm, const std::string &modelName,
              const std::string &linkName) {
    const auto model = ecm.EntityByComponents(components::Model(),
                                               components::Name(modelName));
    if (model == kNullEntity) return kNullEntity;
    auto link = ecm.EntityByComponents(components::Link(),
                                  components::ParentEntity(model),
                                  components::Name(linkName));
    if (link == kNullEntity && linkName == "part") {
      link = ecm.EntityByComponents(components::Link(),
          components::ParentEntity(model), components::Name(modelName + "__part"));
    }
    return link;
  }

  void Process(const std::string &request, EntityComponentManager &ecm) {
    std::istringstream input(request);
    std::string id, op, child, parent;
    if (!(input >> id >> op >> child >> parent)) return;
    if (op == "clear") {
      for (const auto &entry : joints_) ecm.RequestRemoveEntity(entry.second);
      joints_.clear();
      parents_.clear();
      Reply(id, "ok cleared");
      return;
    }
    const Entity part = Link(ecm, child, "part");
    if (part == kNullEntity) {
      Reply(id, "error missing_part");
      return;
    }
    if (op == "pose") {
      const auto pose = worldPose(part, ecm);
      std::ostringstream data;
      data.precision(10);
      data << "ok pose " << pose.Pos().X() << ' ' << pose.Pos().Y() << ' '
           << pose.Pos().Z() << ' ' << pose.Rot().X() << ' '
           << pose.Rot().Y() << ' ' << pose.Rot().Z() << ' '
           << pose.Rot().W();
      Reply(id, data.str());
      return;
    }
    if (op == "freeze" || op == "drop") {
      // Empty carriers fall after release; Reset stops them before restoring stock.
      if (child.find("kit_tray_") != 0) {
        Reply(id, "error not_a_carrier");
        return;
      }
      if (op == "drop") dropped_[child] = part;
      if (op == "freeze") {
        dropped_.erase(child);
        ecm.SetComponentData<components::LinearVelocityCmd>(part, ignition::math::Vector3d::Zero);
        ecm.SetComponentData<components::AngularVelocityCmd>(part, ignition::math::Vector3d::Zero);
        Reply(id, "ok frozen");
        return;
      }
    }
    if (op == "release" || op == "drop") {
      const auto held = joints_.find(child);
      if (held == joints_.end()) {
        Reply(id, "ok released");
        return;
      }
      ecm.RequestRemoveEntity(held->second);
      joints_.erase(held);
      parents_.erase(child);
      Reply(id, "ok released");
      return;
    }
    if (op == "deposit" || op == "park") {
      const auto held = joints_.find(child);
      const Entity belt = Link(ecm, parent, "cad_rig");
      double x, y, z, roll, pitch, yaw;
      if ((op == "deposit" && held == joints_.end()) || belt == kNullEntity ||
          !(input >> x >> y >> z >> roll >> pitch >> yaw)) {
        Reply(id, "error missing_deposit_target");
        return;
      }
      if (worldPose(part, ecm).Pos().Distance({x, y, z}) > 0.001) {
        Reply(id, "error deposit_not_aligned");
        return;
      }
      if (held != joints_.end()) {
        ecm.RequestRemoveEntity(held->second);
        joints_.erase(held);
      }
      Attach(ecm, child, belt, part);
      parents_[child] = parent;
      Reply(id, "ok deposited");
      return;
    }
    if (op == "grasp") {
      if (joints_.count(child) && parents_[child] != "project2_rig") {
        if (parents_[child] == parent) {
          Reply(id, "ok grasped");
          return;
        }
        Reply(id, "error already_attached");
        return;
      }
      const Entity grip = Link(ecm, parent, "gripper_link");
      const Entity model = ecm.EntityByComponents(components::Model(),
                                                  components::Name(parent));
      const Entity joint = model == kNullEntity ? kNullEntity :
          Model(model).JointByName(ecm, "gripper");
      if (grip == kNullEntity) {
        Reply(id, "error missing_gripper_link");
        return;
      }
      if (joint == kNullEntity) {
        Reply(id, "error missing_gripper_joint");
        return;
      }
      const auto q = ecm.Component<components::JointPosition>(joint);
      if (!q) {
        ecm.CreateComponent(joint, components::JointPosition());
        Reply(id, "error joint_feedback_pending");
        return;
      }
      if (q->Data().empty()) {
        Reply(id, "error joint_feedback_pending");
        return;
      }
      const auto pose = worldPose(grip, ecm);
      const Entity jaw = Link(ecm, parent, "moving_jaw_link");
      if (jaw == kNullEntity) {
        Reply(id, "error missing_jaw_link");
        return;
      }
      // The controller checks both pad contacts against the STL before grasping.
      // Use the centre between the fingers for this additional envelope check.
      const auto fixedPad = pose.Pos() + pose.Rot().RotateVector(
          ignition::math::Vector3d(0.0079, -0.100, 0));
      const auto jawPose = worldPose(jaw, ecm);
      const auto movingPad = jawPose.Pos() + jawPose.Rot().RotateVector(
          ignition::math::Vector3d(-0.0123, -0.076, 0.0001));
      const auto tip = (fixedPad + movingPad) * 0.5;
      const auto objectPose = worldPose(part, ecm);
      const auto local = objectPose.Rot().RotateVectorReverse(tip-objectPose.Pos());
      if (q->Data()[0] > 0.95) {
        Reply(id, "error gripper_open");
        return;
      }
      ignition::math::Vector3d half(0.0255, 0.0483, 0.012);
      if (child.find("base_1v4") != std::string::npos) half.Set(0.038444, 0.014, 0.009);
      if (child.find("insert_3v4") != std::string::npos) half.Set(0.025124, 0.0125, 0.0069);
      if (child.find("pin_2v4") != std::string::npos) half.Set(0.014100, 0.014042, 0.005);
      if (child.find("kit_tray_") == 0) half.Set(0.15, 0.06, 0.02);
      if (child.find("lid_") == 0) half.Set(0.02745, 0.05025, 0.012);
      if (std::abs(local.X()) > half.X()+0.003 ||
          std::abs(local.Y()) > half.Y()+0.003 ||
          std::abs(local.Z()) > half.Z()+0.003) {
        Reply(id, "error out_of_reach");
        return;
      }
      if (joints_.count(child)) {
        ecm.RequestRemoveEntity(joints_[child]);
        joints_.erase(child);
      }
      Attach(ecm, child, grip, part);
      parents_[child] = parent;
      Reply(id, "ok grasped");
      return;
    }
    if (op == "seat" || op == "fasten") {
      if (joints_.count(child) && parents_[child] == parent) {
        Reply(id, "ok seated");
        return;
      }
      const Entity base = Link(ecm, parent, "part");
      const auto held = joints_.find(child);
      if (base == kNullEntity || (op == "seat" && held == joints_.end())) {
        Reply(id, "error missing_base_or_grasp");
        return;
      }
      double x, y, z, roll, pitch, yaw;
      if (!(input >> x >> y >> z >> roll >> pitch >> yaw)) {
        Reply(id, "error missing_target");
        return;
      }
      const auto relative = worldPose(base, ecm).Inverse() * worldPose(part, ecm);
      const ignition::math::Pose3d target(x, y, z, roll, pitch, yaw);
      const auto rotation = target.Rot().Inverse() * relative.Rot();
      const double angle = 2.0 * std::acos(std::min(1.0, std::abs(rotation.W())));
      if (relative.Pos().Distance(target.Pos()) + 0.06 * angle > 0.001) {
        Reply(id, "error insertion_not_aligned");
        return;
      }
      if (held != joints_.end()) {
        ecm.RequestRemoveEntity(held->second);
        joints_.erase(held);
      }
      Attach(ecm, child, base, part);
      parents_[child] = parent;
      Reply(id, "ok seated");
      return;
    }
    Reply(id, "error unknown_operation");
  }

  void Attach(EntityComponentManager &ecm, const std::string &child,
              Entity parentLink, Entity childLink) {
    const Entity joint = ecm.CreateEntity();
    ecm.CreateComponent(joint, components::Name("grasp_" + child + "_" + std::to_string(joint)));
    ecm.CreateComponent(joint, components::DetachableJoint(
        {parentLink, childLink, "fixed"}));
    joints_[child] = joint;
  }

  ignition::transport::Node node_;
  ignition::transport::Node::Publisher reply_;
  std::mutex mutex_;
  std::deque<std::string> requests_;
  std::unordered_map<std::string, Entity> joints_;
  std::unordered_map<std::string, Entity> dropped_;
  std::unordered_map<std::string, std::string> parents_;
};
}  // namespace group4_project2_al

IGNITION_ADD_PLUGIN(group4_project2_al::GraspSystem,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)
