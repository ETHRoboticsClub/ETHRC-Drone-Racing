#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/double_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/wrench.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>

#include "sitl_gazebo_plugins/boreas_dynamics.hpp"

namespace gz::sim::systems
{
namespace
{
constexpr size_t kMaxMotors = 255;
constexpr const char *kAnsiReset = "\033[0m";
constexpr const char *kAnsiBoldRed = "\033[1;31m";
constexpr const char *kAnsiBoldYellow = "\033[1;33m";
constexpr const char *kAnsiBoldCyan = "\033[1;36m";
constexpr const char *kAnsiBoldMagenta = "\033[1;35m";

std::string Colorize(const char *_color, const std::string &_message)
{
  return std::string(_color) + _message + kAnsiReset;
}

std::string ErrorText(const std::string &_message)
{
  return Colorize(kAnsiBoldRed, _message);
}

std::string WarnText(const std::string &_message)
{
  return Colorize(kAnsiBoldYellow, _message);
}

std::string InfoText(const std::string &_message)
{
  return Colorize(kAnsiBoldCyan, _message);
}

std::string ResetText(const std::string &_message)
{
  return Colorize(kAnsiBoldMagenta, _message);
}

bool IsFinite(const math::Vector3d &_value)
{
  return std::isfinite(_value.X()) && std::isfinite(_value.Y()) &&
         std::isfinite(_value.Z());
}

bool IsFinite(const math::Quaterniond &_value)
{
  return std::isfinite(_value.W()) && std::isfinite(_value.X()) &&
         std::isfinite(_value.Y()) && std::isfinite(_value.Z());
}

bool IsFinite(const math::Pose3d &_value)
{
  return IsFinite(_value.Pos()) && IsFinite(_value.Rot());
}

std::string FormatVector(const math::Vector3d &_value)
{
  std::ostringstream stream;
  stream << "(" << _value.X() << ", " << _value.Y() << ", " << _value.Z() << ")";
  return stream.str();
}

struct ServoPacket
{
  float motorSpeed[kMaxMotors];
};

struct FdmPacket
{
  double timestamp;
  double imuAngularVelocityRPY[3];
  double imuLinearAccelerationXYZ[3];
  double imuOrientationQuat[4];
  double velocityXYZ[3];
  double positionXYZ[3];
  double escTemperature[4];
  double escVoltage[4];
  double escCurrent[4];
  double escConsumption[4];
  double escRpm[4];
};

template <typename T>
T SdfValue(const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string &_name, const T &_defaultValue)
{
  if (_sdf->HasElement(_name))
    return _sdf->Get<T>(_name);
  return _defaultValue;
}

void MakeSockAddr(const std::string &_address, const uint16_t _port,
    sockaddr_in &_sockaddr)
{
  std::memset(&_sockaddr, 0, sizeof(_sockaddr));
  _sockaddr.sin_family = AF_INET;
  _sockaddr.sin_port = htons(_port);
  _sockaddr.sin_addr.s_addr = inet_addr(_address.c_str());
}

std::mutex gMotorSocketMutex;
bool gMotorSocketBound = false;
}  // namespace

class EthrcBetaflightSITLPlugin:
  public System,
  public ISystemConfigure,
  public ISystemPreUpdate,
  public ISystemPostUpdate
{
  public: ~EthrcBetaflightSITLPlugin() override;

  public: void Configure(const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager &) override;

  public: void PreUpdate(const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

  public: void PostUpdate(const UpdateInfo &_info,
      const EntityComponentManager &_ecm) override;

  private: struct Rotor
  {
    int id{0};
    Entity jointEntity{kNullEntity};
    std::string jointName;
    math::Vector3d position{0, 0, 0};
    double turningDirection{1.0};
    double normalizedCommand{0.0};
    ethrc::sitl::RotorDynamics dynamics;
  };

  private: bool LoadConfiguration(const std::filesystem::path &_path);
  private: bool Bind();
  private: bool ReceiveMotorCommand();
  private: bool StateIsSane(EntityComponentManager &_ecm);
  private: void ResetVehicleState(EntityComponentManager &_ecm,
      const std::string &_reason);
  private: void ApplyForces(EntityComponentManager &_ecm, double _dtSeconds);
  private: void UpdateRotorVisuals(EntityComponentManager &_ecm);
  private: void PublishDiagnostics();
  private: void SendState(const UpdateInfo &_info,
      const EntityComponentManager &_ecm) const;

  private: Model model{kNullEntity};
  private: Entity modelEntity{kNullEntity};
  private: Link baseLink{kNullEntity};
  private: Entity imuEntity{kNullEntity};
  private: std::vector<Rotor> rotors;
  private: transport::Node transportNode;
  private: transport::Node::Publisher motorCommandPublisher;
  private: transport::Node::Publisher motorRpmPublisher;
  private: transport::Node::Publisher motorThrustPublisher;
  private: transport::Node::Publisher appliedWrenchPublisher;
  private: transport::Node::Publisher modelProfilePublisher;

  private: int socketFd{-1};
  private: std::string bindAddress{"127.0.0.1"};
  private: std::string betaflightAddress{"127.0.0.1"};
  private: uint16_t motorInputPort{9002};
  private: uint16_t stateOutputPort{9003};
  private: bool betaflightOnline{false};
  private: int missedPackets{0};
  private: int connectionTimeoutMaxCount{50};
  private: bool ownsMotorSocket{false};

  private: ethrc::sitl::PropulsionConfig propulsionConfig{
      {0.0, 0.0, 9.22805765}, 0.00125, 30000.0, 0.0, 0.0, 0.0, 1.0};
  private: std::array<double, 3> translationalDrag{0.0, 0.0, 0.0};
  private: std::array<double, 3> angularDrag{0.0, 0.0, 0.0};
  private: std::string airframeProfile{"legacy_sdf"};
  private: std::string calibrationStatus{"provisional"};
  private: double visualMaxRpm{3600.0};
  private: double launchReleaseCommand{0.45};
  private: double maxWorldPosition{30.0};
  private: double minWorldAltitude{-0.5};
  private: double maxWorldAltitude{20.0};
  private: double maxLinearSpeed{35.0};
  private: double maxAngularSpeed{80.0};
  private: math::Pose3d resetPose{0, 0, 0.5, 0, 0, 0};
  private: std::chrono::steady_clock::duration lastUpdateTime{0};
  private: bool stateGuardLatched{false};
  private: bool launchReleased{false};
  private: bool rotorVisualsSpinning{false};
  private: mutable bool previousOrientationValid{false};
  private: mutable math::Quaterniond previousOrientation{1, 0, 0, 0};
  private: math::Vector3d lastAppliedForceWorld{0, 0, 0};
  private: math::Vector3d lastAppliedTorqueWorld{0, 0, 0};
  private: mutable std::mutex mutex;
};

EthrcBetaflightSITLPlugin::~EthrcBetaflightSITLPlugin()
{
  if (this->socketFd >= 0)
  {
    close(this->socketFd);
    this->socketFd = -1;
  }

  if (this->ownsMotorSocket)
  {
    std::lock_guard<std::mutex> lock(gMotorSocketMutex);
    gMotorSocketBound = false;
    this->ownsMotorSocket = false;
  }
}

void EthrcBetaflightSITLPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{
  this->model = Model(_entity);
  this->modelEntity = _entity;

  if (!this->model.Valid(_ecm))
  {
    gzerr << ErrorText("[EthrcBetaflightSITLPlugin] Plugin must be attached to a model.") << "\n";
    return;
  }

  this->baseLink = Link(this->model.CanonicalLink(_ecm));
  if (!this->baseLink.Valid(_ecm))
  {
    gzerr << ErrorText("[EthrcBetaflightSITLPlugin] Could not find canonical link.") << "\n";
    return;
  }

  _ecm.CreateComponent(this->baseLink.Entity(),
      components::WorldLinearVelocity());
  _ecm.CreateComponent(this->baseLink.Entity(),
      components::WorldAngularVelocity());

  this->bindAddress = SdfValue<std::string>(_sdf, "bindAddress", this->bindAddress);
  this->betaflightAddress =
      SdfValue<std::string>(_sdf, "betaflightAddress", this->betaflightAddress);
  this->motorInputPort =
      static_cast<uint16_t>(SdfValue<int>(_sdf, "motorInputPort", this->motorInputPort));
  this->stateOutputPort =
      static_cast<uint16_t>(SdfValue<int>(_sdf, "stateOutputPort", this->stateOutputPort));
  this->connectionTimeoutMaxCount =
      SdfValue<int>(_sdf, "connectionTimeoutMaxCount", this->connectionTimeoutMaxCount);
  const double legacyMaxTotalThrust =
      SdfValue<double>(_sdf, "maxTotalThrust", 36.9122306);
  this->propulsionConfig.thrustCurveN =
      {0.0, 0.0, legacyMaxTotalThrust / 4.0};
  this->propulsionConfig.reactionMomentRatioM =
      SdfValue<double>(_sdf, "yawTorqueCoefficient", 0.00125);
  this->propulsionConfig.maximumRpm =
      SdfValue<double>(_sdf, "maxRpm", 30000.0);
  this->visualMaxRpm =
      SdfValue<double>(_sdf, "visualMaxRpm", this->visualMaxRpm);
  this->launchReleaseCommand =
      SdfValue<double>(_sdf, "launchReleaseCommand", this->launchReleaseCommand);
  this->maxWorldPosition =
      SdfValue<double>(_sdf, "maxWorldPosition", this->maxWorldPosition);
  this->minWorldAltitude =
      SdfValue<double>(_sdf, "minWorldAltitude", this->minWorldAltitude);
  this->maxWorldAltitude =
      SdfValue<double>(_sdf, "maxWorldAltitude", this->maxWorldAltitude);
  this->maxLinearSpeed =
      SdfValue<double>(_sdf, "maxLinearSpeed", this->maxLinearSpeed);
  this->maxAngularSpeed =
      SdfValue<double>(_sdf, "maxAngularSpeed", this->maxAngularSpeed);
  this->resetPose =
      SdfValue<math::Pose3d>(_sdf, "resetPose", this->resetPose);

  const std::string configurationFile =
      SdfValue<std::string>(_sdf, "configurationFile", "");
  if (!configurationFile.empty())
  {
    std::filesystem::path configPath(configurationFile);
    if (configPath.is_relative())
    {
      const char *configuredPath = std::getenv("ETHRC_BOREAS_CONFIG");
      if (configuredPath && std::filesystem::is_regular_file(configuredPath))
      {
        configPath = configuredPath;
      }
      else if (!_sdf->FilePath().empty())
      {
        configPath = std::filesystem::path(_sdf->FilePath()).parent_path() /
            configPath;
      }
      else
      {
        configPath = std::filesystem::current_path() / configPath;
      }
    }
    if (!std::filesystem::is_regular_file(configPath))
    {
      for (auto directory = std::filesystem::current_path(); !directory.empty();
          directory = directory.parent_path())
      {
        const auto candidate =
            directory / "config" / "airframes" / "boreas_4s.yaml";
        if (std::filesystem::is_regular_file(candidate))
        {
          configPath = candidate;
          break;
        }
        if (directory == directory.root_path())
          break;
      }
    }
    if (!this->LoadConfiguration(configPath.lexically_normal()))
      return;
  }

  const std::string imuName = SdfValue<std::string>(_sdf, "imuName", "imu_sensor");
  std::string sensorName = imuName;
  const size_t lastSep = imuName.find_last_of("::");
  if (lastSep != std::string::npos)
    sensorName = imuName.substr(lastSep + 1);

  _ecm.Each<components::Imu, components::Name>(
    [&](const Entity &_imuEntity, const components::Imu *,
        const components::Name *_name) -> bool
    {
      if (_name->Data().find(sensorName) != std::string::npos)
      {
        this->imuEntity = _imuEntity;
        return false;
      }
      return true;
    });

  if (this->imuEntity == kNullEntity)
  {
    gzerr << ErrorText("[EthrcBetaflightSITLPlugin] IMU [" + imuName + "] not found.") << "\n";
    return;
  }

  if (this->rotors.empty() && _sdf->HasElement("rotor"))
  {
    auto sdfNonConst = std::const_pointer_cast<sdf::Element>(_sdf);
    sdf::ElementPtr rotorSDF = sdfNonConst->GetElement("rotor");
    while (rotorSDF)
    {
      Rotor rotor;
      rotor.id = rotorSDF->Get<int>("id", static_cast<int>(this->rotors.size())).first;
      rotor.jointName = rotorSDF->Get<std::string>("jointName", "").first;
      rotor.position = rotorSDF->Get<math::Vector3d>("position", {0, 0, 0}).first;

      const std::string direction =
          rotorSDF->Get<std::string>("turningDirection", "ccw").first;
      rotor.turningDirection = direction == "cw" ? -1.0 : 1.0;
      rotor.dynamics = ethrc::sitl::RotorDynamics(this->propulsionConfig);

      this->rotors.push_back(rotor);
      rotorSDF = rotorSDF->GetNextElement("rotor");
    }
  }

  if (this->rotors.empty())
  {
    gzerr << ErrorText("[EthrcBetaflightSITLPlugin] No rotors configured.") << "\n";
    return;
  }

  std::sort(this->rotors.begin(), this->rotors.end(),
    [](const Rotor &_a, const Rotor &_b)
    {
      return _a.id < _b.id;
    });
  for (auto &rotor : this->rotors)
  {
    rotor.jointEntity = this->model.JointByName(_ecm, rotor.jointName);
    if (rotor.jointEntity != kNullEntity)
    {
      _ecm.CreateComponent(rotor.jointEntity, components::JointVelocity());
      _ecm.CreateComponent(rotor.jointEntity, components::JointVelocityCmd({0.0}));
    }
    else if (!rotor.jointName.empty())
    {
      gzwarn << WarnText("[EthrcBetaflightSITLPlugin] Joint [" +
                rotor.jointName +
                "] not found; thrust still applies but rotor visual won't spin.") << "\n";
    }
  }

  if (!this->Bind())
    return;

  this->motorCommandPublisher = this->transportNode.Advertise<msgs::Double_V>(
      "/drone/sim/motor_command");
  this->motorRpmPublisher = this->transportNode.Advertise<msgs::Double_V>(
      "/drone/sim/motor_rpm");
  this->motorThrustPublisher = this->transportNode.Advertise<msgs::Double_V>(
      "/drone/sim/motor_thrust");
  this->appliedWrenchPublisher = this->transportNode.Advertise<msgs::Wrench>(
      "/drone/sim/applied_wrench");
  this->modelProfilePublisher = this->transportNode.Advertise<msgs::StringMsg>(
      "/drone/sim/model_profile");
  msgs::StringMsg profileMessage;
  profileMessage.set_data(this->airframeProfile + ":" + this->calibrationStatus);
  this->modelProfilePublisher.Publish(profileMessage);

  gzlog << InfoText("[EthrcBetaflightSITLPlugin] Listening for Betaflight motors on " +
           this->bindAddress + ":" + std::to_string(this->motorInputPort) +
           ", sending state to " + this->betaflightAddress + ":" +
           std::to_string(this->stateOutputPort) + ", profile=" +
           this->airframeProfile + ", calibration=" + this->calibrationStatus +
           ", maxTotalThrust=" +
           std::to_string(4.0 * (
               this->propulsionConfig.thrustCurveN[0] +
               this->propulsionConfig.thrustCurveN[1] +
               this->propulsionConfig.thrustCurveN[2])) + " N.") << "\n";
}

bool EthrcBetaflightSITLPlugin::LoadConfiguration(
    const std::filesystem::path &_path)
{
  try
  {
    const YAML::Node root = YAML::LoadFile(_path.string());
    this->airframeProfile = root["airframe"].as<std::string>();
    this->calibrationStatus = root["calibration_status"].as<std::string>();

    const YAML::Node propulsion = root["propulsion"];
    const auto thrustCurve = propulsion["thrust_curve_n"].as<std::vector<double>>();
    if (thrustCurve.size() != 3u)
      throw std::runtime_error("propulsion.thrust_curve_n must have three coefficients");
    std::copy(thrustCurve.begin(), thrustCurve.end(),
        this->propulsionConfig.thrustCurveN.begin());
    this->propulsionConfig.reactionMomentRatioM =
        propulsion["reaction_moment_ratio_m"].as<double>();
    this->propulsionConfig.maximumRpm = propulsion["maximum_rpm"].as<double>();
    this->propulsionConfig.timeConstantUpS =
        propulsion["motor_time_constant_up_s"].as<double>();
    this->propulsionConfig.timeConstantDownS =
        propulsion["motor_time_constant_down_s"].as<double>();
    this->propulsionConfig.inputMin = propulsion["input_min"].as<double>();
    this->propulsionConfig.inputMax = propulsion["input_max"].as<double>();

    const YAML::Node aerodynamics = root["aerodynamics"];
    const auto translational =
        aerodynamics["translational_drag"].as<std::vector<double>>();
    const auto angular = aerodynamics["angular_drag"].as<std::vector<double>>();
    if (translational.size() != 3u || angular.size() != 3u)
      throw std::runtime_error("aerodynamic drag vectors must have three coefficients");
    std::copy(translational.begin(), translational.end(), this->translationalDrag.begin());
    std::copy(angular.begin(), angular.end(), this->angularDrag.begin());

    for (const YAML::Node &rotorNode : root["rotors"])
    {
      Rotor rotor;
      rotor.id = rotorNode["id"].as<int>();
      rotor.jointName = rotorNode["joint_name"].as<std::string>();
      const auto position = rotorNode["position_m"].as<std::vector<double>>();
      if (position.size() != 3u)
        throw std::runtime_error("rotor position_m must have three values");
      rotor.position = {position[0], position[1], position[2]};
      const std::string direction =
          rotorNode["turning_direction"].as<std::string>();
      if (direction != "cw" && direction != "ccw")
        throw std::runtime_error("rotor turning_direction must be cw or ccw");
      rotor.turningDirection = direction == "cw" ? -1.0 : 1.0;
      rotor.dynamics = ethrc::sitl::RotorDynamics(this->propulsionConfig);
      this->rotors.push_back(rotor);
    }
  }
  catch (const std::exception &error)
  {
    gzerr << ErrorText("[EthrcBetaflightSITLPlugin] Invalid Boreas configuration [" +
              _path.string() + "]: " + error.what()) << "\n";
    this->rotors.clear();
    return false;
  }

  gzlog << InfoText("[EthrcBetaflightSITLPlugin] Loaded airframe configuration [" +
           _path.string() + "].") << "\n";
  return true;
}

bool EthrcBetaflightSITLPlugin::Bind()
{
  {
    std::lock_guard<std::mutex> lock(gMotorSocketMutex);
    if (gMotorSocketBound)
    {
      gzwarn << WarnText("[EthrcBetaflightSITLPlugin] Another plugin instance in this "
                 "Gazebo process already owns the Betaflight motor UDP socket; "
                 "disabling this duplicate instance.") << "\n";
      return false;
    }
  }

  this->socketFd = socket(AF_INET, SOCK_DGRAM, 0);
  if (this->socketFd < 0)
  {
    gzerr << ErrorText("[EthrcBetaflightSITLPlugin] Failed to create UDP socket.") << "\n";
    return false;
  }

  int flags = fcntl(this->socketFd, F_GETFD);
  if (flags >= 0)
    fcntl(this->socketFd, F_SETFD, flags | FD_CLOEXEC);

  sockaddr_in sockaddr;
  MakeSockAddr(this->bindAddress, this->motorInputPort, sockaddr);
  if (bind(this->socketFd, reinterpret_cast<struct sockaddr *>(&sockaddr),
      sizeof(sockaddr)) != 0)
  {
    gzerr << ErrorText("[EthrcBetaflightSITLPlugin] Failed to bind " +
              this->bindAddress + ":" + std::to_string(this->motorInputPort) + ".") << "\n";
    close(this->socketFd);
    this->socketFd = -1;
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(gMotorSocketMutex);
    gMotorSocketBound = true;
    this->ownsMotorSocket = true;
  }

  return true;
}

bool EthrcBetaflightSITLPlugin::ReceiveMotorCommand()
{
  if (this->socketFd < 0)
    return false;

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(this->socketFd, &fds);

  timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = this->betaflightOnline ? 1000 : 0;

  if (select(this->socketFd + 1, &fds, nullptr, nullptr, &timeout) != 1)
  {
    if (this->betaflightOnline &&
        ++this->missedPackets > this->connectionTimeoutMaxCount)
    {
      this->betaflightOnline = false;
      for (auto &rotor : this->rotors)
      {
        rotor.normalizedCommand = 0.0;
        rotor.dynamics.Reset();
      }
      gzwarn << WarnText("[EthrcBetaflightSITLPlugin] Betaflight motor stream timed out.") << "\n";
    }
    return false;
  }

  ServoPacket packet;
  const ssize_t recvSize = recv(this->socketFd, &packet, sizeof(packet), 0);
  const ssize_t expectedSize =
      static_cast<ssize_t>(this->rotors.size() * sizeof(packet.motorSpeed[0]));
  if (recvSize < expectedSize)
  {
    gzwarn << WarnText("[EthrcBetaflightSITLPlugin] Ignoring short motor packet: " +
              std::to_string(recvSize) + " bytes, expected at least " +
              std::to_string(expectedSize) + ".") << "\n";
    return false;
  }

  this->betaflightOnline = true;
  this->missedPackets = 0;

  for (size_t i = 0; i < this->rotors.size() && i < kMaxMotors; ++i)
  {
    const double command = static_cast<double>(packet.motorSpeed[i]);
    if (!std::isfinite(command))
    {
      gzwarn << WarnText("[EthrcBetaflightSITLPlugin] Ignoring non-finite motor command " +
                std::to_string(i) + ".") << "\n";
      return false;
    }

    this->rotors[i].normalizedCommand = std::clamp(command, 0.0, 1.0);
  }

  return true;
}

bool EthrcBetaflightSITLPlugin::StateIsSane(EntityComponentManager &_ecm)
{
  const auto poseComp = _ecm.Component<components::Pose>(this->modelEntity);
  const auto linearVelComp = _ecm.Component<components::WorldLinearVelocity>(
      this->baseLink.Entity());
  const auto angularVelComp = _ecm.Component<components::WorldAngularVelocity>(
      this->baseLink.Entity());

  if (!poseComp)
    return true;

  const math::Pose3d pose = poseComp->Data();
  if (!IsFinite(pose))
  {
    this->ResetVehicleState(_ecm, "non-finite pose");
    return false;
  }

  const math::Vector3d pos = pose.Pos();
  if (std::abs(pos.X()) > this->maxWorldPosition ||
      std::abs(pos.Y()) > this->maxWorldPosition ||
      pos.Z() < this->minWorldAltitude ||
      pos.Z() > this->maxWorldAltitude)
  {
    this->ResetVehicleState(_ecm,
        "pose outside SITL safety bounds: pos=" + FormatVector(pos) +
        ", bounds xy=+/-" + std::to_string(this->maxWorldPosition) +
        ", z=[" + std::to_string(this->minWorldAltitude) + ", " +
        std::to_string(this->maxWorldAltitude) + "]");
    return false;
  }

  if (linearVelComp && (!IsFinite(linearVelComp->Data()) ||
      linearVelComp->Data().Length() > this->maxLinearSpeed))
  {
    const math::Vector3d linearVel = linearVelComp->Data();
    this->ResetVehicleState(_ecm,
        "linear velocity outside SITL safety bounds: vel=" +
        FormatVector(linearVel) + ", speed=" +
        std::to_string(linearVel.Length()) + " m/s, max=" +
        std::to_string(this->maxLinearSpeed) + " m/s");
    return false;
  }

  if (angularVelComp && (!IsFinite(angularVelComp->Data()) ||
      angularVelComp->Data().Length() > this->maxAngularSpeed))
  {
    const math::Vector3d angularVel = angularVelComp->Data();
    this->ResetVehicleState(_ecm,
        "angular velocity outside SITL safety bounds: omega=" +
        FormatVector(angularVel) + ", speed=" +
        std::to_string(angularVel.Length()) + " rad/s, max=" +
        std::to_string(this->maxAngularSpeed) + " rad/s");
    return false;
  }

  this->stateGuardLatched = false;
  return true;
}

void EthrcBetaflightSITLPlugin::ResetVehicleState(EntityComponentManager &_ecm,
    const std::string &_reason)
{
  for (auto &rotor : this->rotors)
  {
    rotor.normalizedCommand = 0.0;
    rotor.dynamics.Reset();
  }
  this->UpdateRotorVisuals(_ecm);

  this->betaflightOnline = false;
  this->missedPackets = 0;
  this->launchReleased = false;
  this->previousOrientationValid = false;
  this->model.SetWorldPoseCmd(_ecm, this->resetPose);
  this->baseLink.SetLinearVelocity(_ecm, {0, 0, 0});
  this->baseLink.SetAngularVelocity(_ecm, {0, 0, 0});

  if (!this->stateGuardLatched)
  {
    gzwarn << ResetText("[EthrcBetaflightSITLPlugin] RESETTING VEHICLE STATE: " +
               _reason + ".") << "\n";
    this->stateGuardLatched = true;
  }
}

void EthrcBetaflightSITLPlugin::ApplyForces(
    EntityComponentManager &_ecm, const double _dtSeconds)
{
  const auto poseComp = _ecm.Component<components::Pose>(this->modelEntity);
  if (!poseComp)
    return;

  const math::Vector3d bodyUpWorld =
      poseComp->Data().Rot().RotateVector({0, 0, 1});
  this->lastAppliedForceWorld.Set(0, 0, 0);
  this->lastAppliedTorqueWorld.Set(0, 0, 0);
  for (auto &rotor : this->rotors)
  {
    rotor.dynamics.Step(rotor.normalizedCommand, _dtSeconds);
    const double thrust = rotor.dynamics.ThrustN();
    const math::Vector3d forceWorld = bodyUpWorld * thrust;
    const math::Vector3d torqueWorld =
        bodyUpWorld * rotor.dynamics.ReactionTorqueNm(rotor.turningDirection);

    const math::Vector3d rotorOffsetWorld =
        poseComp->Data().Rot().RotateVector(rotor.position);
    this->lastAppliedForceWorld += forceWorld;
    this->lastAppliedTorqueWorld += torqueWorld + rotorOffsetWorld.Cross(forceWorld);
    // Do not drive rotor joints with JointForceCmd here. The propulsion model
    // applies thrust directly; joint force commands would inject physical
    // torque into the airframe instead of acting as visual-only RPM.
  }

  // AddWorldWrench applies at the link origin. Forming each rotor's moment
  // about that same origin lets Gazebo account for the inertial COM offset.
  this->baseLink.AddWorldWrench(
      _ecm, this->lastAppliedForceWorld, this->lastAppliedTorqueWorld);

  const auto linearVelComp = _ecm.Component<components::WorldLinearVelocity>(
      this->baseLink.Entity());
  const auto angularVelComp = _ecm.Component<components::WorldAngularVelocity>(
      this->baseLink.Entity());
  const math::Quaterniond bodyToWorld = poseComp->Data().Rot();

  if (linearVelComp)
  {
    const math::Vector3d bodyVelocity =
        bodyToWorld.RotateVectorReverse(linearVelComp->Data());
    const auto drag = ethrc::sitl::QuadraticDrag(
        {bodyVelocity.X(), bodyVelocity.Y(), bodyVelocity.Z()},
        this->translationalDrag);
    const math::Vector3d dragWorld =
        bodyToWorld.RotateVector({drag[0], drag[1], drag[2]});
    this->baseLink.AddWorldWrench(_ecm, dragWorld, {0, 0, 0});
    this->lastAppliedForceWorld += dragWorld;
  }

  if (angularVelComp)
  {
    const math::Vector3d bodyAngularVelocity =
        bodyToWorld.RotateVectorReverse(angularVelComp->Data());
    const auto drag = ethrc::sitl::QuadraticDrag(
        {bodyAngularVelocity.X(), bodyAngularVelocity.Y(), bodyAngularVelocity.Z()},
        this->angularDrag);
    const math::Vector3d dragWorld =
        bodyToWorld.RotateVector({drag[0], drag[1], drag[2]});
    this->baseLink.AddWorldWrench(_ecm, {0, 0, 0}, dragWorld);
    this->lastAppliedTorqueWorld += dragWorld;
  }
}

void EthrcBetaflightSITLPlugin::PublishDiagnostics()
{
  msgs::Double_V commands;
  msgs::Double_V rpm;
  msgs::Double_V thrust;
  for (const auto &rotor : this->rotors)
  {
    commands.add_data(rotor.normalizedCommand);
    rpm.add_data(rotor.dynamics.Rpm());
    thrust.add_data(rotor.dynamics.ThrustN());
  }
  this->motorCommandPublisher.Publish(commands);
  this->motorRpmPublisher.Publish(rpm);
  this->motorThrustPublisher.Publish(thrust);

  msgs::Wrench wrench;
  wrench.mutable_force()->set_x(this->lastAppliedForceWorld.X());
  wrench.mutable_force()->set_y(this->lastAppliedForceWorld.Y());
  wrench.mutable_force()->set_z(this->lastAppliedForceWorld.Z());
  wrench.mutable_torque()->set_x(this->lastAppliedTorqueWorld.X());
  wrench.mutable_torque()->set_y(this->lastAppliedTorqueWorld.Y());
  wrench.mutable_torque()->set_z(this->lastAppliedTorqueWorld.Z());
  this->appliedWrenchPublisher.Publish(wrench);

  msgs::StringMsg profile;
  profile.set_data(this->airframeProfile + ":" + this->calibrationStatus);
  this->modelProfilePublisher.Publish(profile);
}

void EthrcBetaflightSITLPlugin::UpdateRotorVisuals(EntityComponentManager &_ecm)
{
  bool anyRotorSpinning = false;
  for (const auto &rotor : this->rotors)
  {
    anyRotorSpinning = anyRotorSpinning || rotor.dynamics.NormalizedSpeed() > 0.001;

    if (rotor.jointEntity == kNullEntity)
      continue;

    const double rpm = rotor.dynamics.NormalizedSpeed() * this->visualMaxRpm;
    const double angularVelocity =
        rotor.turningDirection * rpm * 2.0 * GZ_PI / 60.0;
    _ecm.SetComponentData<components::JointVelocityCmd>(
        rotor.jointEntity, {angularVelocity});
  }

  if (anyRotorSpinning != this->rotorVisualsSpinning)
  {
    gzlog << InfoText(
        std::string("[EthrcBetaflightSITLPlugin] Rotor visuals ") +
        (anyRotorSpinning ? "spinning from Betaflight motor output."
                          : "stopped.")) << "\n";
    this->rotorVisualsSpinning = anyRotorSpinning;
  }
}

void EthrcBetaflightSITLPlugin::SendState(const UpdateInfo &_info,
    const EntityComponentManager &_ecm) const
{
  if (this->socketFd < 0)
    return;

  auto accelComp = _ecm.Component<components::LinearAcceleration>(this->imuEntity);
  auto poseComp = _ecm.Component<components::Pose>(this->modelEntity);
  auto velComp = _ecm.Component<components::WorldLinearVelocity>(
      this->baseLink.Entity());

  if (!accelComp || !poseComp || !velComp)
    return;

  FdmPacket packet{};
  packet.timestamp = std::chrono::duration<double>(_info.simTime).count();

  const math::Vector3d linearAccel = accelComp->Data();
  packet.imuLinearAccelerationXYZ[0] = linearAccel.X();
  packet.imuLinearAccelerationXYZ[1] = linearAccel.Y();
  packet.imuLinearAccelerationXYZ[2] = linearAccel.Z();

  const math::Quaterniond orientation = poseComp->Data().Rot();
  math::Vector3d angularVelBodyFlu{0, 0, 0};
  const double dtSeconds = std::chrono::duration<double>(_info.dt).count();
  if (this->previousOrientationValid && dtSeconds > 0.0)
  {
    math::Quaterniond delta = this->previousOrientation.Inverse() * orientation;
    delta.Normalize();
    if (delta.W() < 0.0)
      delta = {-delta.W(), -delta.X(), -delta.Y(), -delta.Z()};
    math::Vector3d axis;
    double angle = 0.0;
    delta.AxisAngle(axis, angle);
    angularVelBodyFlu = axis * (angle / dtSeconds);
  }
  this->previousOrientation = orientation;
  this->previousOrientationValid = true;
  packet.imuAngularVelocityRPY[0] = angularVelBodyFlu.X();
  packet.imuAngularVelocityRPY[1] = -angularVelBodyFlu.Y();
  packet.imuAngularVelocityRPY[2] = -angularVelBodyFlu.Z();

  const math::Pose3d modelWorldPose = poseComp->Data();
  const math::Pose3d gazeboToNED(0, 0, 0, GZ_PI, 0, 0);
  const math::Pose3d worldToModel(
      gazeboToNED.Pos() + gazeboToNED.Rot().RotateVector(modelWorldPose.Pos()),
      gazeboToNED.Rot() * modelWorldPose.Rot());
  const math::Pose3d nedToModel(
      worldToModel.Pos() - gazeboToNED.Pos(),
      worldToModel.Rot() * gazeboToNED.Rot().Inverse());

  packet.positionXYZ[0] = nedToModel.Pos().X();
  packet.positionXYZ[1] = nedToModel.Pos().Y();
  packet.positionXYZ[2] = nedToModel.Pos().Z();

  packet.imuOrientationQuat[0] = nedToModel.Rot().W();
  packet.imuOrientationQuat[1] = nedToModel.Rot().X();
  packet.imuOrientationQuat[2] = nedToModel.Rot().Y();
  packet.imuOrientationQuat[3] = nedToModel.Rot().Z();

  const math::Vector3d velGazeboWorldFrame = velComp->Data();
  const math::Vector3d velNedFrame =
      gazeboToNED.Rot().RotateVectorReverse(velGazeboWorldFrame);
  packet.velocityXYZ[0] = velNedFrame.X();
  packet.velocityXYZ[1] = velNedFrame.Y();
  packet.velocityXYZ[2] = velNedFrame.Z();

  for (size_t i = 0; i < this->rotors.size() && i < 4; ++i)
    packet.escRpm[i] = this->rotors[i].dynamics.Rpm();

  sockaddr_in sockaddr;
  MakeSockAddr(this->betaflightAddress, this->stateOutputPort, sockaddr);
  sendto(this->socketFd, &packet, sizeof(packet), 0,
      reinterpret_cast<struct sockaddr *>(&sockaddr), sizeof(sockaddr));
}

void EthrcBetaflightSITLPlugin::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  if (_info.paused || this->rotors.empty() || this->socketFd < 0)
    return;

  std::lock_guard<std::mutex> lock(this->mutex);
  if (_info.simTime < this->lastUpdateTime)
  {
    this->lastUpdateTime = std::chrono::steady_clock::duration{0};
    this->missedPackets = 0;
    this->stateGuardLatched = false;
    this->launchReleased = false;
    this->previousOrientationValid = false;
  }
  else if (_info.simTime == this->lastUpdateTime)
  {
    return;
  }

  if (!this->StateIsSane(_ecm))
  {
    this->lastUpdateTime = _info.simTime;
    return;
  }

  this->ReceiveMotorCommand();
  this->UpdateRotorVisuals(_ecm);
  if (!this->betaflightOnline)
  {
    // Keep the vehicle on its virtual launch fixture while the UDP handshake
    // starts. Send FDM state even before the first motor packet so Betaflight
    // can initialize its simulator loop without a gravity-induced startup
    // drop or a motor/state deadlock.
    this->model.SetWorldPoseCmd(_ecm, this->resetPose);
    this->baseLink.SetLinearVelocity(_ecm, {0, 0, 0});
    this->baseLink.SetAngularVelocity(_ecm, {0, 0, 0});
    this->lastUpdateTime = _info.simTime;
    return;
  }
  if (this->betaflightOnline)
  {
    if (!this->launchReleased)
    {
      double maximumCommand = 0.0;
      for (const auto &rotor : this->rotors)
        maximumCommand = std::max(maximumCommand, rotor.normalizedCommand);

      if (maximumCommand < this->launchReleaseCommand)
      {
        // Keep disarmed SITL off contact geometry. This is a launch fixture,
        // not an aerodynamic force, and is removed before propulsion acts.
        this->model.SetWorldPoseCmd(_ecm, this->resetPose);
        this->baseLink.SetLinearVelocity(_ecm, {0, 0, 0});
        this->baseLink.SetAngularVelocity(_ecm, {0, 0, 0});
        this->PublishDiagnostics();
        this->lastUpdateTime = _info.simTime;
        return;
      }

      this->launchReleased = true;
      this->baseLink.SetLinearVelocity(_ecm, {0, 0, 0});
      this->baseLink.SetAngularVelocity(_ecm, {0, 0, 0});
      // Link::Set*Velocity creates persistent command components. They are
      // useful while the virtual launch restraint is active, but must be
      // removed before propulsion is applied or PhysX will keep forcing both
      // velocities back to zero and suppress genuine tilted-thrust motion.
      _ecm.RemoveComponent<components::LinearVelocityCmd>(
          this->baseLink.Entity());
      _ecm.RemoveComponent<components::AngularVelocityCmd>(
          this->baseLink.Entity());
      _ecm.RemoveComponent<components::WorldPoseCmd>(this->modelEntity);
      gzlog << InfoText("[EthrcBetaflightSITLPlugin] Virtual launch restraint released.") << "\n";
    }

    this->ApplyForces(_ecm, std::chrono::duration<double>(_info.dt).count());
    this->PublishDiagnostics();
  }

  this->lastUpdateTime = _info.simTime;
}

void EthrcBetaflightSITLPlugin::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  if (_info.paused || this->rotors.empty() || this->socketFd < 0)
    return;

  std::lock_guard<std::mutex> lock(this->mutex);
  this->SendState(_info, _ecm);
}
}  // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::EthrcBetaflightSITLPlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::EthrcBetaflightSITLPlugin,
                    "gz::sim::systems::EthrcBetaflightSITLPlugin")
