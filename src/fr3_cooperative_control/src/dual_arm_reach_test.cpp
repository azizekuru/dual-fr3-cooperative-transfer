// =============================================================================
//  dual_arm_reach_test.cpp  -  ASAMA 1: SADECE UZANMA TESTI (kutu yok, gripper yok)
//
//  Akis (stop_after parametresi ile istenen adimda durdurulabilir):
//    ADIM 1  iki kol ready pozuna gider
//    ADIM 2  arm_1 shelf_1'e uzanir, geri ceker
//    ADIM 3  arm_1 masaya uzanir, geri ceker, ready'ye doner
//    ADIM 4  arm_2 masaya uzanir (ustten), geri ceker
//    ADIM 5  arm_2 shelf_2'ye uzanir, geri ceker, ready'ye doner
//
//  Calistirma:
//    ros2 launch fr3_cooperative_control reach_test.launch.py            (tek komut, onerilen)
//    ros2 run fr3_cooperative_control dual_arm_reach_test --ros-args -p use_sim_time:=true -p stop_after:=2
//
//  Parametreler (hepsi opsiyonel):
//    stop_after      (int)    bu adimdan sonra dur, varsayilan 99
//    shelf_1_y       (double) shelf_1 hedef noktasinin y'si, -1.04
//    shelf_2_y       (double) shelf_2 hedef noktasinin y'si, 1.04
//    handover_y      (double) masadaki hedef noktanin y'si, 0.05
//    speed           (double) ready / on-poz hareketlerinin hizi (joint limits olcegi), 0.7
//    approach_speed  (double) hedefe duz cizgi yaklasma hizi, 0.3
//    settle_timeout  (double) planlamadan once kolun durmasini bekleme suresi (sn), 2.0; 0 = bekleme
//    via_ready       (bool)   shelf_1'den sonra masaya gecmeden ready'ye ugra, false
//
//  Koordinatlar MoveIt (URDF) cercevesindedir. Gazebo'da robot z=+0.05 ile spawn edildigi icin
//  Gazebo dunya z'si = buradaki z + 0.05.
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rclcpp/rclcpp.hpp>

using geometry_msgs::msg::Pose;
using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;
using namespace std::chrono_literals;

namespace
{

// ------------------------------------------------------------------ sabitler
constexpr double kBox           = 0.04;    // hayali kutu boyutu (sadece yukseklik hesabi icin)
constexpr double kGrip          = 0.012;   // TCP, kutu merkezinin bu kadar ustunde
constexpr double kShelfSurfaceZ = 0.4625;  // raflarin 2. kat ust yuzeyi
constexpr double kTableTopZ     = 0.40;    // masa ust yuzeyi
constexpr double kStandoff      = 0.12;    // on-poz ile hedef arasi mesafe
constexpr double kClearance     = 0.01;    // yuzeyin 1 cm ustunde dur
const std::string kBoxId = "target_box";

const std::vector<double> kReady = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};

geometry_msgs::msg::Quaternion makeQuat(double x, double y, double z, double w)
{
  geometry_msgs::msg::Quaternion q;
  q.x = x; q.y = y; q.z = z; q.w = w;
  return q;
}

Pose makePose(double x, double y, double z, const geometry_msgs::msg::Quaternion & q)
{
  Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation = q;
  return p;
}

// hand_tcp eksenleri (world cercevesinde). z = parmaklarin baktigi yon, y = parmak kapanma ekseni.
//   shelf_1 (arm_1): x=(0,0,-1) y=(1,0,0)  z=(0,-1,0)   yatay, -y yonune bakar
//   masa    (arm_1): x=(0,0,-1) y=(-1,0,0) z=(0,1,0)    yatay, +y yonune bakar
//   masa    (arm_2): x=(0,1,0)  y=(1,0,0)  z=(0,0,-1)   dikey, asagi bakar
//   shelf_2 (arm_2): x=(0,0,1)  y=(1,0,0)  z=(0,1,0)    yatay, +y yonune bakar
const double kS = std::sqrt(0.5);
const auto kQShelf1     = makeQuat( 0.5,  0.5, -0.5, 0.5);
const auto kQTableLeft  = makeQuat(-0.5,  0.5,  0.5, 0.5);
const auto kQTopDown    = makeQuat(kS, kS, 0.0, 0.0);
const auto kQShelf2     = makeQuat(-0.5, -0.5, -0.5, 0.5);

struct Arm
{
  std::string name;   // MoveIt grubu: arm_1 / arm_2
  std::string tcp;    // <side>_fr3v2_hand_tcp
  std::shared_ptr<MoveGroupInterface> group;
  moveit_msgs::msg::RobotTrajectory last_plan;
};

// Bir uzanma icin aday: on-poz + hedef (el yonu ve yukseklik farkli olabilir)
struct Variant
{
  Pose pre;
  Pose target;
  std::string desc;
};

// =============================================================================
class Coordinator
{
public:
  explicit Coordinator(const rclcpp::Node::SharedPtr & node)
  : node_(node), log_(node->get_logger())
  {
    node_->get_parameter_or<int64_t>("stop_after", stop_after_, 99);
    node_->get_parameter_or<double>("shelf_1_y", shelf_1_y_, -1.04);
    node_->get_parameter_or<double>("shelf_2_y", shelf_2_y_, 1.04);
    node_->get_parameter_or<double>("handover_y", handover_y_, 0.05);
    node_->get_parameter_or<bool>("via_ready", via_ready_, false);
    node_->get_parameter_or<bool>("tcp_diagnostics", tcp_diagnostics_, false);
    node_->get_parameter_or<double>("speed", speed_, 0.7);
    node_->get_parameter_or<double>("approach_speed", approach_speed_, 0.3);
    node_->get_parameter_or<double>("settle_timeout", settle_timeout_, 2.0);

    left_ = makeArm("arm_1", "left");
    right_ = makeArm("arm_2", "right");
  }

  bool run()
  {
    cleanScene();

    // ---- ADIM 1 -----------------------------------------------------------
    RCLCPP_INFO(log_, "ADIM 1: kollar ready pozuna gidiyor");
    if (!moveToJoints(left_, kReady, 0.5) || !moveToJoints(right_, kReady, 0.5)) {
      return fail("ready pozuna gidilemedi");
    }
    if (done(1)) {return true;}

    // ---- ADIM 2: arm_1 -> shelf_1 -------------------------------------------
    {
      const double z = kShelfSurfaceZ + kBox / 2.0 + kGrip;
      const Pose target = makePose(0.0, shelf_1_y_, z, kQShelf1);
      const Pose pre = makePose(0.0, shelf_1_y_ + kStandoff, z, kQShelf1);
      RCLCPP_INFO(log_, "ADIM 2: arm_1 shelf_1'e uzaniyor");
      if (!reach(left_, pre, target, "shelf_1")) {return false;}
      if (via_ready_ && !moveToJoints(left_, kReady, 0.5)) {return fail("arm_1 ready'ye donemedi");}
      if (done(2)) {return true;}
    }

    // ---- ADIM 3: arm_1 -> masa ----------------------------------------------
    {
      const double z = kTableTopZ + kBox / 2.0 + kClearance + kGrip;
      std::vector<Variant> vs;
      // Testte B yonu + 2 cm calisti (A yonunda link5 masaya degiyor), ilk o denenir.
      for (const double dz : {0.02, 0.05}) {
        vs.push_back(makeVariant(handover_y_ - kStandoff, handover_y_, z + dz, kQShelf2,
          "el yonu B (180 derece donuk), +" + cm(dz)));
      }
      for (const double dz : {0.02, 0.05}) {
        vs.push_back(makeVariant(handover_y_ - kStandoff, handover_y_, z + dz, kQTableLeft,
          "el yonu A, +" + cm(dz)));
      }
      RCLCPP_INFO(log_, "ADIM 3: arm_1 masaya uzaniyor");
      if (!reachAny(left_, vs, "masa")) {return false;}
      if (!moveToJoints(left_, kReady, 0.5)) {return fail("arm_1 ready'ye donemedi");}
      if (done(3)) {return true;}
    }

    // ---- ADIM 4: arm_2 -> masa (ustten) ---------------------------------------
    {
      const double y = handover_y_ + kGrip;
      const double z = kTableTopZ + kBox / 2.0;
      std::vector<Variant> vs;
      for (const double dz : {0.0, 0.03}) {
        vs.push_back(Variant{makePose(0.0, y, z + dz + 0.15, kQTopDown),
            makePose(0.0, y, z + dz, kQTopDown), "ustten, +" + cm(dz)});
      }
      RCLCPP_INFO(log_, "ADIM 4: arm_2 masaya uzaniyor");
      if (!reachAny(right_, vs, "masa (ustten)")) {return false;}
      if (done(4)) {return true;}
    }

    // ---- ADIM 5: arm_2 -> shelf_2 ---------------------------------------------
    {
      const double z = kShelfSurfaceZ + kBox / 2.0 + kClearance + kGrip;
      std::vector<Variant> vs;
      vs.push_back(makeVariant(shelf_2_y_ - kStandoff, shelf_2_y_, z, kQShelf2, "el yonu B, +0 cm"));
      vs.push_back(makeVariant(shelf_2_y_ - kStandoff, shelf_2_y_, z, kQTableLeft,
        "el yonu A (180 derece donuk), +0 cm"));
      vs.push_back(makeVariant(shelf_2_y_ - kStandoff, shelf_2_y_, z + 0.02, kQShelf2,
        "el yonu B, +2 cm"));
      RCLCPP_INFO(log_, "ADIM 5: arm_2 shelf_2'ye uzaniyor");
      if (!reachAny(right_, vs, "shelf_2")) {return false;}
      if (!moveToJoints(right_, kReady, 0.5)) {return fail("arm_2 ready'ye donemedi");}
    }

    RCLCPP_INFO(log_, "Senaryo tamamlandi.");
    return true;
  }

private:
  // ------------------------------------------------------------ yardimcilar --
  bool fail(const std::string & msg)
  {
    RCLCPP_ERROR(log_, "%s", msg.c_str());
    return false;
  }

  bool done(int step)
  {
    if (stop_after_ <= step) {
      RCLCPP_INFO(log_, "stop_after=%ld -> ADIM %d sonunda duruldu.",
        static_cast<long>(stop_after_), step);
      return true;
    }
    return false;
  }

  Arm makeArm(const std::string & group, const std::string & side)
  {
    Arm a;
    a.name = group;
    a.tcp = side + "_fr3v2_hand_tcp";
    a.group = std::make_shared<MoveGroupInterface>(node_, group);

    ensureJointLimits(a.group);
    a.group->setEndEffectorLink(a.tcp);          // hedefler parmak ucu (TCP) icin
    a.group->setPoseReferenceFrame("world");
    a.group->setPlanningTime(10.0);
    a.group->setNumPlanningAttempts(5);
    a.group->setGoalPositionTolerance(0.003);
    a.group->setGoalOrientationTolerance(0.02);
    return a;
  }

  // Bir hedefe uzanma: on-poz -> (bekle, olc) -> hedefe duz cizgi -> (bekle, olc) -> on-poza geri
  bool reach(Arm & a, const Pose & pre, const Pose & target, const std::string & label)
  {
    RCLCPP_INFO(log_, "%s [%s]: on-poza gidiyor", a.name.c_str(), label.c_str());
    if (!moveToPose(a, pre, 0.5)) {
      return fail(a.name + " [" + label + "] on-poz planlanamadi");
    }
    std::this_thread::sleep_for(500ms);   // kol otursun
    logTcp(a, pre);

    RCLCPP_INFO(log_, "%s [%s]: hedefe uzaniyor", a.name.c_str(), label.c_str());
    if (!moveLinear(a, {target}, 0.15)) {
      return fail(a.name + " [" + label + "] hedefe uzanamadi");
    }
    std::this_thread::sleep_for(500ms);
    logTcp(a, target);

    RCLCPP_INFO(log_, "%s [%s]: geri cekiliyor", a.name.c_str(), label.c_str());
    if (!moveLinear(a, {pre}, 0.15)) {
      return fail(a.name + " [" + label + "] geri cekilemedi");
    }
    return true;
  }

  static Variant makeVariant(
    double y_pre, double y_target, double z, const geometry_msgs::msg::Quaternion & q,
    const std::string & desc)
  {
    return Variant{makePose(0.0, y_pre, z, q), makePose(0.0, y_target, z, q), desc};
  }

  static std::string cm(double dz) {return std::to_string(static_cast<int>(dz * 100.0 + 0.5)) + " cm";}

  // Adaylari sirayla dener. Bir aday basarisiz olursa kol ready'ye doner, sonraki denenir.
  // Calisan adayin aciklamasi loga yazilir (sonraki asamalarda onu kullaniriz).
  bool reachAny(Arm & a, const std::vector<Variant> & vs, const std::string & label)
  {
    for (size_t i = 0; i < vs.size(); ++i) {
      RCLCPP_INFO(log_, "%s [%s]: aday %zu/%zu deneniyor: %s",
        a.name.c_str(), label.c_str(), i + 1, vs.size(), vs[i].desc.c_str());
      if (reach(a, vs[i].pre, vs[i].target, label)) {
        RCLCPP_INFO(log_, "%s [%s]: *** ADAY %zu CALISTI: %s ***",
          a.name.c_str(), label.c_str(), i + 1, vs[i].desc.c_str());
        return true;
      }
      RCLCPP_WARN(log_, "%s [%s]: aday %zu basarisiz, ready'ye donup sonrakini deniyorum",
        a.name.c_str(), label.c_str(), i + 1);
      if (!moveToJoints(a, kReady, 0.5)) {return fail("ready'ye donulemedi");}
    }
    return fail(a.name + " [" + label + "] hicbir aday calismadi");
  }

  // Bu dugum joint_limits.yaml'i yuklemiyor (onu sadece move_group biliyor), bu yuzden
  // Cartesian yolun zaman parametrizasyonu "No acceleration limit" ile patliyordu.
  // Tanimsiz hiz/ivme sinirlarini muhafazakar degerlerle tamamliyoruz; gercek hizi
  // moveLinear'daki olcek (0.15) belirler.
  void ensureJointLimits(const std::shared_ptr<MoveGroupInterface> & g)
  {
    auto model = std::const_pointer_cast<moveit::core::RobotModel>(g->getRobotModel());
    for (moveit::core::JointModel * jm : model->getActiveJointModels()) {
      for (const std::string & var : jm->getVariableNames()) {
        moveit::core::VariableBounds b = jm->getVariableBounds(var);
        bool changed = false;
        if (!b.velocity_bounded_) {
          b.velocity_bounded_ = true;
          b.min_velocity_ = -1.5;
          b.max_velocity_ = 1.5;
          changed = true;
        }
        if (!b.acceleration_bounded_) {
          b.acceleration_bounded_ = true;
          b.min_acceleration_ = -3.0;
          b.max_acceleration_ = 3.0;
          changed = true;
        }
        if (changed) {jm->setVariableBounds(var, b);}
      }
    }
  }

  // Kol durana kadar bekle. Gazebo'da kontrolcu gecikmesi yuzunden hareket "bitti" dendikten
  // sonra kol hala oturuyor olabilir; o zaman plan gercek baslangic durumundan sapar ve MoveIt
  // trajectory'yi baslangic toleransi asildi diye aninda reddeder (Execute request aborted).
  void waitSettled(Arm & a)
  {
    if (settle_timeout_ <= 0.0) {return;}
    auto prev = a.group->getCurrentJointValues();
    int calm = 0;
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < std::chrono::duration<double>(settle_timeout_)) {
      std::this_thread::sleep_for(250ms);
      const auto cur = a.group->getCurrentJointValues();
      double d = 0.0;
      for (size_t i = 0; i < cur.size() && i < prev.size(); ++i) {
        d = std::max(d, std::abs(cur[i] - prev[i]));
      }
      calm = (d < 1e-3) ? calm + 1 : 0;
      prev = cur;
      if (calm >= 2) {return;}
    }
    RCLCPP_DEBUG(log_, "%s %.1f sn icinde tam durmadi, yine de devam ediliyor",
      a.name.c_str(), settle_timeout_);
  }

  // ------------------------------------------------------------- hareketler --
  bool moveToJoints(Arm & a, const std::vector<double> & q, double scale)
  {
    (void)scale;   // hiz artik "speed" parametresinden gelir
    a.group->setMaxVelocityScalingFactor(speed_);
    a.group->setMaxAccelerationScalingFactor(speed_);
    constexpr int kAttempts = 6;
    for (int i = 1; i <= kAttempts; ++i) {
      waitSettled(a);
      a.group->setStartStateToCurrentState();
      a.group->setJointValueTarget(q);
      MoveGroupInterface::Plan plan;
      const auto ec = a.group->plan(plan);
      if (ec == moveit::core::MoveItErrorCode::SUCCESS) {
        if (tcp_diagnostics_) {a.last_plan = plan.trajectory;}
        const auto ex = a.group->execute(plan);
        if (ex == moveit::core::MoveItErrorCode::SUCCESS) {return true;}
        RCLCPP_WARN(log_, "%s joint plani uygulanamadi (deneme %d/%d, hata kodu %d), yeniden deneniyor",
          a.name.c_str(), i, kAttempts, ex.val);
        continue;
      }
      RCLCPP_WARN(log_, "%s joint plani basarisiz (deneme %d/%d, hata kodu %d)",
        a.name.c_str(), i, kAttempts, ec.val);
    }
    return false;
  }

  bool moveToPose(Arm & a, const Pose & target, double scale)
  {
    (void)scale;   // hiz artik "speed" parametresinden gelir
    a.group->setMaxVelocityScalingFactor(speed_);
    a.group->setMaxAccelerationScalingFactor(speed_);
    // OMPL rastgele calisir; yol dogrulamasi bazen takilir, birkac kez dene
    constexpr int kAttempts = 6;
    for (int i = 1; i <= kAttempts; ++i) {
      waitSettled(a);
      a.group->setStartStateToCurrentState();
      a.group->setPoseTarget(target);
      MoveGroupInterface::Plan plan;
      const auto ec = a.group->plan(plan);
      a.group->clearPoseTargets();
      if (ec == moveit::core::MoveItErrorCode::SUCCESS) {
        if (tcp_diagnostics_) {a.last_plan = plan.trajectory;}
        const auto ex = a.group->execute(plan);
        if (ex == moveit::core::MoveItErrorCode::SUCCESS) {return true;}
        RCLCPP_WARN(log_, "%s poz plani uygulanamadi (deneme %d/%d, hata kodu %d), yeniden deneniyor",
          a.name.c_str(), i, kAttempts, ex.val);
        continue;
      }
      RCLCPP_WARN(log_, "%s poz plani basarisiz (deneme %d/%d, hata kodu %d) hedef=(%.3f, %.3f, %.3f)",
        a.name.c_str(), i, kAttempts, ec.val,
        target.position.x, target.position.y, target.position.z);
    }
    return false;
  }

  // Duz cizgi (Cartesian) hareket: yaklasma / geri cekilme icin
  bool moveLinear(Arm & a, const std::vector<Pose> & waypoints, double scale)
  {
    waitSettled(a);
    a.group->setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory traj;
    const double fraction = a.group->computeCartesianPath(waypoints, 0.005, traj, true);
    if (fraction < 0.99) {
      // Teshis: ayni yolu cakisma kontrolu kapali dene. Tamamlaniyorsa sorun cakisma, degilse IK.
      moveit_msgs::msg::RobotTrajectory probe;
      const double free_fraction = a.group->computeCartesianPath(waypoints, 0.005, probe, false);
      RCLCPP_ERROR(log_,
        "%s Cartesian yol: cakisma kontrolu acikken %%%.0f, kapaliyken %%%.0f -> %s",
        a.name.c_str(), fraction * 100.0, free_fraction * 100.0,
        free_fraction >= 0.99 ? "yol IK ile erisilebilir, sorun CARPISMA" :
                                "kapaliyken de tamamlanmiyor, sorun IK / erisim");
      if (free_fraction >= 0.99) {explainCollision(a, probe);}
      return false;
    }

    const auto state = a.group->getCurrentState();
    if (!state) {
      RCLCPP_ERROR(log_, "%s guncel durum alinamadi", a.name.c_str());
      return false;
    }
    robot_trajectory::RobotTrajectory rt(a.group->getRobotModel(), a.group->getName());
    rt.setRobotTrajectoryMsg(*state, traj);
    (void)scale;   // hiz artik "approach_speed" parametresinden gelir
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(rt, approach_speed_, approach_speed_)) {
      RCLCPP_ERROR(log_, "%s zaman parametrizasyonu basarisiz", a.name.c_str());
      return false;
    }
    rt.getRobotTrajectoryMsg(traj);
    if (tcp_diagnostics_) {a.last_plan = traj;}
    return a.group->execute(traj) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  // Cartesian yol carpismadan kesilirse: cakismasiz yolun her noktasini MoveIt'in
  // /check_state_validity servisine sorup ilk gecersiz noktayi ve temas eden linkleri yazdirir.
  void explainCollision(Arm & a, const moveit_msgs::msg::RobotTrajectory & path)
  {
    auto client =
      node_->create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
    if (!client->wait_for_service(2s)) {
      RCLCPP_WARN(log_, "/check_state_validity servisi yok, teshis yapilamadi");
      return;
    }
    const auto & jt = path.joint_trajectory;
    for (size_t i = 0; i < jt.points.size(); ++i) {
      auto req = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
      req->group_name = a.name;
      req->robot_state.joint_state.name = jt.joint_names;
      req->robot_state.joint_state.position = jt.points[i].positions;
      auto fut = client->async_send_request(req);
      if (fut.wait_for(3s) != std::future_status::ready) {continue;}
      const auto res = fut.get();
      if (!res->valid) {
        RCLCPP_ERROR(log_, "%s: yolun %zu. noktasi (toplam %zu) gecersiz, ~%.1f cm ilerledikten sonra",
          a.name.c_str(), i, jt.points.size(), i * 0.5);
        for (const auto & c : res->contacts) {
          RCLCPP_ERROR(log_, "  temas: %s <-> %s  konum=(%.3f, %.3f, %.3f)  derinlik=%.4f",
            c.contact_body_1.c_str(), c.contact_body_2.c_str(),
            c.position.x, c.position.y, c.position.z, c.depth);
        }
        return;
      }
    }
    RCLCPP_WARN(log_, "%s: cakismasiz yolun hicbir noktasi gecersiz bulunmadi", a.name.c_str());
  }

  // TCP'nin gercek konumu ile hedefi karsilastirir (MoveIt cercevesi).
  void logTcp(Arm & a, const Pose & expected)
  {
    const auto p = a.group->getCurrentPose(a.tcp).pose;
    const double dx = p.position.x - expected.position.x;
    const double dy = p.position.y - expected.position.y;
    const double dz = p.position.z - expected.position.z;
    RCLCPP_INFO(log_,
      "%s TCP gercek=(%.3f, %.3f, %.3f)  hedef=(%.3f, %.3f, %.3f)  sapma=%.1f mm",
      a.name.c_str(), p.position.x, p.position.y, p.position.z,
      expected.position.x, expected.position.y, expected.position.z,
      1000.0 * std::sqrt(dx * dx + dy * dy + dz * dz));
    if (!tcp_diagnostics_ || a.last_plan.joint_trajectory.points.empty()) {return;}
    const auto & jt = a.last_plan.joint_trajectory;
    auto state = a.group->getCurrentState(2.0);
    if (!state) {RCLCPP_ERROR(log_, "TCP DIAG: no fresh state");return;}
    auto planned = *state;
    planned.setVariablePositions(jt.joint_names, jt.points.back().positions);
    planned.update();
    const auto endpoint = planned.getGlobalLinkTransform(a.tcp);
    const Eigen::Vector3d requested(expected.position.x, expected.position.y, expected.position.z);
    const Eigen::Quaterniond requested_rotation(expected.orientation.w, expected.orientation.x,
      expected.orientation.y, expected.orientation.z);
    RCLCPP_INFO(log_, "TCP DIAG %s: planned_endpoint_error=%.3f mm angle=%.4f rad",
      a.name.c_str(), 1000.0 * (endpoint.translation() - requested).norm(),
      Eigen::Quaterniond(endpoint.rotation()).angularDistance(requested_rotation));
    const auto started = std::chrono::steady_clock::now();
    for (int sample = 0; sample <= 10; ++sample) {
      if (sample > 0) {std::this_thread::sleep_for(500ms);}
      state = a.group->getCurrentState(2.0);
      if (!state) {RCLCPP_ERROR(log_, "TCP DIAG: missing fresh sample");continue;}
      state->update();
      const auto actual = state->getGlobalLinkTransform(a.tcp);
      double max_joint_error = 0.0;
      for (size_t i = 0; i < jt.joint_names.size(); ++i) {
        max_joint_error = std::max(max_joint_error,
          std::abs(state->getVariablePosition(jt.joint_names[i]) - jt.points.back().positions[i]));
      }
      RCLCPP_INFO(log_,
        "TCP DIAG %s t=%.2f s: target_error=%.3f mm tracking_error=%.3f mm joint_error=%.6f rad",
        a.name.c_str(), std::chrono::duration<double>(
          std::chrono::steady_clock::now() - started).count(),
        1000.0 * (actual.translation() - requested).norm(),
        1000.0 * (actual.translation() - endpoint.translation()).norm(), max_joint_error);
    }

  }

  // --------------------------------------------------------- planning scene --
  // Onceki calistirmalardan kalmis olabilecek kutuyu sahneden temizle
  void cleanScene()
  {
    const auto attached = psi_.getAttachedObjects({kBoxId});
    for (const auto & kv : attached) {
      moveit_msgs::msg::AttachedCollisionObject aco;
      aco.link_name = kv.second.link_name;
      aco.object.id = kBoxId;
      aco.object.operation = aco.object.REMOVE;
      psi_.applyAttachedCollisionObject(aco);
    }
    for (const auto & n : psi_.getKnownObjectNames()) {
      if (n == kBoxId) {
        psi_.removeCollisionObjects({kBoxId});
        break;
      }
    }
  }

  // ------------------------------------------------------------------ veri --
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger log_;
  PlanningSceneInterface psi_;
  Arm left_, right_;

  int64_t stop_after_ = 99;
  double shelf_1_y_ = -1.04;
  double shelf_2_y_ = 1.04;
  double handover_y_ = 0.05;
  bool tcp_diagnostics_ = false;
  bool via_ready_ = false;        // shelf_1'den masaya gecerken once ready'ye uğra (true) ya da dogrudan git (false)
  double speed_ = 0.7;            // serbest hareketler (ready, on-poz): joint limits'in kaci
  double approach_speed_ = 0.3;   // hedefe duz cizgi yaklasma / geri cekilme
  double settle_timeout_ = 2.0;   // her planlamadan once kolun durmasini en cok bu kadar bekle (0 = bekleme)
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "dual_arm_reach_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Servis cagrilari ve MoveIt callback'leri icin dugumu ayri thread'de donduruyoruz
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  int rc = 0;
  {
    Coordinator coordinator(node);
    rc = coordinator.run() ? 0 : 1;
  }

  rclcpp::shutdown();
  spinner.join();
  return rc;
}
