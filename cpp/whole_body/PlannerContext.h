/**
 * @file
 * @brief 控制核心的对象装配入口：模型/OCP → 已验收数学包装 → SQP MPC → 参考与诊断。
 * 本文件主要管理依赖关系，不是 SQP 算法实现。TASK5A/TASK5C 名称是保留的历史配置。
 */
#pragma once
#include "controller/WBMpcController.h"
#include "diagnostics/StructuralObserver.h"
#include "diagnostics/TimeLog.h"
#include "math/GeneratedFootContracts.h"
#include "math/GeneratedFootCost.h"
#include "math/GeneratedQuadraticCosts.h"
#include "math/GeometryInstallBridge.h"
#include "reference/ExternalReferenceAdapter.h"
#include "time/CanonicalControlTime.h"

#include <humanoid_wb_mpc/WBMpcInterface.h>

#include <algorithm>
#include <sstream>
#include <unordered_map>
using namespace ocs2;
using namespace ocs2::humanoid;
/**
 * @brief 从工程根目录派生模型与配置路径，并保留既有日志/运行配置字段。
 * horizon=4.8 是此历史输入结构的字段，实际 MPC 预测时域由 task.info/MPC 配置读取，
 * 不能把该字段直接理解为求解器预测时域。
 */
struct Inputs {
  std::string backend, task, ref, urdf, gait, mjcf, root, recorded;
  double horizon, pacingMs, startup;
  bool heavy;
  Inputs(const std::string &projectRoot, const std::string &logDirectory)
      : backend("simscape"), task(projectRoot + "/config/task.info"),
        ref(projectRoot + "/config/reference.info"),
        urdf(projectRoot + "/assets/g1_29dof.urdf"),
        gait(projectRoot + "/config/gait.info"), mjcf("unused"),
        root(logDirectory), recorded("unused"), horizon(4.8), pacingMs(0),
        startup(0), heavy(false) {}
};
/**
 * @brief 拥有一套完整规划上下文。成员按声明顺序初始化、按逆序析构。
 * interface 先提供基础 OCP，migratedProblem 包装既有数学实现，mpc 使用包装后的问题。
 * Evidence 对象记录真实数学调用来源；它们不是新的代价或约束。
 */
struct PlannerContext {
  WBMpcInterface interface;
  std::shared_ptr<task5c::Evidence> mathematicalEvidence, frictionEvidence,
      polynomialEvidence, relaxedEvidence, jointEvidence, terminalEvidence,
      runningEvidence, stateEvidence, nominalStateEvidence,
      nominalInputEvidence, preEvidence, stanceEvidence, swingEvidence,
      momentEvidence, collisionEvidence, footCostEvidence;
  OptimalControlProblem migratedProblem;
  task5c::GeneratedInitializer migratedInitializer;
  SqpMpc mpc;
  std::shared_ptr<TimeLog> log;
  std::shared_ptr<task5a::ExternalReferenceAdapter> external;
  std::shared_ptr<task5b::StructuralObserver> structure;
  task5a::Packet currentPacket;
  std::map<uint64_t, task5a::Packet> packets;
  bool migrated;
  uint64_t expectedSession = 0;
/**
 * @brief 先构建 WBMpcInterface，再从内向外安装数学包装和初始化器，最后创建 SqpMpc。
 * install 链涵盖接触约束、罚函数、关节范围、二次代价、预计算与足端数学等边界；
 * 具体 Legacy/Shadow/生成实现权限由既有环境配置控制，并非全部在此重新定义。
 * 随后绑定 Simulink 外部参考：每次求解都检查 session 与实际观测 tick 一致。
 * 日志和结构观察器挂到求解同步接口；/proc/self/maps 仅用于 Linux 运行依赖记录。
 */
  PlannerContext(const Inputs &i)
      : interface(i.task, i.urdf, i.ref),
        mathematicalEvidence(std::make_shared<task5c::Evidence>(i.root)),
        frictionEvidence(
            std::make_shared<task5c::Evidence>(i.root, "friction_")),
        polynomialEvidence(
            std::make_shared<task5c::Evidence>(i.root, "polynomial_")),
        relaxedEvidence(std::make_shared<task5c::Evidence>(i.root, "relaxed_")),
        jointEvidence(std::make_shared<task5c::Evidence>(i.root, "joint_")),
        terminalEvidence(
            std::make_shared<task5c::Evidence>(i.root, "terminal_")),
        runningEvidence(std::make_shared<task5c::Evidence>(i.root, "running_")),
        stateEvidence(std::make_shared<task5c::Evidence>(i.root, "state_")),
        nominalStateEvidence(
            std::make_shared<task5c::Evidence>(i.root, "nominal_state_")),
        nominalInputEvidence(
            std::make_shared<task5c::Evidence>(i.root, "nominal_input_")),
        preEvidence(
            std::make_shared<task5c::Evidence>(i.root, "precomputation_")),
        stanceEvidence(std::make_shared<task5c::Evidence>(i.root, "stance_")),
        swingEvidence(std::make_shared<task5c::Evidence>(i.root, "swing_")),
        momentEvidence(std::make_shared<task5c::Evidence>(i.root, "moment_")),
        collisionEvidence(
            std::make_shared<task5c::Evidence>(i.root, "collision_")),
        footCostEvidence(
            std::make_shared<task5c::Evidence>(i.root, "foot_cost_")),
        migratedProblem(task5c::footcost::installFootCosts(
            task5c::footcost::installCollisionBridge(
                task5c::footcost::installMomentBridge(
                    task5c::wavec::install(
                        task5c::installQuadraticCosts(
                            task5c::installJointLimits(
                                task5c::installPenalties(
                                    task5c::installFriction(
                                        task5c::install(interface,
                                                        mathematicalEvidence),
                                        interface, i.task, frictionEvidence),
                                    polynomialEvidence, relaxedEvidence),
                                interface, jointEvidence),
                            interface, terminalEvidence, runningEvidence,
                            nominalStateEvidence, nominalInputEvidence),
                        interface, preEvidence, stanceEvidence, swingEvidence),
                    interface, i.task, momentEvidence),
                interface, i.task, collisionEvidence),
            interface, footCostEvidence)),
        migratedInitializer(interface.getInitializer(), interface,
                            nominalInputEvidence),
        mpc(interface.mpcSettings(), interface.sqpSettings(), migratedProblem,
            migratedInitializer),
        log(std::make_shared<TimeLog>(i.root, i.heavy)),
        migrated(std::getenv("TASK5A_AUTHORITY") &&
                 std::string(std::getenv("TASK5A_AUTHORITY")) ==
                     "SIMULINK_REFERENCE") {
    {
      std::ifstream maps("/proc/self/maps");
      std::ofstream(i.root + "/loaded_process_maps.txt") << maps.rdbuf();
    }
    if (migrated) {
      external = std::make_shared<task5a::ExternalReferenceAdapter>(
          interface.getSwitchedModelReferenceManagerPtr(),
          [this](double t) {
            uint64_t tick = std::llround(t / .002);
            auto p = currentPacket;
            if (expectedSession && p.session != expectedSession)
              throw std::runtime_error(
                  "G: Reference and PlantState sessions disagree");
            if (p.tick != tick)
              throw std::runtime_error(
                  "G: Reference tick does not match actual solver observation");
            log->reference(t, p.filtered);
            return p;
          },
          i.root);
      mpc.getSolverPtr()->setReferenceManager(external);
    } else
      throw std::runtime_error("MEX requires SIMULINK_REFERENCE authority");
    mpc.getSolverPtr()->addSynchronizedModule(log);
    log->configure(interface.getPinocchioInterface(),
                   interface.getMpcRobotModel(),
                   interface.getSwitchedModelReferenceManagerPtr());
    task5b::exportConfiguration(interface, i.task, i.root);
    structure = std::make_shared<task5b::StructuralObserver>(interface, mpc,
                                                             log, i.root);
    mpc.getSolverPtr()->addSynchronizedModule(structure);
    std::ofstream(i.root + "/reference_authority.txt")
        << (migrated ? "SIMULINK_REFERENCE" : "LEGACY_CPP_REFERENCE") << "\n";
  }
/**
 * @brief 为控制调度器绑定数学来源记录、确定性仿真模式和时间事件回调。
 * 此处不启动线程，也不改变 MPC/SQP 参数。
 */
  void configure(WaveBControlTimeWBMpcController &c) {
    c.setMathematicsEvidence(stateEvidence, nominalInputEvidence);
    c.enableDeterministicSimulation();
    c.setTraceCallback([this](const char *e, double t, double d, double r) {
      log->trace(e, t, d, r);
    });
  }
};
