#pragma once
// 已加载设置和 OCP 项清单；独立于诊断计算工作区。
#include "extensions/TermRegistry.h"
#include <ocs2_sqp/SqpMpc.h>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <sstream>

namespace g1_controller::diagnostics {
template<class Problem>
inline std::string describeProblemConfiguration(Problem& i,const std::string&task){
 boost::property_tree::ptree raw,out;boost::property_tree::read_info(task,raw);out.add_child("source_config",raw);
 auto&s=i.sqpSettings();auto&m=i.mpcSettings();out.put("state_dim",i.getMpcRobotModel().getStateDim());out.put("input_dim",i.getMpcRobotModel().getInputDim());
 out.put("loaded.horizon",m.timeHorizon_);out.put("loaded.mpc_hz",m.mpcDesiredFrequency_);out.put("loaded.mrt_hz",m.mrtDesiredFrequency_);out.put("loaded.cold_start",m.coldStart_);
 #define FIELD(name) out.put("loaded.sqp." #name,s.name)
 FIELD(sqpIteration);FIELD(deltaTol);FIELD(costTol);FIELD(alpha_decay);FIELD(alpha_min);FIELD(gamma_c);FIELD(g_max);FIELD(g_min);FIELD(armijoFactor);FIELD(dt);FIELD(useFeedbackPolicy);FIELD(createValueFunction);FIELD(inequalityConstraintMu);FIELD(inequalityConstraintDelta);FIELD(projectStateInputEqualityConstraints);FIELD(extractProjectionMultiplier);FIELD(nThreads);FIELD(threadPriority);
 #undef FIELD
 out.put("loaded.sqp.integrator",ocs2::sensitivity_integrator::toString(s.integratorType));
 boost::property_tree::ptree names;for(auto&n:i.modelSettings().mpcModelJointNames){boost::property_tree::ptree item;item.put("",n);names.push_back({"",item});}out.add_child("joint_names_mpc_order",names);
 auto&ocp=i.getOptimalControlProblem();
 auto inventory=[&](const char*name,auto&collection){boost::property_tree::ptree rows;for(auto&[n,index]: ::humanoid::ocp_extensions::termNames(*collection)){boost::property_tree::ptree row;row.put("name",n);row.put("index",index);rows.push_back({"",row});}out.add_child(name,rows);};
 inventory("running_cost",ocp.costPtr);inventory("terminal_cost",ocp.finalCostPtr);inventory("state_soft",ocp.stateSoftConstraintPtr);inventory("input_soft",ocp.softConstraintPtr);inventory("hard_equality",ocp.equalityConstraintPtr);
 std::ostringstream text;boost::property_tree::write_json(text,out);return text.str();
}
} // namespace g1_controller::diagnostics
