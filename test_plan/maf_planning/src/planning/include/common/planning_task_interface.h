#ifndef MSQUARE_DECISION_PLANNING_COMMON_PLANNING_TASK_INTERFACE_H_
#define MSQUARE_DECISION_PLANNING_COMMON_PLANNING_TASK_INTERFACE_H_

#include "mtaskflow/mtaskflow.hpp"
#include "pnc.h"

namespace msquare {

class PlanningTaskInterface : public mtaskflow::FlowTask {
public:
  MSD_API virtual ~PlanningTaskInterface() = default;
  MSD_API virtual void set_callback(MSDPlanningOutputCallback callback) = 0;
  MSD_API virtual void set_callback(MSDPlanningLdpOutputCallback callback) = 0;
  MSD_API virtual void set_callback(MSDPlanningTriggerCallback callback) = 0;
  MSD_API virtual void
  set_callback(std::function<void(const maf_planning::SBPRequest &sbp_request)>
                   &callback) = 0;
  MSD_API virtual void set_callback(MSDPlanningInfoCallback callback) = 0;
  MSD_API virtual void set_callback(MSDMdebugCallback callback) = 0;
  MSD_API virtual void set_sync_callback(NPPHeaderLoggerCallback callback) = 0;

  MSD_API virtual void reset() = 0;
};

} // namespace msquare

#endif
