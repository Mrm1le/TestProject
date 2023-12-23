#pragma once

#include <atomic>
#include <memory>

#include "maf_interface/maf_worldmodel.h"
#include "msd/worldmodel/worldmodel_generator.h"
#include "mtaskflow/mtaskflow.hpp"

namespace msd_worldmodel {

class WorldmodelPubTask : public mtaskflow::FlowTask {
public:
  WorldmodelPubTask(
      const mtaskflow::FlowReceiver<ProcessedMapPtr> &processed_map_receiver,
      const mtaskflow::FlowReceiver<ObjectsInterfacePtr> &wm_obj_receiver)
      : processed_map_receiver_(processed_map_receiver),
        wm_obj_receiver_(wm_obj_receiver) {}

  void on_init() {}

  void on_running() {
    return;
    while (!processed_map_receiver_->empty()) {
      ProcessedMapPtr msg{};
      auto ret = processed_map_receiver_->pop_oldest(msg);
      if (!ret) {
        continue;
      }

      if (map_callback_) {
        map_callback_(msg);
      }
    }

    while (!wm_obj_receiver_->empty()) {
      ObjectsInterfacePtr msg{};
      auto ret = wm_obj_receiver_->pop_oldest(msg);
      if (!ret) {
        continue;
      }

      if (od_callback_) {
        od_callback_(msg);
      }
    }
  }

  void on_stoping() {}

  void on_terminate() {}

  void set_callback(MSDMapCallback callback) { map_callback_ = callback; }

  void set_callback(MSDObjectsCallback callback) { od_callback_ = callback; }

private:
  mtaskflow::FlowReceiver<ProcessedMapPtr> processed_map_receiver_;
  mtaskflow::FlowReceiver<ObjectsInterfacePtr> wm_obj_receiver_;
  MSDMapCallback map_callback_{};
  MSDObjectsCallback od_callback_{};
};

} // namespace msd_worldmodel
