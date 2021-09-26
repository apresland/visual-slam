#include "mappoint.h"

void MapPoint::removeObservation(std::shared_ptr<Feature> feat) {
    std::unique_lock<std::mutex> lck(mutex_);
    for (auto iter = observations_.begin(); iter != observations_.end();
         iter++) {
        if (iter->lock() == feat) {
            observations_.erase(iter);
            feat->landmark_.reset();
            observed_times_--;
            break;
        }
    }
}
