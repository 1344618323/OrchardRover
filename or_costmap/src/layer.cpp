//
// Created by cxn on 2020/7/1.
//
#include "layer.h"

namespace or_costmap {

    Layer::Layer()
            : layered_costmap_(NULL), is_current_(false), is_enabled_(false), name_(), tf_(NULL) {}

    void Layer::Initialize(LayeredCostmap *parent, std::string name, tf::TransformListener *tf) {
        layered_costmap_ = parent;
        name_ = name;
        tf_ = tf;
        OnInitialize();
    }

    const std::vector<geometry_msgs::Point> &Layer::GetFootprint() const {
        return layered_costmap_->GetFootprint();
    }

} //namespace roborts_costmap