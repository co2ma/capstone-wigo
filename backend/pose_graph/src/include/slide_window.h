#pragma once

#include <map>
#include <queue>
#include <mutex>
#include <thread>

#include "parameters.h"
#include "feature_manager.h"
#include "../utility/utility.h"
#include "../utility/tic_toc.h"

#include <eigen3/Eigen/Core>

class SlideWindow {
  public:
    SlideWindow();
    // ~SlideWindow();

    void addFeatureListBuf(const FeatureListPtr feature_list);

    void process();
    bool failureDetection();

  public:
    std::mutex m_estimator;
   
    Matrix3d ric;
    Vector3d tic;

    enum MarginalizationFlag {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };
    
    MarginalizationFlag marginalization_flag;
    
    int frame_count;
    int loop_window_index;
    double initial_timestamp;

    FeatureManager f_manager;

    Vector3d Ps[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    double Headers[(WINDOW_SIZE + 1)];

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;

    queue<FeatureListPtr> feature_list_buf;
};
