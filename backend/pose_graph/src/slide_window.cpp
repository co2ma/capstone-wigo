#include "include/slide_window.h"

SlideWindow::SlideWindow(): f_manager{Rs} {}

bool SlideWindow::failureDetection(){
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if((tmp_P - last_P).norm() > 5)
        return true;

    if(abs(tmp_P.z() - last_P.z()) > 1)
        return true;
    
    return false;
}

void SlideWindow::addFeatureListBuf(const FeatureListPtr feature_list){
    m_estimator.lock();
    v_feature_per_frame_buf.push(v_feature_per_frame);
    m_estimator.unlock();
}

// @todo header 정보는 FeaturePerId에 통합하기
// Headers[frame_count] = header;

void SlideWindow::process(){
    while (true){
        m_estimator.lock();

        FeatureListPtr feature_list = nullptr;
        if(!feature_list_buf.empty()){
            feature_list = feature_list_buf.front();
            feature_list_buf.pop();
        }

        m_estimator.unlock();

        if(feature_list != nullptr){
            if (f_manager.addFeatureCheckParallax(frame_count, feature_list, 5))
                marginalization_flag = MARGIN_OLD;

            else
                marginalization_flag = MARGIN_SECOND_NEW;
        
            if(frame_count == WINDOW_SIZE){
                if (!failureDetection()) continue;

                if (marginalization_flag == MARGIN_OLD){
                    double t_0 = Headers[0];
                    back_R0 = Rs[0];
                    back_P0 = Ps[0];
            
                    if (frame_count == WINDOW_SIZE){
                        for (int i = 0; i < WINDOW_SIZE; i++){
                            Rs[i].swap(Rs[i + 1]);
                            Ps[i].swap(Ps[i + 1]);
                            Headers[i] = Headers[i + 1];
                        }
                        
                        Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
                        Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
                        Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
                        
                        Matrix3d R0, R1;
                        Vector3d P0, P1;
                        
                        R0 = back_R0 * ric;
                        R1 = Rs[0] * ric;
                    
                        P0 = back_P0 + back_R0 * tic;
                        P1 = Ps[0] + Rs[0] * tic;
                    
                        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
                    }
                }
            
                else {
                    if (frame_count == WINDOW_SIZE){
                        Ps[frame_count - 1] = Ps[frame_count];
                        Rs[frame_count - 1] = Rs[frame_count];
                        Headers[frame_count - 1] = Headers[frame_count];
                        
                        f_manager.removeFront(frame_count);
                    }
                }

                f_manager.removeFailures();
    
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];
            }
            
            else {
                frame_count++;
            }
        }
        
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}
