#include "../include/Aimer.hpp"
#include <iostream>
// #include <chrono>


int s_min = 44;
int s_max = 255;
int v_min = 23;
int v_max = 255;

void on_trackbar(int, void*)
{
    lower_ = cv::Scalar(60, s_min, v_min);
    upper_ = cv::Scalar(140, s_max, v_max);
}

Aimer::Aimer()
{
    int i = 0;
    // s_min = 40;
    // s_max = 255;
    // v_min = 40;
    // v_max = 255;
    lower_ = cv::Scalar(60, s_min, v_min);
    upper_ = cv::Scalar(140, s_max, v_max);

    // —— 引导灯链路（上相机：迈德，index 0）——
    MVCamera_ = std::make_shared<MVCameraNode>();
    GuideLight_ = std::make_shared<GuideLight::GuideLight>(lower_, upper_);

    // —— 飞镖镖头链路（下相机：海康，index 1）——
    HikCamera_ = std::make_shared<HikCameraNode>(0);
    DartDetect_ = std::make_shared<DartDetect::DartDetector>(dart_ref_x_);

    // —— 共用串口 ——
    Serial_ = std::make_shared<SerialNode>();

    lost_time_thr_ = 1.2;

   // cv::namedWindow("param panel", cv::WINDOW_NORMAL);

    //cv::namedWindow("image_", cv::WINDOW_NORMAL);
    Aimer_thread = true;
    // std::cout << "1111111" << std::endl;


    while(Aimer_thread && MVCamera_->camera_thread_)
    {
        // cv::createTrackbar("bao he du_min", "param panel", &s_min, 255);
        // cv::createTrackbar("ming liang du_min", "param panel", &v_min, 255);
        // cv::createTrackbar("bao he du_max", "param panel", &s_max, 255);
        // cv::createTrackbar("ming liang du_max", "param panel", &v_max, 255);
        // lower_ = cv::Scalar(60, s_min, v_min);
        // upper_ = cv::Scalar(140, s_max, v_max);



        DateStruct data;
        data.Flag = 0xFE;
        data.c_x = 0.;
        data.lv_x = 0.;
        data.dart_x = 0.f;
        data.dart_state = 0;
        data.End = 0xFF;

        auto prev = std::chrono::high_resolution_clock::now();

        // bool lost = true;

        if(!target_init_)
        {
            last_tracked_time_ = std::chrono::high_resolution_clock::now();
        }
        auto lost_duration = std::chrono::duration_cast<std::chrono::microseconds>(prev - last_tracked_time_);
        double lost_ms = lost_duration.count() / 1000.0; // 微秒转毫秒
        bool lost = (lost_ms > lost_time_thr_ * 1000.);
        std::cout << std::fixed << std::setprecision(2) << "lost_time: " << lost_ms << " ms" << std::endl;

        if(!target_init_)
            lost = true;


        // std::cout << "image_got_completed: " << MVCamera_->image_got_completed << std::endl;
        cv::waitKey(1);

        // ============================================================
        // 飞镖镖头链路（下相机：海康）—— 每帧独立运行，不依赖引导灯
        // ============================================================
        if (HikCamera_->type != 0 && HikCamera_->image_got_completed && HikCamera_->image_data_.data())
        {
            std::unique_lock<std::mutex> dart_lock(HikCamera_->capture_mutex_);
            dart_image_ = cv::Mat(
                HikCamera_->h,
                HikCamera_->w,
                HikCamera_->type,
                HikCamera_->image_data_.data()
            ).clone();
            HikCamera_->image_got_completed = false;
            dart_lock.unlock();

            // 落点判定与画面标注都在 processFrame 内完成
            DartDetect_->processFrame(dart_image_);
            if (DartDetect_->hasLanded()) {
                float land_x = DartDetect_->landingPoint().x;
                data.dart_x = land_x;
                data.dart_state = 2;
                dart_correction_ =
                    (dart_ref_x_ - land_x) * dart_correction_gain_;
            } else {
                data.dart_x = 0.f;
                data.dart_state = 0;
                dart_correction_ = 0.f;
            }
        }
       
        if(MVCamera_->type != 0 && MVCamera_->image_got_completed && MVCamera_->image_data_.data())
        {
            
            std::unique_lock<std::mutex> lock(MVCamera_->capture_mutex_);
            
            image_ = cv::Mat(
                MVCamera_->h, 
                MVCamera_->w, 
                MVCamera_->type, 
                MVCamera_->image_data_.data()
            ).clone();
            std::cout << "Aimer: " << ++i << std::endl;

            // MVCamera_->image_data_.clear();
            MVCamera_->image_got_completed = false;

        }
        else 
        {
            // data.target_id = static_cast<uint8_t>(last_target_);
            // if(lost && !target_init_)
            //     data.target_id = static_cast<uint8_t>(TARGET::OutPost);
            // else if(lost)
            // {
            //     if(last_target_ == TARGET::OutPost)
            //         data.target_id = static_cast<uint8_t>(TARGET::Base);
            //     if(last_target_ == TARGET::Base)
            //         data.target_id = static_cast<uint8_t>(TARGET::OutPost);
            // }

            // try
            // {
            //     Serial_->sendData(data);
            // }
            // catch(const std::exception& e)
            // {
            //     std::cerr << e.what() << '\n';
            //     continue;
            // }
            
            continue;
        }

        std::vector<GuideLight::TrackedBlob> blobs = GuideLight_->processFrame(image_);
        GuideLight_->drawBlobs(image_, blobs);

        // // if(blobs.empty())
        // //     GuideLight_->init_Filter = false;
        cv::Point2f center = GuideLight_->Filter(blobs);


        // DateStruct data;
        // data.Flag = 0xFE;
        // data.c_x = 0.;
        // data.lv_x = 0.;
        // data.End = 0xFF;


        if(center.x == -1 || isnan(center.x))
        {
            data.target_id = static_cast<uint8_t>(last_target_);
            if(lost && !target_init_)
                data.target_id = static_cast<uint8_t>(TARGET::OutPost);
            else if(lost)
            {
                if(last_target_ == TARGET::OutPost)
                    data.target_id = static_cast<uint8_t>(TARGET::Base);
                if(last_target_ == TARGET::Base)
                    data.target_id = static_cast<uint8_t>(TARGET::OutPost);
            }

            cv::imshow("image_", image_);
            try
            {
                Serial_->sendData(data);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                continue;
            }
            continue;
            // center.x = 0.;
            // center.y = 0.;
        }
        GuideLight::TrackedBlob blob = blobs[0];
        blob.center.x = center.x;

        GuideLight_->drawBlob(image_, blob);

        cv::imshow("image_", image_);

        if(blobs.empty() || center.x == -1 || isnan(center.x))
        {
            data.target_id = static_cast<uint8_t>(last_target_);
            if(lost && !target_init_)
                data.target_id = static_cast<uint8_t>(TARGET::OutPost);
            else if(lost)
            {
                if(last_target_ == TARGET::OutPost)
                    data.target_id = static_cast<uint8_t>(TARGET::Base);
                if(last_target_ == TARGET::Base)
                    data.target_id = static_cast<uint8_t>(TARGET::OutPost);
            }

            try
            {
                Serial_->sendData(data);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                continue;
            }
            continue;
            // center.x = 0.;
            // center.y = 0.;
        }

        double dis = 6883.65854*0.0623/(blob.max_r * 2.);
        std::cout << "dis:  " << dis << std::endl;
        std::cout << "max_r:  " << blob.max_r << std::endl;
        // DateStruct data;

        // data.Flag = 0xFE;
        // 预瞄基准 = 画面中心 + 镖头落点闭环修正量（上位机融合）
        // 下位机仍按 c_x - lv_x 算偏差，等效于把预瞄点整体平移 Δx
        data.c_x = 1224 + dart_correction_;
        data.lv_x = center.x;
        if(dis > 20)
        {
            target_ = TARGET::Base;
        }
        else
        {
            target_ = TARGET::OutPost;
        }
        data.target_id = static_cast<uint8_t>(target_);
        // data.End = 0xFF;

        if(!target_init_)
        {
            target_init_ = true;
        }
        last_target_ = target_;

        try
        {
            Serial_->sendData(data);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            continue;
        }
        

        last_tracked_time_ = std::chrono::high_resolution_clock::now();

        auto post = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(post - prev);
        double cost_ms = duration.count() / 1000.0; // 微秒转毫秒

        std::cout << std::fixed << std::setprecision(2) << "Detector_cost: " << cost_ms << " ms" << std::endl;


        GuideLight_->dt_ = cost_ms / 1000.; // 毫秒转秒
        
    }

}

// Aimer::~Aimer() 
// {
//     Aimer_thread = false;
// }



int main(int argc, char * argv[])
{

    Aimer aimer;


    return 0;
}


