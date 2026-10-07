#include "../../include/Detector/Guide_light.hpp"
#include "../../include/Filter/OneEuroFilter.hpp"

namespace GuideLight
{

GuideLight::GuideLight(cv::Scalar lower, cv::Scalar upper)
    : 
    next_id_(0), 
    frame_count_(0)
{
    EuroFilter = std::make_shared<OneEuroFilter>();

}


double GuideLight::calcu_circularity(const std::vector<cv::Point> &contour)
{
    double area = cv::contourArea(contour);
    std::cout << "area: " << area << std::endl;
    if (area < min_area_) 
        return -2.;
    double perimeter = cv::arcLength(contour, true);
    double circularity = (perimeter * perimeter) / (4*CV_PI*area);
    std::cout << "circularity: " << circularity << std::endl;
    if(circularity < 0.66 || circularity > 1.39)
        return -2.;

    return circularity;
}

std::vector<TrackedBlob> GuideLight::processFrame(const cv::Mat& frame) {

    frame_count_++;

    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv, lower_, upper_, mask);

    cv::dilate(mask, mask, kernel3); //膨胀
    cv::erode(mask, mask, kernel3); 


    cv::namedWindow("mask", cv::WINDOW_NORMAL);
    cv::imshow("mask", mask);

    // cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, {5,5}));
    // cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, {5,5}));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //------霍夫圆防误检------
    std::vector<cv::Vec3f> hough_circles;
    cv::HoughCircles(mask, hough_circles, cv::HOUGH_GRADIENT, 4, 50., 100., 7.5, 5., 50.);
    std::cout << "hough_circles.size():  " << hough_circles.size() << std::endl;
    //------霍夫圆防误检------

    std::cout << "contours.size(): " << contours.size() << std::endl;

    std::unordered_map<int, TrackedBlob> current_blobs_map;
    std::vector<int> need_erases;
    std::vector<TrackedBlob> current_blobs;

    std::sort(contours.begin(), contours.end(), 
      [this](const std::vector<cv::Point> &contour_a, const std::vector<cv::Point> &contour_b)
        { 
          return fabs(calcu_circularity(contour_a)  - 1) > 
                  fabs(calcu_circularity(contour_b) - 1); 
        });

    cv::drawContours(frame, contours, -1, cv::Scalar(0, 0, 255));
    for(auto hough_circle : hough_circles)
    {
        cv::Point2f hough_cir(hough_circle[0], hough_circle[1]);
        double radius = hough_circle[2];

        cv::circle(frame, hough_cir, radius, cv::Scalar(255, 0, 50), 2);

    }

    int map_id = -1;
    for (const auto& contour : contours) {
        map_id++;
        double area = cv::contourArea(contour);
        std::cout << "single_area: " << area << std::endl;

        // 面积
        if (area < min_area_ || area > max_area_) 
            continue;

        // 圆度
        double perimeter = cv::arcLength(contour, true);
        double circularity = (perimeter * perimeter) / (4*CV_PI*area);
        std::cout << "single_circularity: " << circularity << std::endl;
        if(circularity < 0.8 || circularity > 1.5)
            continue;

        // 由矩求质心
        cv::Moments M = cv::moments(contour);
        if (M.m00 == 0) 
            continue;
        cv::Point2f center(M.m10 / M.m00, M.m01 / M.m00);
        
        bool matched = false;
        for(size_t i = 0; i < hough_circles.size(); i++)
        {
            cv::Point2f hough_point(hough_circles[i][0], hough_circles[i][1]);
            if(cv::norm(hough_point - center) < 10.)
            {
                matched = true;
                break;
            }
        }
        // if(!matched && (circularity > 1.33 || circularity < 0.88))
        //     continue;
        
        float max_r = 0.;
        for(const auto& point : contour)
        {
            double r = cv::norm(cv::Point2f(center.x - point.x, center.y - point.y));
            max_r += r;
        }
        max_r = max_r / contour.size();



        // if(!init_Filter)
        // {
        //     EuroFilter->setParameter(center.x, 0., 0.5, 0.5, 0.5);
        //     init_Filter = true;
        // }
        // std::cout << "dt_: " << std::setprecision(5) << dt_ << std::endl;
        // center.x = EuroFilter->filter(center.x, dt_);
        // if(isnanf(center.x))
        //     init_Filter = false;
        current_blobs_map.emplace(
            map_id, 
            TrackedBlob{center, max_r, circularity, -1, frame_count_}
        );
    }

    for(int i = 0; i < current_blobs_map.size(); i++)
    {
        for(int j = i+1; j < current_blobs_map.size(); j++)
        {
            if(cv::norm(
                cv::Point2f(
                    fabs(current_blobs_map[i].center.x - current_blobs_map[j].center.x), 
                    fabs(current_blobs_map[i].center.y - current_blobs_map[j].center.y)
                )) < 300) //计算任意两个blob之间的距离并判断
            {
                // current_blobs_map.erase(i);
                // current_blobs_map.erase(j);
                need_erases.emplace_back(i);
                need_erases.emplace_back(j);
                // if(j == current_blobs_map.size()-1)
                // {
                //     need_erases.emplace_back(j);
                // }
            }
        }
    }
    for(const auto need_erase : need_erases)
    {
        current_blobs_map.erase(need_erase);
    }
    for(const auto current_blob_map : current_blobs_map)
    {
        current_blobs.emplace_back(current_blob_map.second);
    }

    assignIds(current_blobs);

    // // 清理长时间未见的旧blob
    // cleanupOldBlobs();

    prev_blobs_ = current_blobs;

    return current_blobs;
}

void GuideLight::drawBlob(cv::Mat& frame, const TrackedBlob& blob)
{
    if(blob.center.x == -1)
    {
        init_Filter = false;
        return;
    }
    // for (const auto& b : blob) {

        //cv::line加一条直线可以
        cv::line(frame, cv::Point(blob.center.x, 0), cv::Point(blob.center.x, 2048), cv::Scalar(255, 255, 255), 3);
        cv::circle(frame, blob.center, 2, cv::Scalar(150, 0, 255), 2);

        cv::putText(frame, "ID:" + std::to_string(blob.id) + "  Circularity: " + std::to_string(blob.circularity),

                    blob.center + cv::Point2f(10, 10),

                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(40, 0, 255), 1);
    //     break;
    // }
}

void GuideLight::drawBlobs(cv::Mat& frame, const std::vector<TrackedBlob>& blobs)
{
    if(blobs.empty())
    {
        init_Filter = false;
        return;
    }
    for (const auto& b : blobs) {

        //cv::line加一条直线可以
        // cv::line(frame, cv::Point(b.center.x, 0), cv::Point(b.center.x, 2048), cv::Scalar(255, 255, 255), 1);
        // cv::circle(frame, b.center, 2, cv::Scalar(150, 0, 255), 2);

        cv::putText(frame, "ID:" + std::to_string(b.id) + "  Circularity: " + std::to_string(b.circularity),

                    b.center + cv::Point2f(10, 10),

                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(40, 0, 255), 1);
        // break;
    }
}

cv::Point2f GuideLight::Filter(std::vector<TrackedBlob>& current)
{   
    cv::Point2f center;
    if(current.empty() || prev_blobs_.empty())
    {
        init_Filter = false;
        return cv::Point2f(-1., -1.);
    }
    
    center = current[0].center;

    if(init_Filter)
    {
        if(current[0].id != blobs_prev_lost_[0].id)
        {
            std::cout << "not  matched" << std::endl;
            if(abs(current[0].last_seen - blobs_prev_lost_[0].last_seen) > max_missing_frames_)
            {
                prev_blobs_.clear();
                blobs_prev_lost_.clear();
                init_Filter = false;
                return cv::Point2f(-1., -1.);
            }
            // blobs_prev_lost_ = prev_blobs_;
            center = blobs_prev_lost_[0].center;
        }
        else
        {
            blobs_prev_lost_ = prev_blobs_;
        }
    }

    if(!init_Filter)
    {
        EuroFilter->setParameter(center.x, 0., 0.5, 0.5, 0.5);
        init_Filter = true;
        blobs_prev_lost_ = prev_blobs_;
    }
    std::cout << "dt_: " << std::setprecision(5) << dt_ << std::endl;
    center.x = EuroFilter->filter(center.x, dt_);
    if(isnanf(center.x))
    {
        init_Filter = false;
        return cv::Point2f(-1., -1.);
    }

    // blobs_prev_lost_ = prev_blobs_;
    last_tracked_id = prev_blobs_[0].id;

    return center;
}

void GuideLight::assignIds(std::vector<TrackedBlob>& current) 
{
    std::vector<bool> matched_prev(prev_blobs_.size(), false);
    std::vector<bool> matched_curr(current.size(), false);

    for (size_t i = 0; i < current.size(); ++i) {
        float best_dist = 50.0f;
        int best_j = -1;
        for (size_t j = 0; j < prev_blobs_.size(); ++j) {
            if (matched_prev[j]) 
                continue;
            float dx = current[i].center.x - prev_blobs_[j].center.x;
            float dy = current[i].center.y - prev_blobs_[j].center.y;
            float dist = std::sqrt(dx*dx + dy*dy);
            if (dist < best_dist) {
                best_dist = dist;
                best_j = j;
            }
        }
        if (best_j != -1) {
            current[i].id = prev_blobs_[best_j].id;
            matched_prev[best_j] = true;
            matched_curr[i] = true;
        }

    }

    double max_id = -1;
    for(const auto prev_blob : prev_blobs_)
    {
        if(max_id == -1)
        {
            max_id = prev_blob.id;
            continue;
        }
        if(prev_blob.id > max_id)
            max_id = prev_blob.id;
    }

    std::vector<int> blobs_Ids;
    for(const auto prev_blob : prev_blobs_)
    {
        blobs_Ids.emplace_back(prev_blob.id);
    }

    // 为未匹配的blob分配新ID
    for (size_t i = 0; i < current.size(); ++i) {
        // int j = 0;
        if (!matched_curr[i]) {
            for(int j = 0; j < current.size(); ++j)
            {
                auto it = std::find(blobs_Ids.begin(), blobs_Ids.end(), j);
                if (it == blobs_Ids.end()) {
                    std::cout << "未找到！" << std::endl;
                    if(j > max_id)
                        current[i].id = next_id_++;
                    else
                        current[i].id = j;
                    continue;

                }
            }
            if(current[i].id == -1)
                current[i].id = next_id_++;
        }
    }

}

// void GuideLight::cleanupOldBlobs(std::vector<TrackedBlob>& current)
// {
//     for(const auto blob : current)
//     {
//         // for(const auto prev_blob : prev_blobs_)
//         // {
//             prev_blobs_.erase(
//                 std::remove_if(
//                     prev_blobs_.begin(),
//                     prev_blobs_.end(),
//                     [this, blob](const TrackedBlob& prev_blob) {
//                         return prev_blob.id == blob.id && 
//                             abs(blob.last_seen - prev_blob.last_seen) > max_missing_frames_;
//                     }
//                 ),
//                 prev_blobs_.end()
//             );
//         // }
//     }



// }



}