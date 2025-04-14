#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    cv::Point2f p0 = control_points[0];
    cv::Point2f p1 = control_points[1];
    cv::Point2f p2 = control_points[2];
    cv::Point2f p3 = control_points[3];

    cv::Point2f p01 = (t) * p0 + (1-t) * p1;
    cv::Point2f p12 = (t) * p1 + (1-t) * p2;
    cv::Point2f p23 = (t) * p2 + (1-t) * p3;

    cv::Point2f p012 = (t) * p01 + (1-t) * p12;
    cv::Point2f p123 = (t) * p12 + (1-t) * p23;

    cv::Point2f p0123 = (t) * p012 + (1-t) * p123;

    return p0123;

}

void draw_point_aa(cv::Mat& window, float x, float y, cv::Vec3b color) {
    int ix = static_cast<int>(x);
    int iy = static_cast<int>(y);

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            int px = ix + dx;
            int py = iy + dy;

            if (px >= 0 && px < window.cols && py >= 0 && py < window.rows) {
                float fx = x - px;
                float fy = y - py;
                float weight = std::exp(-(fx * fx + fy * fy) * 4.0f); // 可调节平滑程度

                cv::Vec3b& pix = window.at<cv::Vec3b>(py, px);
                for (int c = 0; c < 3; ++c) {
                    pix[c] = static_cast<uchar>(std::min(255.f, pix[c] + weight * color[c]));
                }
            }
        }
    }
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (float t = 0; t < 1; t += 0.001) {
        cv::Point2f p = recursive_bezier(control_points, t); 
        draw_point_aa(window, p.x, p.y, cv::Vec3b(0, 255, 0));
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
