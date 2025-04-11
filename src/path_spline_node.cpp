#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <cmath>

class SplineSmootherNode : public rclcpp::Node {
public:
  SplineSmootherNode() : Node("spline_smoother_node") {
    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
      "/generated_path", 10,
      std::bind(&SplineSmootherNode::path_callback, this, std::placeholders::_1));

    pub_smoothed_path_ = this->create_publisher<nav_msgs::msg::Path>("/smoothed_path", 10);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_smoothed_path_;

  struct Coeffs {
    std::vector<double> a, b, c, d;
  };

  Coeffs computeSpline(const std::vector<double>& t, const std::vector<double>& y) {
    size_t n = t.size() - 1;
    std::vector<double> a(y.begin(), y.end());
    std::vector<double> b(n), d(n), h(n), alpha(n), c(n + 1), l(n + 1), mu(n + 1), z(n + 1);

    for (size_t i = 0; i < n; ++i)
      h[i] = t[i+1] - t[i];

    for (size_t i = 1; i < n; ++i)
      alpha[i] = (3.0/h[i]) * (a[i+1] - a[i]) - (3.0/h[i-1]) * (a[i] - a[i-1]);

    l[0] = 1.0; mu[0] = z[0] = 0.0;
    for (size_t i = 1; i < n; ++i) {
      l[i] = 2.0*(t[i+1] - t[i-1]) - h[i-1]*mu[i-1];
      mu[i] = h[i]/l[i];
      z[i] = (alpha[i] - h[i-1]*z[i-1]) / l[i];
    }

    l[n] = 1.0; z[n] = c[n] = 0.0;

    for (int j = n - 1; j >= 0; --j) {
      c[j] = z[j] - mu[j]*c[j+1];
      b[j] = (a[j+1] - a[j])/h[j] - h[j]*(c[j+1] + 2.0*c[j])/3.0;
      d[j] = (c[j+1] - c[j]) / (3.0*h[j]);
    }

    return {a, b, c, d};
  }

  double evalSpline(const Coeffs& coeffs, size_t i, double tau) {
    return coeffs.a[i]
         + coeffs.b[i] * tau
         + coeffs.c[i] * tau * tau
         + coeffs.d[i] * tau * tau * tau;
  }

  std::vector<geometry_msgs::msg::PoseStamped> downsamplePath(
    const std::vector<geometry_msgs::msg::PoseStamped>& input, float min_dist = 3.0f)
  {
    std::vector<geometry_msgs::msg::PoseStamped> result;
    if (input.empty()) return result;

    result.push_back(input[0]);
    auto last = input[0].pose.position;

    for (const auto& pose : input) {
      auto curr = pose.pose.position;
      float dx = curr.x - last.x;
      float dy = curr.y - last.y;
      float dist = std::sqrt(dx*dx + dy*dy);
      if (dist >= min_dist) {
        result.push_back(pose);
        last = curr;
      }
    }
    return result;
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    auto downsampled = downsamplePath(msg->poses, 5.0f);  // 기본 downsample 간격 5m
    size_t N = downsampled.size();
    if (N < 4) return;

    std::vector<double> t(N), x(N), y(N);
    for (size_t i = 0; i < N; ++i) {
      t[i] = static_cast<double>(i);
      x[i] = downsampled[i].pose.position.x;
      y[i] = downsampled[i].pose.position.y;
    }

    auto coeff_x = computeSpline(t, x);
    auto coeff_y = computeSpline(t, y);

    nav_msgs::msg::Path smooth_path;
    smooth_path.header = msg->header;

    double resolution = 0.05; // 5cm
    int max_points = 300;
    int count = 0;

    for (size_t i = 0; i < N - 1; ++i) {
      // 거리 기반으로 샘플 수 결정
      double dx = x[i+1] - x[i];
      double dy = y[i+1] - y[i];
      double segment_length = std::sqrt(dx*dx + dy*dy);
      int samples = std::max(1, static_cast<int>(segment_length / resolution));

      for (int j = 0; j < samples; ++j) {
        if (count >= max_points) break;

        double tau = static_cast<double>(j) / samples;
        double px = evalSpline(coeff_x, i, tau);
        double py = evalSpline(coeff_y, i, tau);
        double pz = downsampled[i].pose.position.z;

        geometry_msgs::msg::PoseStamped ps;
        ps.header = msg->header;
        ps.pose.position.x = px;
        ps.pose.position.y = py;
        ps.pose.position.z = pz;
        ps.pose.orientation.w = 1.0;

        smooth_path.poses.push_back(ps);
        count++;
      }

      if (count >= max_points) {
        break;
      }
    }

    pub_smoothed_path_->publish(smooth_path);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SplineSmootherNode>());
  rclcpp::shutdown();
  return 0;
}
