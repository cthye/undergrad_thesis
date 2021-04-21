#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <visualization_msgs/Marker.h>
#include "geometry_msgs/Twist.h"
#include "laser_line_extraction/LineSegment.h"
#include "laser_line_extraction/LineSegmentList.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32.h"

struct Line {
    laser_line_extraction::LineSegment line_segment;
    float slope;  // 斜率
    float dist;   // 最近点距离
    float x;      // 最近点x坐标
    float y;      // 最近点y坐标

    inline float yIntercept() { return (-slope * x) + y; }
};

class LaserDetector {
   public:
    LaserDetector();

   private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Publisher green_marker_pub_;
    ros::Publisher blue_marker_pub_;

    ros::Publisher cmd_pub_;

    ros::Subscriber line_segments_sub_;

    int debug_viz_;
    std::string frame_id_;

    float slope_tolerance_;
    float max_slope_diff_;
    float max_dist_;
    float x_target_;
    float wheel_base_;
    float k_cmd_;

    void lineSegmentsCallback(
        const laser_line_extraction::LineSegmentList::ConstPtr &msg);
    void populateMarkerMsg(const std::vector<Line> &lines, char color);
};

LaserDetector::LaserDetector() : nh_(), private_nh_("~") {
    std::string cmd_topic;
    private_nh_.getParam("cmd_topic", cmd_topic);
    private_nh_.getParam("slope_tolerance", slope_tolerance_);
    private_nh_.getParam("max_slope_diff", max_slope_diff_);
    private_nh_.getParam("max_dist", max_dist_);
    private_nh_.getParam("x_target", x_target_);
    private_nh_.getParam("wheel_base", wheel_base_);
    private_nh_.getParam("k_cmd", k_cmd_);
    private_nh_.getParam("debug_viz", debug_viz_);
    private_nh_.getParam("frame_id", frame_id_);

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic.c_str(), 1);

    line_segments_sub_ = nh_.subscribe<laser_line_extraction::LineSegmentList>(
        "/line_segments", 10, &LaserDetector::lineSegmentsCallback, this);

    if (debug_viz_) {
        green_marker_pub_ =
            nh_.advertise<visualization_msgs::Marker>("/line_needed", 1);
        blue_marker_pub_ =
            nh_.advertise<visualization_msgs::Marker>("/line_target", 1);
    }
}

void LaserDetector::lineSegmentsCallback(
    const laser_line_extraction::LineSegmentList::ConstPtr &msg) {
    geometry_msgs::Twist cmd;
    if (msg->line_segments.size() < 2) {
        ROS_INFO_STREAM_THROTTLE(3, "too few lines: 0");
        cmd.angular.y = 1;  // 转为车道线控制
        cmd_pub_.publish(cmd);
        return;
    }
    
    //ROS_INFO_STREAM_THROTTLE(3, "SUCCESSFULLY GET LINES 0");

    std::vector<Line> lines;
    // 斜率筛选
    for (auto line_segment : msg->line_segments) {
        if (line_segment.start[0] < 0 &&
            line_segment.end[0] < 0) {  // 只需前方的直线
            continue;
        }
        if (line_segment.start[0] == line_segment.end[0]) {
            line_segment.start[0] += 1e-9;
        }
        float slope = (line_segment.end[1] - line_segment.start[1]) /
                      (line_segment.end[0] - line_segment.start[0]);
       
        //ROS_INFO_STREAM("SLOPE" << slope);
	if (fabs(slope) < slope_tolerance_) {
            Line line;
            line.line_segment = line_segment;
            line.slope = slope;

            float dist_start = line_segment.start[0] * line_segment.start[0] +
                               line_segment.start[1] * line_segment.start[1];
            float dist_end = line_segment.end[0] * line_segment.end[0] +
                             line_segment.end[1] * line_segment.end[1];
            if (dist_start < dist_end) {
                line.dist = dist_start;
                line.x = line_segment.start[0];
                line.y = line_segment.start[1];
            } else {
                line.dist = dist_end;
                line.x = line_segment.end[0];
                line.y = line_segment.end[1];
            }
            lines.push_back(line);
        }
    }
    
    //ROS_INFO_STREAM("LINES SIZE " << lines.size());
    
    if (debug_viz_) {
        populateMarkerMsg(lines, 'g');
    }

    if (lines.size() < 2) {
        ROS_INFO_STREAM_THROTTLE(3, "too few lines: 1");
        cmd.angular.y = 1;  // 转为车道线控制
        cmd_pub_.publish(cmd);
        return;
    }

    //ROS_INFO_STREAM_THROTTLE(3, "SUCCESSFULLY GET LINES");

    sort(lines.begin(), lines.end(),
         [](const Line &lhs, const Line &rhs) { return lhs.dist < rhs.dist; });
    // 垂线筛选
    Line line1 = lines[0];
    float y_intercept1 = line1.yIntercept();
    // ROS_DEBUG_STREAM("line1: \n" << line1);
    Line line2 = line1;
    int l2_index = 0;
    float min_dist = max_dist_;
    for (int i = 1; i < lines.size(); ++i) {
        float slope_diff = sqrt((lines[i].slope - line1.slope) *
                                (lines[i].slope - line1.slope));
        float slope_of_perpendicular =
            (lines[i].y - line1.y) / (lines[i].x - line1.x);
        // float diff = fabs(slope_of_perpendicular * line1.slope + 1);
        float dist = sqrt((lines[i].y - line1.y) * (lines[i].y - line1.y) +
                          (lines[i].x - line1.x) * (lines[i].x - line1.x));
        //ROS_INFO_STREAM("slope_diff[" << i << "]: " << slope_diff);
        ROS_INFO_STREAM("slope_diff < max_slope_diff_: " << (slope_diff < max_slope_diff_));
        // ROS_DEBUG_STREAM("diff[" << i << "]: " << diff);
        // ROS_INFO_STREAM("dist[" << i << "]: " << dist);
        // ROS_DEBUG_STREAM("lines[" << i << "]:\n" << lines[i]);

        // ROS_DEBUG_STREAM("diff < slope_product_tolerance_: " << (diff <
        // slope_product_tolerance_));
        // ROS_INFO_STREAM("y_intercept1 * y_intercept2 <= 0: " <<
        //  (y_intercept1 * lines[i].yIntercept() <= 0));
        // ROS_INFO_STREAM("dist(" << dist << ") < min_dist(" << min_dist << "): " << (dist < min_dist));
        if (slope_diff < max_slope_diff_ &&
            (lines[i].yIntercept() * y_intercept1 <= 0) && (dist < min_dist)) {
            l2_index = i;
            min_dist = dist;
            line2 = lines[i];
        }
    }

    if (l2_index == 0) {
        ROS_INFO_STREAM_THROTTLE(1, "no line pair");
        cmd.angular.y = 1;  // 转为车道线控制
        cmd_pub_.publish(cmd);
        return;
    }

    ROS_INFO_STREAM("SUCCESSFULLY GET LINE PAIR");

    // 计算目标点
    float mid_x = (line1.x + line2.x) / 2;
    float mid_y = (line1.y + line2.y) / 2;
    float slope = (line1.slope + line2.slope) / 2;
    float y_target = slope * (x_target_ - mid_x) + mid_y;

    // ROS_DEBUG_STREAM("slope1: " << line1.slope << ", slope2: " <<
    // line2.slope);
    // ROS_DEBUG_STREAM("target point: (" << x_target_ << ", " << y_target <<
    // ")");

    float steer_angle = atan(2 * wheel_base_ * (-y_target) /
                             (x_target_ * x_target_ + y_target * y_target));
    cmd.angular.z = - k_cmd_ * steer_angle;
    cmd.linear.x = 0.2;
    cmd_pub_.publish(cmd);

    if (debug_viz_) {
        populateMarkerMsg({line1, line2}, 'b');
    }
}

void LaserDetector::populateMarkerMsg(const std::vector<Line> &lines, char color) {
    visualization_msgs::Marker marker_msg;
    ros::Publisher *marker_pub;
    marker_msg.ns = "laser_detection";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::LINE_LIST;
    marker_msg.scale.x = 0.1;
    //marker_msg.pose.orientation.w = 1.0;
    if (color == 'g') {
        marker_msg.color.r = 0.0;
        marker_msg.color.g = 1.0;
        marker_msg.color.b = 0.0;
        marker_pub = &green_marker_pub_;
    }
    if (color == 'b') {
        marker_msg.color.r = 0.0;
        marker_msg.color.g = 0.0;
        marker_msg.color.b = 1.0;
        marker_pub = &blue_marker_pub_;
    }
    marker_msg.color.a = 1.0;
    for (Line line : lines) {
        geometry_msgs::Point p_start;
        p_start.x = line.line_segment.start[0];
        p_start.y = line.line_segment.start[1];
        p_start.z = 0;
        marker_msg.points.push_back(p_start);
        geometry_msgs::Point p_end;
        p_end.x = line.line_segment.end[0];
        p_end.y = line.line_segment.end[1];
        p_end.z = 0;
        marker_msg.points.push_back(p_end);
    }
    marker_msg.header.frame_id = frame_id_;
    marker_msg.header.stamp = ros::Time::now();

    marker_pub->publish(marker_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_detection");
    LaserDetector detector;
    ros::spin();

    return 0;
}
