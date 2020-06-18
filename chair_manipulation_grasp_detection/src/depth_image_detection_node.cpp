#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointIndices = pcl::PointIndices;
using SurfaceNormals = pcl::PointCloud<pcl::Normal>;
using SearchMethod = pcl::search::KdTree<pcl::PointXYZ>;

constexpr auto node_name = "depth_image_detection";

double deg2rad(double deg)
{
    return (deg / 180) * M_PI;
}

template<typename PointT>
PointT depth_pixel_to_3D_point(int u,
                               int v,
                               double Z,
                               const sensor_msgs::CameraInfo &camera_info)
{
    // Intrinsic camera matrix for the raw (distorted) images
    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    double fx = camera_info.K[0];
    double fy = camera_info.K[4];
    double cx = camera_info.K[2];
    double cy = camera_info.K[5];

    double X = (double(u) - cx) / fx * Z;
    double Y = (double(v) - cy) / fy * Z;
    return PointT(X, Y, Z);
}

void compute_horizontal_pixel_count_histogram(const cv::Mat &image, cv::Mat &hist)
{
    hist = cv::Mat::zeros(image.rows, 1, CV_64FC1);
    for (int row = 0; row < image.rows; row++)
    {
        int pixel_count = 0;
        for (int col = 0; col < image.cols; col++)
        {
            if (image.at<std::uint8_t>(row, col) > 0)
                pixel_count++;
        }
        hist.at<double>(row, 0) = pixel_count;
    }
}

void compute_vertical_pixel_count_histogram(const cv::Mat &image, cv::Mat &hist)
{
    hist = cv::Mat::zeros(image.cols, 1, CV_64FC1);
    for (int col = 0; col < image.cols; col++)
    {
        int pixel_count = 0;
        for (int row = 0; row < image.rows; row++)
        {
            if (image.at<std::uint8_t>(row, col) > 0)
                pixel_count++;
        }
        hist.at<double>(col, 0) = pixel_count;
    }
}

void save_column_vector(const cv::Mat &hist, const std::string &filename)
{
    std::ofstream file{filename};
    for (int row = 0; row < hist.rows; row++)
    {
        file << (int) hist.at<double>(row, 0) << "\n";
    }
}

void non_maximum_suppression(const cv::Mat &input, cv::Mat &output, double threshold, bool collectMin = false)
{
    output = cv::Mat::zeros(input.size(), CV_64FC1);
    for (int row = 1; row < input.rows - 1; row++)
    {
        auto prev = input.at<double>(row - 1, 0);
        auto curr = input.at<double>(row, 0);
        auto next = input.at<double>(row + 1, 0);

        if (std::abs(curr) < threshold)
        {
            continue;
        }

        if ((curr > prev && curr > next) ||
            (collectMin && curr < prev && curr < next))
        {
            output.at<double>(row, 0) = 1.;
        }
    }
}

struct HistogramSegment
{
    int start_position;
    int end_position;
    double average;

    HistogramSegment(int start_position, int end_position, double average)
            : start_position(start_position),
              end_position(end_position),
              average(average)
    {}
};

void extract_histogram_segments(const cv::Mat &hist,
                                const cv::Mat &peak_indicators,
                                std::vector<HistogramSegment> &segments)
{
    int rows = hist.rows;
    int start_position = 0;
    double sum = 0;

    for (int row = 0; row < rows; row++)
    {
        sum += hist.at<double>(row, 0);

        if (peak_indicators.at<double>(row, 0) > 0)
        {
            int end_position = row;
            int n = end_position - start_position + 1;
            double average = sum / n;
            segments.emplace_back(start_position, end_position, average);
            sum = 0;
            start_position = row + 1;
        }
    }

    int end_position = rows - 1;
    if (segments.empty() || segments.back().end_position != end_position)
    {
        int n = end_position - start_position + 1;
        double average = sum / n;
        segments.emplace_back(start_position, end_position, average);
    }
}

template<typename Scalar>
void mat_to_point_vector(const cv::Mat &mat, std::vector<cv::Point> &points)
{
    for (int row = 0; row < mat.rows; row++)
    {
        for (int col = 0; col < mat.cols; col++)
        {
            if (mat.at<Scalar>(row, col) > 0)
            {
                points.emplace_back(col, row);
            }
        }
    }
}

cv::RotatedRect find_seat_plate_rect(const cv::Mat &image, int y_min, int y_max)
{
    int width = image.size().width;
    int height = y_max - y_min + 1;
    auto crop = image(cv::Rect{0, y_min, width, height});
    std::vector<cv::Point> points;
    mat_to_point_vector<std::uint8_t>(crop, points);
    auto rect = cv::minAreaRect(points);
    cv::Point2f new_center{rect.center.x, rect.center.y + float(y_min)};
    return cv::RotatedRect{new_center, rect.size, rect.angle};
}

void draw_rotated_rect(const cv::Mat &image, const cv::RotatedRect &rect, const cv::Scalar &color)
{
    cv::Point2f vertices[4];
    rect.points(vertices);
    for (int i = 0; i < 4; i++)
    {
        cv::line(image, vertices[i], vertices[(i + 1) % 4], color, 2);
    }
}

double angle_from_rotated_rect(const cv::RotatedRect &rect)
{
    // Convert weird RotatedRect angle
    if (rect.size.width > rect.size.height)
    {
        return deg2rad(-rect.angle);
    }
    return deg2rad(rect.angle + 90);
}

cv::Point2i compute_grasp_pixel_coords(const cv::RotatedRect &plate_rect, double seat_plate_horizontal_position)
{
    double length = plate_rect.size.width > plate_rect.size.height ? plate_rect.size.width : plate_rect.size.height;
    double angle = angle_from_rotated_rect(plate_rect);

    double cx = plate_rect.center.x;
    double cy = plate_rect.center.y;
    int x = int(cx + (seat_plate_horizontal_position - 0.5) * length * std::cos(angle));
    int y = int(cy + (seat_plate_horizontal_position - 0.5) * length * std::sin(angle));
    return cv::Point2i{x, y};
}

bool in_roi(const cv::Point2i &coords, const cv::Mat &roi)
{
    return coords.x >= 0 &&
           coords.x < roi.cols &&
           coords.y >= 0 &&
           coords.y < roi.rows &&
           roi.at<std::uint8_t>(coords) > 0;
}

bool compute_grasp_position(const cv::Point2i &grasp_pixel_coords,
                            const cv::Mat &depth_image,
                            const cv::Mat &image_roi,
                            const sensor_msgs::CameraInfo &camera_info,
                            Eigen::Vector3d &grasp_position)
{
    // If the pixel at the given coordinates is in the region of interest, return that right away
    double depth = depth_image.at<double>(grasp_pixel_coords);
    if (in_roi(grasp_pixel_coords, image_roi))
    {
        grasp_position = depth_pixel_to_3D_point<Eigen::Vector3d>(
                grasp_pixel_coords.x, grasp_pixel_coords.y, depth, camera_info);
        return true;
    }

    // Otherwise, return the center point of the surrounding pixels
    Eigen::Vector3d center{0., 0., 0.};
    static const std::vector<cv::Point2i> coords_offsets = {
            cv::Point2i{-1, -1}, cv::Point2i{-1, 0}, cv::Point2i{-1, 1},
            cv::Point2i{0, -1}, cv::Point2i{0, 1},
            cv::Point2i{1, -1}, cv::Point2i{1, 0}, cv::Point2i{1, 1}};
    std::size_t num_valid_neighbors = 0;
    for (const auto &offset : coords_offsets)
    {
        auto coords = cv::Point2i{grasp_pixel_coords.x + offset.x,
                                  grasp_pixel_coords.y + offset.y};
        if (!in_roi(coords, image_roi))
        {
            continue;
        }
        depth = depth_image.at<double>(coords);
        auto point = depth_pixel_to_3D_point<Eigen::Vector3d>(coords.x, coords.y, depth, camera_info);
        center += point;
        num_valid_neighbors++;
    }

    if (num_valid_neighbors == 0)
    {
        return false;
    }
    grasp_position = center / num_valid_neighbors;
    return true;
}

Eigen::Vector3d compute_surface_normal(const Eigen::Vector3d &position,
                                       const cv::Mat &depth_image,
                                       const cv::Mat &image_roi,
                                       const sensor_msgs::CameraInfo &camera_info,
                                       double normal_radius)
{
    // Retrieve the search surface which is all points that are in the region of interest
    PointCloud::Ptr search_surface{new PointCloud};
    for (int col = 0; col < depth_image.cols; col++)
    {
        for (int row = 0; row < depth_image.rows; row++)
        {
            cv::Point2i coords{col, row};
            if (in_roi(coords, image_roi))
            {
                auto depth = depth_image.at<double>(coords);
                auto point = depth_pixel_to_3D_point<pcl::PointXYZ>(col, row, depth, camera_info);
                search_surface->push_back(point);
            }
        }
    }

    // Create a point cloud with a single point containing the requested position
    PointCloud::Ptr position_cloud{new PointCloud};
    position_cloud->push_back(pcl::PointXYZ{
            float(position.x()), float(position.y()), float(position.z())});

    SurfaceNormals::Ptr normals{new SurfaceNormals};
    SearchMethod::Ptr search_method{new SearchMethod};

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> estimation;
    estimation.setInputCloud(position_cloud);
    estimation.setSearchSurface(search_surface);
    estimation.setSearchMethod(search_method);
    estimation.setRadiusSearch(normal_radius);
    estimation.compute(*normals);

    auto normal = (*normals)[0];
    return Eigen::Vector3d{double(normal.normal_x),
                           double(normal.normal_y),
                           double(normal.normal_z)};
}

Eigen::Quaterniond compute_frame_rotation(const Eigen::Vector3d &frame_axis, double axis_angle)
{
    // This function computes the rotation quaternion from the parent frame to a child frame which is defined by an axis
    // and an angle (relative to the parent's frame).
    // In order to do so, we have to compute the roll and pitch angles of the axis vector first.
    // Then we consecutively apply intrinsic roll (x-axis), pitch (y-axis) and yaw (z-axis) rotations where
    // the yaw angle is the angle of the child frame's axis.
    auto roll = std::atan2(-frame_axis.y(), frame_axis.z());
    auto pitch = std::asin(-frame_axis.x());

    // Use tf2::Quaternion here because Eigen does not implement euler angles
    tf2::Quaternion q;
    q.setRPY(roll, pitch, axis_angle);
    return Eigen::Quaterniond{q.w(), q.x(), q.y(), q.z()};
}

int main(int argc, char *argv[])
{
    // Initialization
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    tf2_ros::TransformBroadcaster transform_broadcaster;

    // Parameters
    auto depth_image_topic = priv_nh.param<std::string>("depth_image_topic", "kinect/depth/image_raw");
    auto camera_info_topic = priv_nh.param<std::string>("camera_info_topic", "kinect/depth/camera_info");
    auto grasp_frame = priv_nh.param<std::string>("grasp_frame", "grasp");
    auto debug_output_dir = priv_nh.param<std::string>("debug_output_dir", "/tmp");
    auto generate_debug_output = priv_nh.param<bool>("generate_debug_output", false);
    auto show_visualization = priv_nh.param<bool>("show_visualization", false);
    auto min_depth_tolerance = priv_nh.param<double>("min_depth_tolerance",
                                                     std::numeric_limits<double>::infinity());
    auto roi_x = priv_nh.param<int>("roi_x", 0);
    auto roi_y = priv_nh.param<int>("roi_y", 0);
    auto roi_width = priv_nh.param<int>("roi_width", 640);
    auto roi_height = priv_nh.param<int>("roi_height", 480);
    auto gaussian_ksize = priv_nh.param<int>("gaussian_ksize", 15);
    auto differentiator = priv_nh.param<std::string>("differentiator", "laplacian");
    auto differentiator_ksize = priv_nh.param<int>("differentiator_ksize", 3);
    auto nms_threshold = priv_nh.param<double>("nms_threshold", 1.);
    auto seat_plate_horizontal_position = priv_nh.param<double>("seat_plate_horizontal_position", 0.3);
    auto normal_radius = priv_nh.param<double>("normal_radius", 0.05);

    // Create a window for visualization
    const std::string window_name = "Depth image detection";
    if (show_visualization)
    {
        cv::namedWindow(window_name);
    }

    while (ros::ok())
    {
        // Listen for depth image
        std::string topic = nh.resolveName(depth_image_topic);
        ROS_INFO_STREAM_NAMED(node_name, "Cloud service called; waiting for an Image on topic " << topic);
        auto image_msg = ros::topic::waitForMessage<sensor_msgs::Image>(topic, nh);

        // Convert image to OpenCV matrix
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_64FC1);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR_STREAM_NAMED(node_name, "cv_bridge exception: " << e.what());
            return -1;
        }
        auto depth_image = cv_ptr->image;
        auto camera_frame = image_msg->header.frame_id;

        // Get camera info
        topic = nh.resolveName(camera_info_topic);
        auto camera_info_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(topic, nh);

        // Extract min and max depth
        double min_depth, max_depth;
        cv::minMaxLoc(depth_image, &min_depth, &max_depth);

        // Truncate depth
        cv::Mat image_threshold;
        cv::inRange(depth_image,
                    cv::Scalar{min_depth - min_depth_tolerance},
                    cv::Scalar{min_depth + min_depth_tolerance},
                    image_threshold);

        // Mask region of interest
        cv::Mat image_roi = cv::Mat::zeros(image_threshold.size(), image_threshold.type());;
        cv::Mat roi_mask = cv::Mat::zeros(image_threshold.size(), image_threshold.type());
        cv::rectangle(roi_mask, cv::Rect{roi_x, roi_y, roi_width, roi_height}, cv::Scalar{255}, cv::FILLED, 8, 0);
        image_threshold.copyTo(image_roi, roi_mask);

        // Compute horizontal and vertical pixel count histograms
        cv::Mat horizontal_hist;
        cv::Mat vertical_hist;
        compute_horizontal_pixel_count_histogram(image_roi, horizontal_hist);
        compute_vertical_pixel_count_histogram(image_roi, vertical_hist);

        // Apply blur before computing gradients
        cv::Mat horizontal_hist_blurred;
        cv::Mat vertical_hist_blurred;
        cv::GaussianBlur(
                horizontal_hist, horizontal_hist_blurred, cv::Size(1, gaussian_ksize), 0., 0., cv::BORDER_DEFAULT);
        cv::GaussianBlur(
                vertical_hist, vertical_hist_blurred, cv::Size(1, gaussian_ksize), 0., 0., cv::BORDER_DEFAULT);

        // Compute the gradients of the histograms
        cv::Mat horizontal_hist_gradient;
        cv::Mat vertical_hist_gradient;
        if (differentiator == "sobel")
        {
            cv::Sobel(horizontal_hist_blurred, horizontal_hist_gradient, -1, 0, 1, differentiator_ksize, 1., 0.,
                      cv::BORDER_REPLICATE);
            cv::Sobel(vertical_hist_blurred, vertical_hist_gradient, -1, 0, 1, differentiator_ksize, 1., 0.,
                      cv::BORDER_REPLICATE);
        }
        else if (differentiator == "laplacian")
        {
            cv::Laplacian(horizontal_hist_blurred, horizontal_hist_gradient, -1, differentiator_ksize, 1., 0.,
                          cv::BORDER_REPLICATE);
            cv::Laplacian(vertical_hist_blurred, vertical_hist_gradient, -1, differentiator_ksize, 1., 0.,
                          cv::BORDER_REPLICATE);
        }
        else
        {
            ROS_ERROR_STREAM_NAMED(node_name, "No such differentiator '" << differentiator << "'.");
            return -1;
        }

        // Apply non-maximum-suppression to get start and end positions of each peak in the histograms
        cv::Mat horizontal_peak_indicators;
        cv::Mat vertical_peak_indicators;
        non_maximum_suppression(horizontal_hist_gradient, horizontal_peak_indicators, nms_threshold);
        non_maximum_suppression(vertical_hist_gradient, vertical_peak_indicators, nms_threshold);

        // Extract segments from the histogram containing peaks and non-peaks
        std::vector<HistogramSegment> segments;
        extract_histogram_segments(horizontal_hist, horizontal_peak_indicators, segments);

        // If we detect less than three segments, that means that the chair does not have a graspable seat plate
        if (segments.size() < 3)
        {
            ROS_WARN_NAMED(node_name, "Not able to find the seat plate!");
            continue;
        }

        // The segment with highest average value contains the seat plate
        auto seat_plate_segment = std::max_element(
                segments.begin(), segments.end(), [&](const auto &lhs, const auto &rhs)
                {
                    return lhs.average < rhs.average;
                });

        // Fit a rectangle to the determined vertical area of the seat plate in the image
        cv::RotatedRect plate_rect = find_seat_plate_rect(image_roi, seat_plate_segment->start_position,
                                                          seat_plate_segment->end_position);

        // Get the grasp pixel coordinates from the rectangle.
        // seat_plate_horizontal_position represents the percentage of positional offset
        // along the rectangle's horizontal axis (from the left).
        auto grasp_pixel_coords = compute_grasp_pixel_coords(plate_rect, seat_plate_horizontal_position);
        ROS_INFO_STREAM_NAMED(node_name, "Grasp pixel coordinates: ("
                << grasp_pixel_coords.x << "," << grasp_pixel_coords.y << ")");

        // Check the potential edge case where the computed grasp pixel coordinates are not in the image anymore.
        cv::Rect image_rect = cv::Rect{0, 0, depth_image.cols, depth_image.rows};
        if (!image_rect.contains(grasp_pixel_coords))
        {
            ROS_WARN_NAMED(node_name, "Grasp pixel coords are out of image bounds!");
            continue;
        }

        // Get the 3D position of the grasp. It can be the case that some depth values are NaN.
        // These pixels are filtered out in the image_roi.
        // We have to check for the case that our computed grasp pixel coordinate yields such a NaN value
        // and infer the final grasp position from its surrounding depth pixels.
        Eigen::Vector3d grasp_position;
        if (!compute_grasp_position(grasp_pixel_coords, depth_image, image_roi, *camera_info_msg, grasp_position))
        {
            ROS_WARN_NAMED(node_name, "Unable to compute grasp position!");
            continue;
        }

        // The grasp coordinate system is obtained from the surface normal at the grasp position
        auto grasp_surface_normal = compute_surface_normal(
                grasp_position, depth_image, image_roi, *camera_info_msg, normal_radius);
        auto grasp_rotation = compute_frame_rotation(grasp_surface_normal, angle_from_rotated_rect(plate_rect));

        // Create grasp pose message and publish it
        geometry_msgs::TransformStamped chair_transform;
        chair_transform.header.stamp = ros::Time::now();
        chair_transform.header.frame_id = camera_frame;
        chair_transform.child_frame_id = grasp_frame;
        chair_transform.transform.translation.x = grasp_position.x();
        chair_transform.transform.translation.y = grasp_position.y();
        chair_transform.transform.translation.z = grasp_position.z();
        chair_transform.transform.rotation.x = grasp_rotation.x();
        chair_transform.transform.rotation.y = grasp_rotation.y();
        chair_transform.transform.rotation.z = grasp_rotation.z();
        chair_transform.transform.rotation.w = grasp_rotation.w();
        transform_broadcaster.sendTransform(chair_transform);

        // Generate output files for debugging
        if (generate_debug_output)
        {
            save_column_vector(horizontal_hist_blurred, debug_output_dir + "/horizontal_hist.csv");
            save_column_vector(vertical_hist_blurred, debug_output_dir + "/vertical_hist.csv");

            save_column_vector(horizontal_hist_gradient, debug_output_dir + "/horizontal_hist_gradient.csv");
            save_column_vector(vertical_hist_gradient, debug_output_dir + "/vertical_hist_gradient.csv");

            save_column_vector(horizontal_peak_indicators, debug_output_dir + "/horizontal_peak_indicators.csv");
            save_column_vector(vertical_peak_indicators, debug_output_dir + "/vertical_peak_indicators.csv");
        }

        // Show a window for visualization
        if (show_visualization)
        {
            draw_rotated_rect(image_roi, plate_rect, cv::Scalar{128.});
            cv::circle(image_roi, grasp_pixel_coords, 3, cv::Scalar{128.});
            cv::imshow(window_name, image_roi);
            cv::waitKey(3);
        }
    }

    cv::destroyWindow(window_name);
    return 0;
}
