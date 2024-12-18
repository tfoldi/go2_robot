#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>
#include <cstring> // For std::memcpy

class PointCloudCreator
{
public:
    PointCloudCreator(uint8_t *heap, uint32_t positions, size_t point_count, double resolution)
        : heap_(heap), positions_(positions), point_count_(point_count), resolution_(resolution)
    {
    }

    sensor_msgs::msg::PointCloud2 create_point_cloud()
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;

        // Set the frame ID and timestamp
        cloud_msg.header.frame_id = "map";
        cloud_msg.header.stamp = rclcpp::Clock().now();

        // Define the fields: x, y, z
        cloud_msg.height = 1;
        cloud_msg.width = point_count_;
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;

        // Define the fields (x, y, z as floats)
        cloud_msg.fields.resize(3);

        cloud_msg.fields[0].name = "x";
        cloud_msg.fields[0].offset = 0;
        cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[0].count = 1;

        cloud_msg.fields[1].name = "y";
        cloud_msg.fields[1].offset = 4;
        cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[1].count = 1;

        cloud_msg.fields[2].name = "z";
        cloud_msg.fields[2].offset = 8;
        cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[2].count = 1;

        // Set the point step and row step
        cloud_msg.point_step = 12;            // 3 floats * 4 bytes each
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;

        // Resize the data buffer
        cloud_msg.data.resize(cloud_msg.row_step);


        size_t valid_points = 0;
        for (size_t i = 0; i < point_count_; i++ )
        {

            uint8_t ui_x = (uint8_t) heap_[positions_ + i * 3];
            uint8_t ui_y = (uint8_t) heap_[positions_ + i * 3 + 1];
            uint8_t ui_z = (uint8_t) heap_[positions_ + i * 3 + 2];

            if (ui_x == 0 && ui_y == 0 && ui_z == 0)
            {
                continue;
            }

            // Extract x, y, z
            float x = ui_x * resolution_;
            float y = ui_y * resolution_;
            float z = ui_z * resolution_;

            // Copy data into the point cloud buffer
            std::memcpy(&cloud_msg.data[i * cloud_msg.point_step + 0], &x, sizeof(float));        // x
            std::memcpy(&cloud_msg.data[i * cloud_msg.point_step + 4], &y, sizeof(float));        // y
            std::memcpy(&cloud_msg.data[i * cloud_msg.point_step + 8], &z, sizeof(float));        // z
            // std::memcpy(&cloud_msg.data[i * cloud_msg.point_step + 12], &intensity, sizeof(float)); // intensity


            // if (i % 300 &&  x!=0 && y!=0 && z!=0)
                // RCLCPP_INFO(rclcpp::get_logger("PointCloudCreator"), "Point %zu: x=%d, y=%d, z=%d\n", i, x, y, z);
            // printf( "Point %zu: x=%d, y=%d, z=%d\n", i, x, y, z);

            // cloud_msg.data[i * cloud_msg.point_step + 0] = x;
            // cloud_msg.data[i * cloud_msg.point_step + 4] = y;
            // cloud_msg.data[i * cloud_msg.point_step + 8] = z;

            valid_points++;
        }

        cloud_msg.width = valid_points;
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
        cloud_msg.data.resize(cloud_msg.row_step);


        return cloud_msg;
    }

private:
    uint8_t *heap_;
    uint32_t positions_;
    size_t point_count_;
    double resolution_;
};
