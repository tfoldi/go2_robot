#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <string.h>
#include <unitree_go/msg/voxel_map_compressed.hpp>

// Include the generated Wasm header
#include "voxel.h"
#include "wasm-rt.h"

u32 w2c_a_a(struct w2c_a *instance, uint32_t t) {
  wasm_rt_memory_t *memory = w2c_libvoxel_c((struct w2c_libvoxel *)instance);
  return memory->size;
}

void w2c_a_b(struct w2c_a *instance, uint32_t target, uint32_t start,
             uint32_t end) {
  wasm_rt_memory_t *memory = w2c_libvoxel_c((struct w2c_libvoxel *)instance);
  uint8_t *heap = memory->data;

  // Calculate the number of bytes to copy
  uint32_t length = end - start;

  // Ensure the copy operation stays within bounds
  if (start >= memory->size || end > memory->size || target >= memory->size ||
      target + length > memory->size) {
    return; // Bounds check failed, don't perform the copy
  }

  // Use memmove to handle overlapping ranges
  memmove(&heap[target], &heap[start], length);
}

// LidarDecoder Class
class LidarDecoder {
public:
  LidarDecoder() {
    wasm2c_libvoxel_instantiate(&instance_, w2c_a_instance_);
    buffer_ = w2c_libvoxel_c(&instance_)->data;
    memory_size_ = w2c_libvoxel_c(&instance_)->size;

    input_ = w2c_libvoxel_f(&instance_, 61440);
    decompressBuffer_ = w2c_libvoxel_f(&instance_, 80000);
    positions_ = w2c_libvoxel_f(&instance_, 2880000);
    uvs_ = w2c_libvoxel_f(&instance_, 1920000);
    indices_ = w2c_libvoxel_f(&instance_, 5760000);
    decompressedSize_ = w2c_libvoxel_f(&instance_, 4);
    faceCount_ = w2c_libvoxel_f(&instance_, 4);
    pointCount_ = w2c_libvoxel_f(&instance_, 4);
  }

  ~LidarDecoder() { wasm2c_libvoxel_free(&instance_); }

  void decode(const std::vector<uint8_t> &compressed_data, float resolution,
              float origin_z) {
    // Copy compressed data into Wasm memory
    if (compressed_data.size() > memory_size_) {
      throw std::runtime_error("Compressed data exceeds memory size");
    }
    std::memcpy(&buffer_[input_], compressed_data.data(),
                compressed_data.size());

    int some_v = static_cast<int>(origin_z / resolution);

    w2c_libvoxel_e(&instance_, input_, compressed_data.size(),
                   decompressBufferSize_, decompressBuffer_, decompressedSize_,
                   positions_, uvs_, indices_, faceCount_, pointCount_, some_v);

    int point_count = *reinterpret_cast<int *>(&buffer_[pointCount_]);
    int face_count = *reinterpret_cast<int *>(&buffer_[faceCount_]);

    RCLCPP_INFO(rclcpp::get_logger("LidarDecoder"),
                "Decoded %d points and %d faces", point_count, face_count);
  }

private:
  w2c_libvoxel instance_;
  struct w2c_a *w2c_a_instance_;
  uint8_t *buffer_;
  size_t memory_size_;

  uint32_t input_;
  uint32_t decompressBuffer_;
  uint32_t positions_;
  uint32_t uvs_;
  uint32_t indices_;
  uint32_t decompressedSize_;
  uint32_t faceCount_;
  uint32_t pointCount_;
  uint32_t decompressBufferSize_ = 80000;
};

class VoxelMapNode : public rclcpp::Node {
public:
  VoxelMapNode() : Node("voxelmap_node") {
    RCLCPP_INFO(this->get_logger(), "VoxelMapNode has started.");

    point_cloud_sub_ =
        this->create_subscription<unitree_go::msg::VoxelMapCompressed>(
            "/utlidar/voxel_map_compressed", 10,
            std::bind(&VoxelMapNode::voxel_map_callback, this,
                      std::placeholders::_1));
  }

private:
  void
  voxel_map_callback(const unitree_go::msg::VoxelMapCompressed::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
                "Received a VoxelMapCompressed message with %d:%d:%d width "
                "and precision %f.",
                msg->width[0], msg->width[1], msg->width[2], msg->resolution);

    try {
      decoder_.decode(msg->data, msg->resolution, msg->width[2]);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error during decoding: %s", e.what());
    }
  }

  rclcpp::Subscription<unitree_go::msg::VoxelMapCompressed>::SharedPtr
      point_cloud_sub_;

  LidarDecoder decoder_;
};

int main(int argc, char **argv) {
  wasm_rt_init();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelMapNode>());
  rclcpp::shutdown();
  return 0;
}
