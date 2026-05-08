// Copyright (c) 2026, Ultradent Products Inc. All rights reserved.
#include <atomic>
#include <charconv>
#include <csignal>
#include <cstring>
#include <vector>

#include "Amfitrack.hpp"

namespace {
std::atomic<bool> g_quit;
void SignalHandler(int) {
  g_quit = true;
}
bool ParseArgs(int argc, char* argv[], std::vector<uint8_t>& sensor_ids) {
  if (!argv) {
    std::cerr << "argv cannot be null" << std::endl;
    return false;
  }
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <sensor_id> [<sensor_id> ...]" << std::endl;
    return false;
  }
  for (int i = 1; i < argc; ++i) {
    uint64_t temp = 0;

    const char* begin = argv[i];
    const char* end = argv[i] + std::strlen(argv[i]);
    const auto result = std::from_chars(begin, end, temp, 10);

    if (result.ec != std::errc()) {
      std::cerr << "Invalid sensor id: " << argv[i] << std::endl;
      return false;
    }
    if (temp > std::numeric_limits<uint8_t>::max()) { return false; }

    sensor_ids.push_back(static_cast<uint8_t>(temp));
  }
  return true;
}
void WriteData(const std::vector<lib_AmfiProt_Amfitrack_Pose_t>& poses) {
  std::cout << "\r";
  for (const auto& pose : poses) {
    std::cout << " x: " << pose.position_x_in_m << " y: " << pose.position_y_in_m
              << " z: " << pose.position_z_in_m << std::flush;
  }

  const size_t n = poses.size();
  for (int i = 1; i < n; ++i) {
    std::cout << "\33[A";
  }

}
}  // namespace

/**
 * @brief Very simple example that displays sensor state and position in the terminal.
 * @details  Call signature: ./example <sensor id> [sensor id] ... [sensor id]
 * @param argc number of arguments
 * @param argv argument values
 * @return Returns 0 upon success 1 otherwise.
 */
int main(int argc, char* argv[]) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);
  std::vector<uint8_t> sensor_ids;
  if (!ParseArgs(argc, argv, sensor_ids)) {
    return 1;
  }

  auto& amfi_track = AMFITRACK_API::AMFITRACK::getInstance();

  amfi_track.initialize_amfitrack();
  while (!g_quit) {
    amfi_track.amfitrack_main_loop();

    std::vector<lib_AmfiProt_Amfitrack_Pose_t> poses;
    for (const auto& sensor_id : sensor_ids) {
      if (amfi_track.getDeviceActive(sensor_id)) {
        lib_AmfiProt_Amfitrack_Pose_t pose{};
        amfi_track.getDevicePose(sensor_id, &pose);
        poses.push_back(pose);
      }
    }

    WriteData(poses);

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}