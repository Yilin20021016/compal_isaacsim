# Changelog

## [1.2.4] - 2025-12-11
### Changed
- Update ros2 test case to wait for viewport to be ready

## [1.2.3] - 2025-12-07
### Changed
- Fix clang tidy issues in cpp code

## [1.2.2] - 2025-11-26
### Changed
- PointCloud2 message backend now optionally supports host-pinned buffer

## [1.2.1] - 2025-11-26
### Added
 - Default setting values to control ROS2PublishImage queue thread optimization.

## [1.2.0] - 2025-11-24
### Added
- Add flag to generateBuffer() to allocate CUDA pinned memory for image buffers

## [1.1.0] - 2025-11-24
### Changed
- Moved handle interface from isaacsim.core.nodes extension to this extension.

## [1.0.2] - 2025-11-20
### Changed
- Use set_lens_distortion_model in TestCameraInfoUtils

## [1.0.1] - 2025-11-07
### Changed
- Update to Kit 109 and Python 3.12

## [1.0.0] - 2025-11-02
### Added
- Initial release of `isaacsim.ros2.core` extension
- Moved core ROS 2 libraries and backend functionality from isaacsim.ros2.bridge to this extension
