# Changelog

## [1.1.3] - 2025-12-11
### Changed
- Try removing listener from registry after node resets
- Fix issues with unit tests
- Reset simulation time on stop by default for UCXCameraHelper

## [1.1.2] - 2025-12-07
### Changed
- Fix issues found by clang tidy

## [1.1.1] - 2025-12-02
### Changed
- Updated deprecated imports to isaacsim.storage.native

## [1.1.0] - 2025-11-25
### Added
- Joint state publishing node
- Joint command subscribing node
- Odometry publishing node
- IMU publishing node
- RGB image publishing node
- Camera helper node for connecting the replicator pipeline with image publishing

## [1.0.0] - 2025-11-18
### Added
- Initial release of UCX Nodes extension
- OmniGraph nodes for high-performance UCX communication
