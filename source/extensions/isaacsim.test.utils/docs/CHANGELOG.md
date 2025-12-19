# Changelog
## [0.6.4] - 2025-12-07
### Changed
- Update description

## [0.6.3] - 2025-12-02
### Added
- Specify --/app/settings/fabricDefaultStageFrameHistoryCount=3 for startup test

### Removed
- Remove omni.replicator.core as an explicit test dependency

## [0.6.2] - 2025-11-20
### Changed
- Add omni.replicator.core as an explicit dependency for image capture utils

## [0.6.1] - 2025-11-10
### Changed
- Fix invalid escape sequence

## [0.6.0] - 2025-11-07
### Added
- Added `compare_images_in_directories()` function to compare images in two directories

## [0.5.1] - 2025-09-26
### Changed
- Update license headers

## [0.5.0] - 2025-09-22
### Changed
- Can now optionally exclude blank pixels from image_comparison.compute_difference_metrics.

## [0.4.0] - 2025-09-19
### Added
- Added `image_io.py` module with image I/O utilities:
  - `save_rgb_image()` function for saving RGB/RGBA image data to disk
  - `save_depth_image()` function for saving depth data as TIFF (float32) or grayscale visualization
  - `read_image_as_array()` function for reading images as numpy arrays compatible with comparison utilities
- Added `capture_viewport_annotator_data_async()` function to capture annotator data from existing viewports

### Changed
- Moved image saving functions (`save_rgb_image`, `save_depth_image`) from `image_capture.py` to new `image_io.py` module
- Added optional `render_product` parameter to `capture_annotator_data_async()`, `capture_rgb_data_async()`, `capture_depth_data_async()

## [0.3.0] - 2025-09-15
### Changed
- Added file_validation.py for utilities to validate folder contents and file lists

## [0.2.1] - 2025-09-08
### Added
- Base test class stores the asset root path using fast path

## [0.2.0] - 2025-08-28
### Added
- Added image comparison utilities
- Added image capture utilities
- Added test utilities

## [0.1.0] - 2025-08-11
### Added
- Initial release
