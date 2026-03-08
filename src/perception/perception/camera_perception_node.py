"""Camera perception node for the RC car.

Subscribes to raw camera images and provides a dedicated hook for future
computer-vision pipelines.
"""

# Copyright (c) 2024 Maintainer
#
# SPDX-License-Identifier: MIT

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraPerceptionNode(Node):
    """Receive camera frames for CV processing."""

    def __init__(self):
        super().__init__('camera_perception_node')

        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('log_every_n_frames', 100)
        self.declare_parameter('sample_stride', 4)
        self.declare_parameter('low_light_threshold', 60.0)

        self._frame_count = 0
        camera_topic = self.get_parameter('camera_topic').value
        self._log_every_n_frames = int(
            self.get_parameter('log_every_n_frames').value
        )
        self._sample_stride = max(1, int(self.get_parameter('sample_stride').value))
        self._low_light_threshold = float(
            self.get_parameter('low_light_threshold').value
        )
        self._prev_mean_intensity = None
        self._warned_unsupported_encoding = set()

        self._camera_sub = self.create_subscription(
            Image,
            camera_topic,
            self._camera_callback,
            10,
        )

        self.get_logger().info(
            f'Camera perception node started. Subscribed to {camera_topic}'
        )

    def _camera_callback(self, msg: Image) -> None:
        """Handle incoming camera frames."""
        self._frame_count += 1

        frame_features = self._compute_frame_features(msg)
        if frame_features is None:
            return

        mean_intensity = frame_features['mean_intensity']
        center_intensity = frame_features['center_intensity']
        contrast_ratio = frame_features['contrast_ratio']

        if self._prev_mean_intensity is None:
            motion_proxy = 0.0
        else:
            motion_proxy = abs(mean_intensity - self._prev_mean_intensity)
        self._prev_mean_intensity = mean_intensity

        if self._log_every_n_frames > 0 and (
            self._frame_count % self._log_every_n_frames == 0
        ):
            lighting = 'low_light' if mean_intensity < self._low_light_threshold else 'normal'
            self.get_logger().info(
                'Frame '
                f'{self._frame_count}: mean={mean_intensity:.1f}, '
                f'center={center_intensity:.1f}, '
                f'contrast={contrast_ratio:.3f}, '
                f'motion={motion_proxy:.1f}, '
                f'lighting={lighting}, '
                f'encoding={msg.encoding}'
            )

    def _compute_frame_features(self, msg: Image):
        """Compute lightweight vision features from a raw image message."""
        width = msg.width
        height = msg.height
        if width == 0 or height == 0:
            return None

        encoding = msg.encoding.lower()
        if encoding not in {'mono8', 'rgb8', 'bgr8', 'rgba8', 'bgra8'}:
            if encoding not in self._warned_unsupported_encoding:
                self.get_logger().warn(
                    f'Unsupported image encoding: {msg.encoding}. '
                    'Expected mono8/rgb8/bgr8/rgba8/bgra8.'
                )
                self._warned_unsupported_encoding.add(encoding)
            return None

        stride = self._sample_stride
        step = msg.step
        data = msg.data

        center_x_min = int(width * 0.35)
        center_x_max = int(width * 0.65)
        center_y_min = int(height * 0.35)
        center_y_max = int(height * 0.65)

        total_intensity = 0.0
        sample_count = 0
        center_intensity = 0.0
        center_count = 0
        high_contrast_count = 0
        horizontal_pairs = 0

        if encoding == 'mono8':
            bytes_per_pixel = 1
        elif encoding in {'rgb8', 'bgr8'}:
            bytes_per_pixel = 3
        else:
            bytes_per_pixel = 4

        for y in range(0, height, stride):
            row_start = y * step
            prev_intensity = None
            for x in range(0, width, stride):
                pixel_index = row_start + x * bytes_per_pixel

                if encoding == 'mono8':
                    intensity = float(data[pixel_index])
                elif encoding == 'rgb8':
                    red = data[pixel_index]
                    green = data[pixel_index + 1]
                    blue = data[pixel_index + 2]
                    intensity = 0.299 * red + 0.587 * green + 0.114 * blue
                elif encoding == 'bgr8':
                    blue = data[pixel_index]
                    green = data[pixel_index + 1]
                    red = data[pixel_index + 2]
                    intensity = 0.299 * red + 0.587 * green + 0.114 * blue
                elif encoding == 'rgba8':
                    red = data[pixel_index]
                    green = data[pixel_index + 1]
                    blue = data[pixel_index + 2]
                    intensity = 0.299 * red + 0.587 * green + 0.114 * blue
                else:  # bgra8
                    blue = data[pixel_index]
                    green = data[pixel_index + 1]
                    red = data[pixel_index + 2]
                    intensity = 0.299 * red + 0.587 * green + 0.114 * blue

                total_intensity += intensity
                sample_count += 1

                if (
                    center_x_min <= x <= center_x_max
                    and center_y_min <= y <= center_y_max
                ):
                    center_intensity += intensity
                    center_count += 1

                if prev_intensity is not None:
                    if abs(intensity - prev_intensity) > 20.0:
                        high_contrast_count += 1
                    horizontal_pairs += 1
                prev_intensity = intensity

        if sample_count == 0:
            return None

        mean_intensity = total_intensity / sample_count
        if center_count > 0:
            center_mean = center_intensity / center_count
        else:
            center_mean = mean_intensity

        if horizontal_pairs > 0:
            contrast_ratio = high_contrast_count / horizontal_pairs
        else:
            contrast_ratio = 0.0

        return {
            'mean_intensity': mean_intensity,
            'center_intensity': center_mean,
            'contrast_ratio': contrast_ratio,
        }


def main(args=None):
    """Entry point for the camera perception node."""
    rclpy.init(args=args)
    node = CameraPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
