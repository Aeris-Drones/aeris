"""Synthetic gas plume simulator for testing map visualization pipelines.

Generates time-varying isopleth polygons that mimic drifting gas plumes,
allowing validation of gas concentration overlays without requiring
actual chemical sensors.
"""

import math
from typing import Any, List

import rclpy
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.node import Node

from aeris_msgs.msg import GasIsopleth
from geometry_msgs.msg import Point32, Polygon


class GasIsoplethNode(Node):
    """Generates synthetic gas plume contours with realistic drift and diffusion.

    Produces time-varying rectangular polygons that simulate plume boundaries
    moving with wind drift. Used to validate gas visualization overlays
    and downstream alerting logic without hardware dependencies.

    The simulation uses sinusoidal functions to model:
    - Plume drift: Horizontal displacement due to wind
    - Plume expansion/contraction: Vertical size variation due to turbulence

    Attributes:
        _rate_hz: Publication rate in Hz.
        _species: Chemical species being simulated (e.g., "VOC", "CO2").
        _units: Concentration units (e.g., "ppm", "ppb").
        _counter: Frame counter for time-varying calculations.
        _publisher: ROS publisher for GasIsopleth messages.
        _timer: ROS timer triggering periodic publications.

    ROS Parameters:
        rate_hz (float): Publication frequency in Hz. Must be positive.
            Default: 0.5
        species (str): Chemical species identifier.
            Default: "VOC"
        units (str): Concentration units.
            Default: "ppm"

    ROS Topics:
        Published:
            gas/isopleth (aeris_msgs/msg/GasIsopleth): Gas plume contours
                with species, units, polygons, and centerline.
    """

    def __init__(self) -> None:
        """Initialize the gas isopleth node.

        Declares parameters, initializes the frame counter, creates the
        publisher, and starts the timer.
        """
        super().__init__("gas_isopleth")
        self._rate_hz: float = self.declare_parameter("rate_hz", 0.5).value
        self._species: str = self.declare_parameter("species", "VOC").value
        self._units: str = self.declare_parameter("units", "ppm").value
        self._counter: int = 0
        self._publisher = self.create_publisher(
            GasIsopleth, "gas/isopleth", 10
        )
        self._timer = self.create_timer(self._period_sec, self._publish_sample)
        self.add_on_set_parameters_callback(self._handle_param_update)

    @property
    def _period_sec(self) -> float:
        """Calculate timer period from current rate.

        Returns:
            Timer period in seconds, clamped to prevent excessive rates.
        """
        rate = max(self._rate_hz, 0.1)
        return 1.0 / rate

    def _handle_param_update(self, params: List[Parameter]) -> SetParametersResult:
        """Handle runtime parameter updates.

        Args:
            params: List of Parameter objects being set.

        Returns:
            SetParametersResult indicating success or failure.

        Note:
            When rate_hz is updated, the timer is cancelled and recreated
            with the new period. Species and units update immediately.
        """
        updated = False
        for param in params:
            if param.name == "rate_hz":
                try:
                    value = float(param.value)
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False)
                if value <= 0.0:
                    return SetParametersResult(successful=False)
                self._rate_hz = value
                updated = True
            elif param.name == "species":
                self._species = str(param.value)
            elif param.name == "units":
                self._units = str(param.value)
        if updated:
            # Recreate timer with new period
            self._timer.cancel()
            self._timer = self.create_timer(
                self._period_sec, self._publish_sample
            )
            self.get_logger().info(
                f"gas isopleth rate set to {self._rate_hz:.2f} Hz"
            )
        return SetParametersResult(successful=True)

    def _rectangle(self) -> Polygon:
        """Generate a time-varying rectangular plume polygon.

        Creates a rectangle representing the gas plume boundary with:
        - Horizontal drift: Sinusoidal variation simulating wind direction changes
        - Vertical expansion: Sinusoidal variation simulating turbulence effects

        Returns:
            Polygon with 4 vertices representing the plume boundary.

        Algorithm:
            drift = sin(counter * 0.1) * 1.5  # Oscillates between -1.5 and +1.5
            height = 1.0 + 0.5 * sin(counter * 0.07)  # Varies between 0.5 and 1.5
        """
        self._counter += 1
        # Calculate horizontal drift using sinusoidal function
        # Period: 2*PI/0.1 ≈ 62 frames for full oscillation
        drift = math.sin(self._counter * 0.1) * 1.5
        width = 2.0
        # Calculate height variation with different frequency
        # Period: 2*PI/0.07 ≈ 90 frames for full oscillation
        height = 1.0 + 0.5 * math.sin(self._counter * 0.07)
        # Compute rectangle bounds centered on drift
        x_min = drift - width / 2.0
        x_max = drift + width / 2.0
        y_min = -height / 2.0
        y_max = height / 2.0
        points = [
            Point32(x=x_min, y=y_min, z=0.0),
            Point32(x=x_max, y=y_min, z=0.0),
            Point32(x=x_max, y=y_max, z=0.0),
            Point32(x=x_min, y=y_max, z=0.0),
        ]
        return Polygon(points=points)

    def _centerline(self) -> List[Point32]:
        """Generate the plume centerline path.

        Creates a polyline representing the center of the gas plume,
        following the same drift pattern as the boundary polygon.

        Returns:
            List of Point32 vertices defining the centerline path.

        Algorithm:
            The centerline follows the drift calculation and creates
            a curved path with 3 points to represent plume curvature.
        """
        drift = math.sin(self._counter * 0.1) * 1.5
        return [
            Point32(x=drift, y=-1.0, z=0.0),
            Point32(x=drift + 0.2, y=0.0, z=0.0),  # Slight curve at center
            Point32(x=drift, y=1.0, z=0.0),
        ]

    def _publish_sample(self) -> None:
        """Generate and publish a synthetic gas isopleth sample.

        Creates a GasIsopleth message with:
        - Current timestamp
        - Configured species and units
        - Time-varying plume boundary polygon
        - Plume centerline path
        """
        msg = GasIsopleth()
        msg.stamp = self.get_clock().now().to_msg()
        msg.species = self._species
        msg.units = self._units
        msg.polygons = [self._rectangle()]
        msg.centerline = self._centerline()
        self._publisher.publish(msg)


def main(args: Any = None) -> None:
    """Entry point for the gas isopleth node.

    Args:
        args: Command line arguments (passed to rclpy.init).
    """
    rclpy.init(args=args)
    node = GasIsoplethNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
