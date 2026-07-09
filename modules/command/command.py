"""
Decision-making logic.
"""

import math

from pymavlink import mavutil

from ..common.modules.logger import logger
from ..telemetry import telemetry


class Position:
    """
    3D vector struct.
    """

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class Command:  # pylint: disable=too-many-instance-attributes
    """
    Command class to make a decision based on recieved telemetry,
    and send out commands based upon the data.
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        target: Position,
        height_tolerance: float,
        z_speed: float,
        angle_tolerance: float,
        turning_speed: float,
        local_logger: logger.Logger,
    ) -> "tuple[bool, Command | None]":
        """
        Falliable create (instantiation) method to create a Command object.
        """
        if height_tolerance < 0 or z_speed <= 0 or angle_tolerance < 0 or turning_speed <= 0:
            local_logger.error("Command configuration contains an invalid tolerance or speed", True)
            return False, None
        return True, cls(
            cls.__private_key,
            connection,
            target,
            height_tolerance,
            z_speed,
            angle_tolerance,
            turning_speed,
            local_logger,
        )

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        target: Position,
        height_tolerance: float,
        z_speed: float,
        angle_tolerance: float,
        turning_speed: float,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Command.__private_key, "Use create() method"

        self.connection = connection
        self.target = target
        self.height_tolerance = height_tolerance
        self.z_speed = z_speed
        self.angle_tolerance = angle_tolerance
        self.turning_speed = turning_speed
        self.local_logger = local_logger
        self.velocity_sum = Position(0, 0, 0)
        self.velocity_count = 0

    def run(
        self,
        data: telemetry.TelemetryData,
    ) -> "tuple[bool, list[str]]":
        """
        Make a decision based on received telemetry data.
        """
        if any(
            value is None
            for value in (
                data.x,
                data.y,
                data.z,
                data.yaw,
                data.x_velocity,
                data.y_velocity,
                data.z_velocity,
            )
        ):
            self.local_logger.error(
                "Telemetry data is missing required position, yaw, or velocity", True
            )
            return False, []

        self.velocity_sum.x += data.x_velocity
        self.velocity_sum.y += data.y_velocity
        self.velocity_sum.z += data.z_velocity
        self.velocity_count += 1
        average_velocity = Position(
            self.velocity_sum.x / self.velocity_count,
            self.velocity_sum.y / self.velocity_count,
            self.velocity_sum.z / self.velocity_count,
        )
        self.local_logger.info(
            f"Average velocity: ({average_velocity.x}, {average_velocity.y}, {average_velocity.z})",
            True,
        )

        outputs: list[str] = []
        altitude_delta = self.target.z - data.z
        try:
            if abs(altitude_delta) > self.height_tolerance:
                self.connection.mav.command_long_send(
                    1,
                    0,
                    mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
                    0,
                    self.z_speed,
                    0,
                    0,
                    0,
                    0,
                    0,
                    self.target.z,
                )
                outputs.append(f"CHANGE ALTITUDE: {altitude_delta}")
            else:
                target_yaw = math.atan2(self.target.y - data.y, self.target.x - data.x)
                yaw_delta = math.degrees(target_yaw - data.yaw)
                yaw_delta = (yaw_delta + 180) % 360 - 180
                if abs(yaw_delta) > self.angle_tolerance:
                    self.connection.mav.command_long_send(
                        1,
                        0,
                        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                        0,
                        abs(yaw_delta),
                        self.turning_speed,
                        -1 if yaw_delta > 0 else 1,
                        1,
                        0,
                        0,
                        0,
                    )
                    outputs.append(f"CHANGE YAW: {yaw_delta}")
        except Exception as exception:  # pylint: disable=broad-exception-caught
            self.local_logger.error(f"Failed to send command: {exception}", True)
            return False, outputs

        return True, outputs


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
