"""
Telemetry gathering logic.
"""

import time

from pymavlink import mavutil

from ..common.modules.logger import logger


class TelemetryData:  # pylint: disable=too-many-instance-attributes
    """
    Python struct to represent Telemtry Data. Contains the most recent attitude and position reading.
    """

    def __init__(
        self,
        time_since_boot: int | None = None,  # ms
        x: float | None = None,  # m
        y: float | None = None,  # m
        z: float | None = None,  # m
        x_velocity: float | None = None,  # m/s
        y_velocity: float | None = None,  # m/s
        z_velocity: float | None = None,  # m/s
        roll: float | None = None,  # rad
        pitch: float | None = None,  # rad
        yaw: float | None = None,  # rad
        roll_speed: float | None = None,  # rad/s
        pitch_speed: float | None = None,  # rad/s
        yaw_speed: float | None = None,  # rad/s
    ) -> None:
        self.time_since_boot = time_since_boot
        self.x = x
        self.y = y
        self.z = z
        self.x_velocity = x_velocity
        self.y_velocity = y_velocity
        self.z_velocity = z_velocity
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.roll_speed = roll_speed
        self.pitch_speed = pitch_speed
        self.yaw_speed = yaw_speed

    def __str__(self) -> str:
        return f"""{{
            time_since_boot: {self.time_since_boot},
            x: {self.x},
            y: {self.y},
            z: {self.z},
            x_velocity: {self.x_velocity},
            y_velocity: {self.y_velocity},
            z_velocity: {self.z_velocity},
            roll: {self.roll},
            pitch: {self.pitch},
            yaw: {self.yaw},
            roll_speed: {self.roll_speed},
            pitch_speed: {self.pitch_speed},
            yaw_speed: {self.yaw_speed}
        }}"""


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class Telemetry:
    """
    Telemetry class to read position and attitude (orientation).
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        timeout: float,
        local_logger: logger.Logger,
    ) -> "tuple[bool, Telemetry | None]":
        """
        Falliable create (instantiation) method to create a Telemetry object.
        """
        if timeout <= 0:
            local_logger.error("Telemetry timeout must be positive", True)
            return False, None
        return True, cls(cls.__private_key, connection, timeout, local_logger)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        timeout: float,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Telemetry.__private_key, "Use create() method"

        self.connection = connection
        self.timeout = timeout
        self.local_logger = local_logger

    def run(
        self,
    ) -> "tuple[bool, TelemetryData | None]":
        """
        Receive LOCAL_POSITION_NED and ATTITUDE messages from the drone,
        combining them together to form a single TelemetryData object.
        """
        deadline = time.monotonic() + self.timeout
        attitude_message = None
        position_message = None

        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            try:
                message = self.connection.recv_match(
                    type=["ATTITUDE", "LOCAL_POSITION_NED"],
                    blocking=True,
                    timeout=max(remaining, 0),
                )
            except Exception as exception:  # pylint: disable=broad-exception-caught
                self.local_logger.error(f"Failed to receive telemetry: {exception}", True)
                return False, None

            if message is None:
                break
            if message.get_type() == "ATTITUDE":
                attitude_message = message
            elif message.get_type() == "LOCAL_POSITION_NED":
                position_message = message

            if attitude_message is not None and position_message is not None:
                timestamp = max(attitude_message.time_boot_ms, position_message.time_boot_ms)
                return True, TelemetryData(
                    time_since_boot=timestamp,
                    x=position_message.x,
                    y=position_message.y,
                    z=position_message.z,
                    x_velocity=position_message.vx,
                    y_velocity=position_message.vy,
                    z_velocity=position_message.vz,
                    roll=attitude_message.roll,
                    pitch=attitude_message.pitch,
                    yaw=attitude_message.yaw,
                    roll_speed=attitude_message.rollspeed,
                    pitch_speed=attitude_message.pitchspeed,
                    yaw_speed=attitude_message.yawspeed,
                )

        self.local_logger.error("Timed out waiting for both telemetry messages", True)
        return False, None


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
