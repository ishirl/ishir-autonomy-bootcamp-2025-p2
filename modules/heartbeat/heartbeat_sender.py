"""
Heartbeat sending logic.
"""

from pymavlink import mavutil


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class HeartbeatSender:
    """
    HeartbeatSender class to send a heartbeat
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        heartbeat_period: float,
    ) -> "tuple[True, HeartbeatSender] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a HeartbeatSender object.
        """
        if heartbeat_period <= 0:
            return False, None
        return True, cls(cls.__private_key, connection, heartbeat_period)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        heartbeat_period: float,
    ) -> None:
        assert key is HeartbeatSender.__private_key, "Use create() method"

        self.connection = connection
        self.heartbeat_period = heartbeat_period

    def run(
        self,
    ) -> bool:
        """
        Attempt to send a heartbeat message.
        """
        try:
            self.connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
        except Exception:  # pylint: disable=broad-exception-caught
            return False
        return True


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
