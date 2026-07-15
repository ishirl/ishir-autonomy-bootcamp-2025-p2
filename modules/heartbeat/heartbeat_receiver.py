"""
Heartbeat receiving logic.
"""

from pymavlink import mavutil

from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class HeartbeatReceiver:
    """
    HeartbeatReceiver class to send a heartbeat
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        heartbeat_period: float,
        disconnect_threshold: int,
        local_logger: logger.Logger,
    ) -> "tuple[bool, HeartbeatReceiver | None]":
        """
        Falliable create (instantiation) method to create a HeartbeatReceiver object.
        """
        if heartbeat_period <= 0 or disconnect_threshold <= 0:
            local_logger.error("Heartbeat configuration must be positive", True)
            return False, None
        return True, cls(
            cls.__private_key,
            connection,
            heartbeat_period,
            disconnect_threshold,
            local_logger,
        )

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        heartbeat_period: float,
        disconnect_threshold: int,
        local_logger: logger.Logger,
    ) -> None:
        assert key is HeartbeatReceiver.__private_key, "Use create() method"

        self.connection = connection
        self.heartbeat_period = heartbeat_period
        self.disconnect_threshold = disconnect_threshold
        self.local_logger = local_logger
        self.missed_heartbeats = 0
        self.status = "Disconnected"

    def run(
        self,
    ) -> str:
        """
        Attempt to recieve a heartbeat message.
        If disconnected for over a threshold number of periods,
        the connection is considered disconnected.
        """
        try:
            message = self.connection.recv_match(
                type="HEARTBEAT",
                blocking=False,
            )
        except Exception as exception:  # pylint: disable=broad-exception-caught
            self.local_logger.error(f"Failed to receive heartbeat: {exception}", True)
            message = None

        if message is not None and message.get_type() == "HEARTBEAT":
            self.missed_heartbeats = 0
            self.status = "Connected"
        else:
            self.missed_heartbeats += 1
            self.local_logger.warning(
                f"Missed heartbeat ({self.missed_heartbeats}/{self.disconnect_threshold})",
                True,
            )
            if self.missed_heartbeats >= self.disconnect_threshold:
                self.status = "Disconnected"

        return self.status


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
