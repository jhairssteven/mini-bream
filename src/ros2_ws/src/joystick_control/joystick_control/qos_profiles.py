from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# Define a reliable and transient local QoS profile, for system state info
reliable_transient_local_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

# Define a best-effort and volatile QoS profile, for non-critical sensor data
best_effort_volatile_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1
)

# Define a reliable and volatile QoS profile with a small depth, for critical command data
reliable_volatile_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=5
)

qos = best_effort_volatile_qos