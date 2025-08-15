from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# For system state info that must be reliable and available to late subscribers
reliable_transient_local_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,      # Rety until success
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, # Keep a last 'depth' messages
    depth=10
)

# For high-rate, non-critical sensor data
best_effort_volatile_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,   # Send ony once without retry
    durability=QoSDurabilityPolicy.VOLATILE,        # Do not store old messages
    depth=1
)

# For critical real-time commands, reliable but no stored history
reliable_volatile_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Rety until success
    durability=QoSDurabilityPolicy.VOLATILE,    # Do not store old messages
    depth=5
)