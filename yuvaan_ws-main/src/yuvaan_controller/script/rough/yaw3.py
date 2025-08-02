import pyrealsense2 as rs
import numpy as np
import time
import math

def complementary_filter_update(ax, ay, az, gx, gy, gz, dt,
                                roll, pitch, yaw, alpha=0.98):
    """
    Update roll, pitch, yaw using a simple complementary filter:
      - Integrate gyro for short-term estimate
      - Use accel for correcting roll/pitch drift
      - Yaw is left uncorrected by accel (no magnetometer).
    """
    # Gyro integration
    roll_gyro  = roll  + gx * dt
    pitch_gyro = pitch + gy * dt
    yaw_gyro   = yaw   + gz * dt

    # Normalize accelerometer
    accel_norm = math.sqrt(ax*ax + ay*ay + az*az)
    if accel_norm > 1e-6:
        axn = ax / accel_norm
        ayn = ay / accel_norm
        azn = az / accel_norm

        # Approximate pitch/roll from accel
        pitch_acc = math.atan2(-axn, math.sqrt(ayn*ayn + azn*azn))
        roll_acc  = math.atan2(ayn, azn)

        # Complementary filter mix
        roll  = alpha * roll_gyro  + (1 - alpha) * roll_acc
        pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc
        # Typically yaw is integrated only
        yaw   = yaw_gyro
    else:
        # If accel data is invalid, just trust gyro
        roll  = roll_gyro
        pitch = pitch_gyro
        yaw   = yaw_gyro

    return roll, pitch, yaw

def main():
    # 1) Setup RealSense pipeline for IMU (accel + gyro)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.accel)
    config.enable_stream(rs.stream.gyro)
    profile = pipeline.start(config)

    # 2) Initialize orientation and timing
    roll, pitch, yaw = 0.0, 0.0, 0.0
    prev_time = time.time()

    # 3) Yaw turn detection settings
    reference_yaw_degs = 0.0
    TURN_THRESHOLD = 90.0  # degrees

    try:
        print("Starting IMU turn detection. Press Ctrl+C to stop.\n")
        while True:
            # Poll new frames
            frames = pipeline.poll_for_frames()
            if not frames:
                continue

            accel_frame = frames.first_or_default(rs.stream.accel)
            gyro_frame  = frames.first_or_default(rs.stream.gyro)
            if not accel_frame or not gyro_frame:
                continue

            # Extract motion data
            accel_data = accel_frame.as_motion_frame().get_motion_data()
            gyro_data  = gyro_frame.as_motion_frame().get_motion_data()

            ax = accel_data.x
            ay = accel_data.y
            az = accel_data.z
            gx = gyro_data.x
            gy = gyro_data.y
            gz = gyro_data.z

            # Compute delta time
            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time

            # 4) Update roll, pitch, yaw
            roll, pitch, yaw = complementary_filter_update(
                ax, ay, az,
                gx, gy, gz,
                dt, roll, pitch, yaw,
                alpha=0.98
            )

            # Convert yaw to degrees
            yaw_degs = math.degrees(yaw)

            # 5) Compare current yaw with reference to detect 90° turns
            delta = yaw_degs - reference_yaw_degs

            # Optionally normalize the difference to (-180, 180]
            if delta > 180:
                delta -= 360
            elif delta <= -180:
                delta += 360

            # Check thresholds
            if delta > TURN_THRESHOLD:
                print("Detected +90° turn (e.g., Left turn)")
                # Reset reference yaw so the next 90° is measured from here
                reference_yaw_degs += 90

            elif delta < -TURN_THRESHOLD:
                print("Detected -90° turn (e.g., Right turn)")
                # Reset reference yaw so the next 90° is measured from here
                reference_yaw_degs -= 90

            # Small delay to avoid busy loop
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping turn detection.")
    finally:
        pipeline.stop()

if __name__ == "__main__":
    main()