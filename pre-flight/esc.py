from pymavlink import mavutil
import time

def check_escs(drone):
    """
    Check the health and validity of the ESCs.
    Verifies if ESC telemetry is available and there are no error reports.
    """
    print("Checking ESC status...")
    esc_healthy = True
    try:
        # Request ESC telemetry information
        for i in range(4):  # Assuming the drone has 4 ESCs (adjust as necessary)
            msg = drone.recv_match(type='ESC_INFO', blocking=True, timeout=2)
            if msg:
                # Extract relevant ESC telemetry details
                for index, (rpm, voltage, current) in enumerate(zip(msg.rpm, msg.voltage, msg.current)):
                    if rpm == 0 or voltage < 10 or current > 50:  # Example thresholds
                        print(f"ESC {index} issue detected: RPM={rpm}, Voltage={voltage}, Current={current}")
                        esc_healthy = False
                    else:
                        print(f"ESC {index} OK: RPM={rpm}, Voltage={voltage}, Current={current}")
            else:
                print("ESC telemetry unavailable.")
                esc_healthy = False
                break
    except Exception as e:
        print(f"Error while checking ESCs: {e}")
        esc_healthy = False

    if esc_healthy:
        print("All ESCs are healthy.")
    else:
        print("ESC issues detected.")
    return esc_healthy



