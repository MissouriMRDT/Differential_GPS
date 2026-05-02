#!/usr/bin/env python

import argparse
import logging
import logging.config
import yaml
import glob
import serial.tools.list_ports
import os
import time
from pathlib import Path

# Force MAVLink 2.0 protocol before importing pymavlink to ensure 
# we get the extended fields in messages like GPS_RAW_INT and to 
# match the flight controller's SERIAL0_PROTOCOL=2 configuration.
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil
from RoveComm_Python.rovecomm import RoveComm, RoveCommPacket, get_manifest

def setup_logger(level) -> logging.Logger:
    """
    Sets up the logger used in the autonomy project with appropriate
    handlers and formatting
    """
    try:
        # Use pathlib to get the path relative to this script's location.
        config_path = Path(__file__).parent / "logging.yaml"
        with open(config_path, "r") as f:
            yaml_conf = yaml.safe_load(f.read())
        logging.config.dictConfig(yaml_conf)
    except Exception:
        # Fallback if file not found
        logging.basicConfig(level=level, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    logger = logging.getLogger()
    logger.setLevel(level)
    return logger

def autodetect_fc(logger: logging.Logger) -> str:
    """
    Auto-detects a connected flight controller over USB.
    Always prefers the lower-numbered port (e.g., ttyACM0 over ttyACM1) 
    because Ardupilot exposes MAVLink on the first USB interface and SLCAN on the second.
    """
    logger.info("Scanning for connected Flight Controllers...")
    
    valid_ports = []
    
    # 1. Scan available COM ports for common identifiers
    for port in serial.tools.list_ports.comports():
        # Safely handle None types for virtual/headless TTY ports
        desc = (port.description or "").lower()
        hwid = (port.hwid or "").lower()
        
        # Look for standard Ardupilot, Pixhawk, or standard STM32 VCP VID:PID combinations
        if any(kw in desc for kw in ['ardupilot', 'pixhawk', 'px4', 'cube', 'fmu', 'stm32', 'ark']):
            valid_ports.append(port)
        elif '0483:5740' in hwid or '1209:5740' in hwid:
            valid_ports.append(port)

    if valid_ports:
        # Sort ports alphabetically so /dev/ttyACM0 always comes before /dev/ttyACM1
        valid_ports.sort(key=lambda p: p.device)
        best_port = valid_ports[0]
        logger.info(f"Auto-selected MAVLink port: {best_port.device} ({best_port.description})")
        return best_port.device

    # 2. Fallback for Raspberry Pi/Linux
    # Sort the fallback ports as well to ensure ttyACM0 is picked before ttyACM1
    acm_ports = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    if acm_ports:
        logger.info(f"Using fallback serial device: {acm_ports[0]}")
        return acm_ports[0]
        
    return None

def main() -> None:
    # Create argument parser.
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--serial-path",
        help="Optional: Force a specific serial path (e.g. /dev/ttyACM0). Overrides auto-detect.",
        default=None,
    )
    parser.add_argument(
        "--baud",
        help="Baud rate for MAVLink connection",
        type=int,
        default=115200, # 115200 is standard for USB Mavlink
    )
    parser.add_argument(
        "--level",
        choices=["DEBUG", "INFO", "WARN", "CRITICAL", "ERROR"],
        default="INFO",
    )

    args = parser.parse_args()
    
    # Setup logger
    level = getattr(logging, args.level)
    logger = setup_logger(level)

    # Get rovecomm manifest and initialize node
    manifest = get_manifest()
    rovecomm_node = RoveComm()

    try:
        # MAIN BLACKBOX LOOP: Never exit, always retry
        while True:
            # Discover FC Port
            serial_port = args.serial_path if args.serial_path else autodetect_fc(logger)
            
            if not serial_port:
                logger.warning("Could not auto-detect a flight controller. Retrying in 5 seconds...")
                time.sleep(5)
                continue

            # Give Linux a moment to fully assign permissions and mount the USB device
            # before we attempt to seize the file descriptor.
            time.sleep(1.0)

            # Setup Mavlink connection
            logger.info(f"Attempting to connect to MAVLink on {serial_port} at {args.baud} baud...")
            master = None
            try:
                # Removed autoreconnect=True! PyMavlink's internal reconnect logic 
                # deadlocks on Linux USB VCPs. We rely on the outer while True loop instead.
                master = mavutil.mavlink_connection(serial_port, baud=args.baud, source_system=255)
                
                # ArduPilot over USB (VCP) will NOT send any data unless DTR is asserted.
                if hasattr(master, 'port'):
                    if hasattr(master.port, 'setDTR'):
                        try:
                            master.port.setDTR(True)
                            # Removed setRTS(True) as it can sometimes force an STM32 hardware reset
                        except Exception as e:
                            logger.warning(f"Could not set DTR on serial port: {e}")
                    
                    # Prevent write operations from deadlocking if the hardware buffer is full
                    try:
                        master.port.write_timeout = 1.0
                    except Exception:
                        pass
                
                # Allow the STM32 Virtual COM Port a moment to settle in the OS 
                time.sleep(1.0)
                
                # Send a GCS Heartbeat. Flight controllers mute USB telemetry 
                # until they hear from a GCS. We blast it a few times to wake it up.
                heartbeat = None
                logger.info("Broadcasting GCS heartbeat at 1Hz and waiting for response...")
                
                for attempt in range(1, 6):
                    logger.info(f"Attempt {attempt}/5: Sending GCS heartbeat...")
                    
                    # If this fails, it will raise an exception and bubble up to our main try/except, 
                    # forcing a clean 3-second restart instead of deadlocking.
                    master.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0
                    )
                        
                    logger.info(f"Attempt {attempt}/5: Listening for response...")
                    # Use PyMavlink's built-in timeout, which is much safer than a manual while loop
                    heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
                    
                    if heartbeat:
                        logger.info("Heartbeat received!")
                        break
                
                if heartbeat:
                    logger.info(f"Target connected! System: {master.target_system}, Component: {master.target_component}")
                else:
                    logger.warning(f"Timeout waiting for MAVLink heartbeat on {serial_port}.")
                    if hasattr(master, 'port') and hasattr(master.port, 'close'):
                        master.port.close()
                    time.sleep(2)
                    continue

            except Exception as e:
                logger.error(f"Connection glitch during setup (Restarting connection): {e}")
                if master and hasattr(master, 'port') and hasattr(master.port, 'close'):
                    master.port.close()
                time.sleep(3)
                continue

            try:
                # Only use the modern targeted interval command.
                requested_messages = [
                    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                    mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
                    mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT
                ]
                
                for msg_id in requested_messages:
                    master.mav.command_long_send(
                        master.target_system, master.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                        0, msg_id, 100000, 0, 0, 0, 0, 0
                    )
            except Exception as e:
                logger.error(f"Failed to request MAVLink streams: {e}")
                if hasattr(master.port, 'close'):
                    master.port.close()
                time.sleep(3)
                continue

            # State tracking variables (caching values between MAVLink packets)
            current_hacc = 0.0
            current_vacc = 0.0
            current_heading_acc = 360.0 
            current_fix_type = 0
            current_diff = 0
            ekf_pos_valid = False

            logger.info("Starting MAVLink read loop. Awaiting data...")
            try:
                while True:
                    # Filter recv_match to only return the messages we actually care about.
                    msg = master.recv_match(
                        type=["GLOBAL_POSITION_INT", "GPS_RAW_INT", "EKF_STATUS_REPORT"], 
                        blocking=True
                    )
                    if not msg:
                        continue
                        
                    msg_type = msg.get_type()

                    # ------------------------------------------------------------------
                    # 1. GLOBAL_POSITION_INT: EKF Fused Position, Altitude & Heading
                    # ------------------------------------------------------------------
                    if msg_type == "GLOBAL_POSITION_INT":
                        # Decode EKF global position
                        lat = msg.lat / 1e7
                        lon = msg.lon / 1e7
                        alt = msg.alt / 1000.0  # mm to meters
                        
                        # Heading is in centidegrees (0-35999)
                        heading = msg.hdg / 100.0 if msg.hdg != 65535 else 0.0

                        # Only proceed if we have non-zero lat/lon
                        if lat != 0 and lon != 0:
                            # Override the fix type to 0 (No Fix) if the EKF hasn't locked its absolute position yet
                            effective_fix_type = current_fix_type if ekf_pos_valid else 0

                            # LOGGING: RoveComm Compass Packet
                            logger.info(f"[TX Compass] Heading: {heading:.4f}")

                            # Send Compass Data (EKF Heading)
                            compass_packet = RoveCommPacket(
                                manifest["Nav"]["Telemetry"]["CompassData"]["dataId"], 
                                "f", 
                                (float(heading),)
                            )
                            rovecomm_node.write(compass_packet, False)

                            # LOGGING: RoveComm GPS Packet
                            logger.info(
                                f"[TX GPS] Lat: {lat:.7f}, Lon: {lon:.7f}, Alt: {alt:.3f}, "
                                f"HAcc: {current_hacc:.3f}, VAcc: {current_vacc:.3f}, "
                                f"HeadAcc: {current_heading_acc:.3f}, Fix: {effective_fix_type}, Diff: {current_diff}"
                            )

                            # Explicitly cast all values to float (Python's native C Double) 
                            # before sending the RoveComm 'd' struct payload.
                            gps_packet = RoveCommPacket(
                                manifest["Nav"]["Telemetry"]["GPSLatLonAlt"]["dataId"], 
                                "d", 
                                (
                                    float(lat), 
                                    float(lon), 
                                    float(alt), 
                                    float(current_hacc), 
                                    float(current_vacc), 
                                    float(current_heading_acc), 
                                    float(effective_fix_type), 
                                    float(current_diff)
                                )
                            )
                            rovecomm_node.write(gps_packet, False)

                    # ------------------------------------------------------------------
                    # 2. GPS_RAW_INT: Satellite Info, Fix Details, Accuracies
                    # ------------------------------------------------------------------
                    elif msg_type == "GPS_RAW_INT":
                        num_svs = msg.satellites_visible
                        mavlink_fix = msg.fix_type
                        
                        # ArduPilot Fix Types: 0/1=No Fix, 2=2D, 3=3D, 4=DGPS, 5=RTK Float, 6=RTK Fixed
                        # Determine differential status from MAVLink fix
                        current_diff = 1 if mavlink_fix >= 4 else 0
                        
                        # Translate MAVLink fix type to U-blox NavPVT fix type standard
                        if mavlink_fix <= 1:
                            current_fix_type = 0  # No Fix
                        elif mavlink_fix == 2:
                            current_fix_type = 2  # 2D Fix
                        elif mavlink_fix >= 3:
                            current_fix_type = 3  # 3D Fix (NavPVT uses 3 for 3D, DGPS, and RTK)
                        
                        # Default to 0.0 to prevent passing HDOP as accuracy. 
                        # We strictly rely on extended fields for actual error estimations.
                        current_hacc = 0.0
                        current_vacc = 0.0
                        
                        # Because we forced MAVLINK20 above, these extended 
                        # fields should reliably be populated if the FC supports them.
                        if hasattr(msg, 'h_acc') and msg.h_acc > 0:
                            current_hacc = msg.h_acc / 1000.0  # mm to meters
                        if hasattr(msg, 'v_acc') and msg.v_acc > 0:
                            current_vacc = msg.v_acc / 1000.0  # mm to meters
                        if hasattr(msg, 'hdg_acc') and msg.hdg_acc > 0:
                            current_heading_acc = msg.hdg_acc / 1e5  # degE5 to degrees
                        
                        # LOGGING: RoveComm Sat Count Packet
                        logger.info(f"[TX SatCount] NumSvs: {num_svs}, Fix: {current_fix_type}")

                        sat_packet = RoveCommPacket(
                            manifest["Nav"]["Telemetry"]["SatelliteCountData"]["dataId"], 
                            "B", 
                            (int(num_svs),)
                        )
                        rovecomm_node.write(sat_packet, False)

                    # ------------------------------------------------------------------
                    # 3. EKF_STATUS_REPORT: Ensure the EKF is initialized & ready
                    # ------------------------------------------------------------------
                    elif msg_type == "EKF_STATUS_REPORT":
                        # Check bit 4 (16) which is EKF_POS_HORIZ_ABS
                        # This bit proves the EKF has securely initialized its absolute position
                        has_horiz_abs = bool(msg.flags & 16)
                        
                        if has_horiz_abs and not ekf_pos_valid:
                            logger.info("EKF Absolute Horizontal Position Lock Acquired! Valid data flowing...")
                        elif not has_horiz_abs and ekf_pos_valid:
                            logger.warning("EKF Absolute Horizontal Position Lock Lost!")
                            
                        ekf_pos_valid = has_horiz_abs

            except Exception as e:
                # This catches serial disconnections, dropped USB cables, etc.
                logger.error(f"Connection lost or read error during MAVLink loop: {e}")
            finally:
                if master and hasattr(master, 'port') and hasattr(master.port, 'close'):
                    master.port.close()
                logger.info("Connection closed. Restarting loop in 3 seconds...")
                time.sleep(3)

    except KeyboardInterrupt:
        logger.info("Terminated by user via KeyboardInterrupt")
    finally:
        # Cleanup RoveComm thread only when truly exiting the program
        rovecomm_node.close_thread()

if __name__ == "__main__":
    main()