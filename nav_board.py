#!/usr/bin/env python

import argparse
import logging
import logging.config
import yaml
import glob
import serial.tools.list_ports
from pymavlink import mavutil
from RoveComm_Python.rovecomm import RoveComm, RoveCommPacket, get_manifest

def setup_logger(level) -> logging.Logger:
    """
    Sets up the logger used in the autonomy project with appropriate
    handlers and formatting
    """
    try:
        with open("/home/pi/Differential_GPS/logging.yaml", "r") as f:
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
    """
    logger.info("Scanning for connected Flight Controllers...")
    
    # 1. Scan available COM ports for common identifiers
    for port in serial.tools.list_ports.comports():
        desc = port.description.lower()
        hwid = port.hwid.lower()
        
        # Look for standard Ardupilot, Pixhawk, or standard STM32 VCP VID:PID combinations
        if any(kw in desc for kw in ['ardupilot', 'pixhawk', 'px4', 'cube', 'fmu', 'stm32']):
            logger.info(f"Found Flight Controller based on description: {port.device} ({port.description})")
            return port.device
        if '0483:5740' in hwid:
            logger.info(f"Found STM32 VCP Flight Controller based on HWID: {port.device}")
            return port.device

    # 2. Fallback for Raspberry Pi/Linux (ArduPilot usually mounts on ttyACM0)
    acm_ports = glob.glob('/dev/ttyACM*')
    if acm_ports:
        logger.info(f"Using fallback ACM device: {acm_ports[0]}")
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

    # Discover FC Port
    serial_port = args.serial_path if args.serial_path else autodetect_fc(logger)
    
    if not serial_port:
        logger.critical("Could not auto-detect a flight controller. Please specify --serial-path.")
        exit(1)

    # Setup Mavlink connection
    logger.info(f"Connecting to MAVLink on {serial_port} at {args.baud} baud...")
    try:
        master = mavutil.mavlink_connection(serial_port, baud=args.baud)
        # Wait for the first heartbeat to confirm connection
        master.wait_heartbeat(timeout=10)
        logger.info(f"Target connected! System: {master.target_system}, Component: {master.target_component}")
        
        # Request all data streams at 10 Hz
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
            
    except Exception as e:
        logger.critical(f"Failed to establish MAVLink connection: {e}")
        exit(1)

    # State tracking variables (caching values between MAVLink packets)
    current_hacc = 0.0
    current_vacc = 0.0
    current_heading_acc = 360.0 
    current_fix_type = 0
    current_diff = 0
    ekf_pos_valid = False

    logger.info("Starting MAVLink read loop. Press Ctrl-C to terminate.")
    try:
        while True:
            # Block until a valid MAVLink message is received
            msg = master.recv_match(blocking=True)
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
                    logger.debug(f"[TX Compass] Heading: {heading:.4f}")

                    # Send Compass Data (EKF Heading)
                    compass_packet = RoveCommPacket(
                        manifest["Nav"]["Telemetry"]["CompassData"]["dataId"], 
                        "f", 
                        (heading,)
                    )
                    rovecomm_node.write(compass_packet, False)

                    # LOGGING: RoveComm GPS Packet
                    logger.info(
                        f"[TX GPS] Lat: {lat:.7f}, Lon: {lon:.7f}, Alt: {alt:.3f}, "
                        f"HAcc: {current_hacc:.3f}, VAcc: {current_vacc:.3f}, "
                        f"HeadAcc: {current_heading_acc:.3f}, Fix: {effective_fix_type}, Diff: {current_diff}"
                    )

                    # Send GPS Data (Fusing EKF position with GPS accuracy)
                    gps_packet = RoveCommPacket(
                        manifest["Nav"]["Telemetry"]["GPSLatLonAlt"]["dataId"], 
                        "d", 
                        (lat, lon, alt, current_hacc, current_vacc, current_heading_acc, effective_fix_type, current_diff)
                    )
                    rovecomm_node.write(gps_packet, False)

            # ------------------------------------------------------------------
            # 2. GPS_RAW_INT: Satellite Info, Fix Details, Accuracies
            # ------------------------------------------------------------------
            elif msg_type == "GPS_RAW_INT":
                num_svs = msg.satellites_visible
                current_fix_type = msg.fix_type
                
                # ArduPilot Fix Types: 0/1=No Fix, 2=2D, 3=3D, 4=DGPS, 5=RTK Float, 6=RTK Fixed
                current_diff = 1 if current_fix_type >= 4 else 0
                
                # Standard horizontal/vertical DOP values (eph/epv are in cm)
                current_hacc = (msg.eph / 100.0) if msg.eph != 65535 else 0.0
                current_vacc = (msg.epv / 100.0) if msg.epv != 65535 else 0.0
                
                # MAVLink 2 extended fields provide precise mm/deg level accuracy
                if hasattr(msg, 'h_acc') and msg.h_acc > 0:
                    current_hacc = msg.h_acc / 1000.0  # mm to meters
                if hasattr(msg, 'v_acc') and msg.v_acc > 0:
                    current_vacc = msg.v_acc / 1000.0  # mm to meters
                if hasattr(msg, 'hdg_acc') and msg.hdg_acc > 0:
                    current_heading_acc = msg.hdg_acc / 1e5  # degE5 to degrees
                
                # LOGGING: RoveComm Sat Count Packet
                logger.debug(f"[TX SatCount] NumSvs: {num_svs}, Fix: {current_fix_type}")

                sat_packet = RoveCommPacket(
                    manifest["Nav"]["Telemetry"]["SatelliteCountData"]["dataId"], 
                    "B", 
                    (num_svs,)
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

    except KeyboardInterrupt:
        logger.info("Terminated by user")
    finally:
        # Cleanup
        rovecomm_node.close_thread()
        if hasattr(master.port, 'close'):
            master.port.close()

if __name__ == "__main__":
    main()