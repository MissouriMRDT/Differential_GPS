#!/usr/bin/env python

import argparse
import logging
import logging.config
import yaml
import serial
from threading import Thread
from queue import Queue
from enum import Enum
import pyubx2
from RoveComm_Python.rovecomm import RoveComm, RoveCommPacket, get_manifest

# Define constants.
SERIAL_BAUD = 921600

def setup_logger(level) -> logging.Logger:
    """
    Sets up the logger used in the autonomy project with appropriate
    handlers and formatting
    """
    # Try to load the specific logging config, otherwise fall back to basic
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

def read_data(stream: serial.Serial, ubr: pyubx2.UBXReader, queue: Queue):
    """
    Read and parse incoming UBX data and place parsed data on queue.
    Runs in a separate thread.
    """
    while True:
        try:
            # .read() blocks internally until data arrives, so no loop delay needed
            (raw_data, parsed_data) = ubr.read()
            
            if parsed_data:
                queue.put(parsed_data)
                
        except (serial.SerialException, Exception):
            # If an error occurs (e.g. serial unplugged), we pass to keep trying
            pass

def main() -> None:
    # Create argument parser.
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--serial-path",
        help="The serial path that is recieving UBX gps data. Most likely /dev/serial0.",
        default="/dev/serial0",
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

    # Setup serial input and ubx decoder.
    try:
        serial_stream = serial.Serial(port=args.serial_path, baudrate=SERIAL_BAUD, timeout=1)
    except serial.SerialException as e:
        logger.critical(f"Failed to open serial port {args.serial_path}: {e}")
        exit(1)

    # validate=True ensures checksums are checked
    ubx_decode = pyubx2.UBXReader(serial_stream, protfilter=pyubx2.UBX_PROTOCOL, validate=True)

    # Create queue for threaded reading.
    read_queue = Queue()
    
    # Create and start read thread.
    # daemon=True ensures thread dies when script exits
    read_thread = Thread(
        target=read_data,
        args=(serial_stream, ubx_decode, read_queue),
        daemon=True
    )
    logger.info("Starting read handler processes. Press Ctrl-C to terminate...")
    read_thread.start()

    # These must persist across loop iterations because data arrives in different packets
    previousHeadingRover = 0.0
    current_heading_acc = 360.0 

    try:
        while True:
            # Wait for data from the thread (Blocking Get)
            parsed_data = read_queue.get()

            # Skip parser errors or non-UBX messages
            if isinstance(parsed_data, str):
                logger.debug(f"Parser returned string (likely error): {parsed_data}")
                continue
            
            msg_id = parsed_data.identity

            # ------------------------------------------------------------------
            # 1. NAV-RELPOSNED: Relative Positioning & Moving Baseline Heading
            # ------------------------------------------------------------------
            if msg_id == "NAV-RELPOSNED":
                # Save the heading accuracy for the next PVT packet
                current_heading_acc = parsed_data.accHeading

                # Process Heading
                relPosHeading = parsed_data.relPosHeading
                
                if relPosHeading == 0:
                    relPosHeadingRover = previousHeadingRover
                else:
                    # Subtract 90 to align to rover front (adjust if needed)
                    # Note: pyubx2 usually scales this to degrees automatically
                    relPosHeadingRover = (relPosHeading - 90) % 360
                    previousHeadingRover = relPosHeadingRover

                # LOGGING: RoveComm Compass Packet
                logger.info(f"[TX Compass] Heading: {relPosHeadingRover:.4f}")

                # Send Compass Data
                packet = RoveCommPacket(
                    manifest["Nav"]["Telemetry"]["CompassData"]["dataId"], 
                    "f", 
                    (relPosHeadingRover,)
                )
                rovecomm_node.write(packet, False)

            # ------------------------------------------------------------------
            # 2. NAV-PVT: Position, Velocity, Time
            # ------------------------------------------------------------------
            elif msg_id == "NAV-PVT":
                lat = parsed_data.lat
                lon = parsed_data.lon
                alt = parsed_data.hMSL
                hAcc = parsed_data.hAcc
                vAcc = parsed_data.vAcc
                fix_type = parsed_data.fixType
                diff = parsed_data.difSoln

                if all(v is not None for v in [lat, lon, alt, hAcc, vAcc]):
                    
                    # Convert to meters/standard units for RoveComm
                    rc_alt = alt / 1000.0
                    rc_hAcc = hAcc / 1000.0
                    rc_vAcc = vAcc / 1000.0

                    # LOGGING: RoveComm GPS Packet
                    # Logs every field being sent in the packet below
                    logger.info(
                        f"[TX GPS] Lat: {lat:.7f}, Lon: {lon:.7f}, Alt: {rc_alt:.3f}, "
                        f"HAcc: {rc_hAcc:.3f}, VAcc: {rc_vAcc:.3f}, "
                        f"HeadAcc: {current_heading_acc:.3f}, Fix: {fix_type}, Diff: {diff}"
                    )

                    # We inject 'current_heading_acc' (from RELPOSNED) here
                    packet = RoveCommPacket(
                        manifest["Nav"]["Telemetry"]["GPSLatLonAlt"]["dataId"], 
                        "d", 
                        (lat, lon, rc_alt, rc_hAcc, rc_vAcc, current_heading_acc, fix_type, diff)
                    )
                    rovecomm_node.write(packet, False)

            # ------------------------------------------------------------------
            # 3. NAV-SAT: Satellite Information
            # ------------------------------------------------------------------
            elif msg_id == "NAV-SAT":
                numSvs = parsed_data.numSvs
                
                # LOGGING: RoveComm Sat Count Packet
                logger.info(f"[TX SatCount] NumSvs: {numSvs}")

                packet = RoveCommPacket(
                    manifest["Nav"]["Telemetry"]["SatelliteCountData"]["dataId"], 
                    "B", 
                    (numSvs,)
                )
                rovecomm_node.write(packet, False)

    except KeyboardInterrupt:
        logger.info("Terminated by user")
    finally:
        # Cleanup
        rovecomm_node.close_thread()
        if serial_stream.is_open:
            serial_stream.close()

if __name__ == "__main__":
    main()