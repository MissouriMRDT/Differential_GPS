# Specify environment.
#!/usr/bin/env python

import argparse
import logging
import logging.config
import yaml
import utm
import rich
import serial
from threading import Thread, Event, Lock
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
    Returns
    -------
        Logger: root set up for console and file logging
    """
    # logging file
    yaml_conf = yaml.safe_load(open("/home/pi/Differential_GPS/logging.yaml", "r").read())
    logging.config.dictConfig(yaml_conf)

    for handler in logging.getLogger().handlers:
        if isinstance(handler, type(rich.logging.RichHandler())):
            handler.setLevel(level)

    return logging.getLogger()

def flush_serial(serial):
    """
    Flushes the given serial object.

    Params:
    -------
    serial - the pyserial object.
    """
    # Clear and reset everything.
    serial.flush()
    serial.flushInput()
    serial.flushOutput()
    serial.reset_input_buffer()
    serial.reset_output_buffer()

def read_data(
    stream: object,
    ubr: pyubx2.UBXReader,
    queue: Queue,
    lock: Lock,
    stop: Event,
):
    """
    Read and parse incoming UBX data and place
    raw and parsed data on queue
    """
    # pylint: disable=unused-variable, broad-except
    # Check if thread should stop.
    while not stop.is_set():
        # See if the serial stream has any data for us.
        if stream.inWaiting() > 0:
            try:
                # Get thread lock for resource.
                lock.acquire()
                # Read and parse data.
                (raw_data, parsed_data) = ubr.read()
                # Release lock.
                lock.release()
                # Store data in queue.
                if parsed_data:
                    queue.put(("", parsed_data))
            except Exception as err:
                pass

def main() -> None:
    # Create argument parser.
    parser = argparse.ArgumentParser()

    # Add arguments.
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

    # Get arguments.
    args = parser.parse_args()
    # Make sure given logging level makes sense.
    if (level := getattr(logging, args.level, -1)) < 0:
        parser.print_help()
        exit(1)

    # Setup the logger, also pass-in optional logging level for console output
    logger = setup_logger(level)

    # Get rovecomm manifest.
    manifest = get_manifest()
    # Initialize the rovecomm node
    rovecomm_node = RoveComm()

    # Setup serial input and ubx decoder.
    serial_stream = serial.Serial(port=args.serial_path, baudrate=SERIAL_BAUD, timeout=0.1)
    ubx_decode = pyubx2.UBXReader(serial_stream, protfilter=pyubx2.UBX_PROTOCOL, validate=1)

    # UBX enumerator constants.
    NAV_FIX_TYPE = Enum("NAV_FIX_TYPE", ["FIX_TYPE_NO_FIX", "FIX_TYPE_DEAD_RECKONING_ONLY", "FIX_TYPE_2D", "FIX_TYPE_3D", "FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED", "FIX_TYPE_TIME_ONLY"])

    # Create queues, locks, and events for threaded reading.
    serial_lock = Lock()
    read_queue = Queue()
    stop_event = Event()
    # Create read thread.
    read_thread = Thread(
        target=read_data,
        args=(
            serial_stream,
            ubx_decode,
            read_queue,
            serial_lock,
            stop_event,
        ),
    )
    logger.info("Starting read handler processes. Press Ctrl-C to terminate...")
    read_thread.start()

    # Create an array to track which messages have been properly read.
    msg_success_array = [0, 0, 0]

    # Check for user interupt.
    try:
        # Create instance variables.
        hAcc, vAcc, accurHeading, lat, lon = None, None, None, None, None
        # Loop forever.
        while True:
            # Wait until there is something in the serial port to read.
            if serial_stream.in_waiting:
                # Decode current message.
                raw_data, parsed_data = read_queue.get()
                
                # Check if serial message was recieved properly.
                if isinstance(parsed_data, str) and "UNKNOWN PROTOCOL" in parsed_data:
                    # Print warning.
                    logger.warning("Serial Message not recieved properly.")
                    # Clear buffers.
                    serial_lock.acquire()
                    flush_serial(serial_stream)
                    serial_lock.release()
                elif parsed_data is not None and not isinstance(parsed_data, str):
                    # Check if message is Navigation Position Velocity Time.
                    if parsed_data.identity == "NAV-PVT":
                        # Get data from parser.
                        lat, lon, alt, hAcc, vAcc, fix_type, diff = parsed_data.lat, parsed_data.lon, parsed_data.hMSL, parsed_data.hAcc, parsed_data.vAcc, parsed_data.fixType, parsed_data.difSoln
                        # Convert rover lat long to UTM.
                        meter_loc = utm.from_latlon(lat, lon)
                        logger.info(f"UTM LATLON Pos: {meter_loc}")
                        # Send RoveComm Packets.
                        packet = RoveCommPacket(manifest["Nav"]["Telemetry"]["GPSLatLon"]["dataId"], "f", (lat, lon))
                        rovecomm_node.write(packet, False)
                        # Logger info.
                        logger.info(f"NAV_PVT: lat = {lat}, lon = {lon}, alt = {alt / 1000} m, horizontal_accur = {hAcc / 1000} m, vertical_accur = {vAcc / 1000} m, fix_type = {NAV_FIX_TYPE(fix_type + 1)}, diff? = {bool(diff)}")
                        # Increment msg array.
                        msg_success_array[0] += 1
                    # Check if message is Relative Positioning Information in NED frame
                    if parsed_data.identity == "NAV-RELPOSNED":
                        # Get data from parser.
                        relPosHeading, accurHeading = parsed_data.relPosHeading, parsed_data.accHeading
                        # Send RoveComm Packets.
                        packet = RoveCommPacket(manifest["Nav"]["Telemetry"]["CompassData"]["dataId"], "f", (-relPosHeading,))
                        rovecomm_node.write(packet, False)
                        packet = RoveCommPacket(manifest["Nav"]["Telemetry"]["IMUData"]["dataId"], "f", (0, -relPosHeading, 0))
                        rovecomm_node.write(packet, False)
                        # Logger info.
                        logger.info(f"NAV-RELPOSNED: relative_position_heading = {relPosHeading}, heading_accur = {accurHeading}")
                        # Increment msg array.
                        msg_success_array[1] += 1
                    # Check if message is Satelite Information
                    if parsed_data.identity == "NAV-SAT":
                        # Get data from parser.
                        gps_time, numSvs = parsed_data.iTOW, parsed_data.numSvs
                        # Send RoveComm Packets.
                        packet = RoveCommPacket(manifest["Nav"]["Telemetry"]["SatelliteCountData"]["dataId"], "h", (numSvs,))
                        rovecomm_node.write(packet, False)
                        # Logger info.
                        logger.info(f"NAV-SAT: gps_time = {gps_time} ms, num_sats = {numSvs}")
                        # Increment msg array.
                        msg_success_array[2] += 1

                    # Check if all accuracy data has been retrieved at least once.
                    if None not in (hAcc, vAcc, accurHeading):
                        # Put send accuracy data over RoveComm.
                        packet = RoveCommPacket(manifest["Nav"]["Telemetry"]["AccuracyData"]["dataId"], "f", (hAcc / 1000, vAcc / 1000, accurHeading))
                        rovecomm_node.write(packet, False)

            # If all messages have been sent at least once reset the serial buffer.
            if all(i > 1 for i in msg_success_array):
                # Clear buffers.
                serial_lock.acquire()
                flush_serial(serial_stream)
                serial_lock.release()
                # Reset array.
                msg_success_array = [0, 0, 0]
            # Check if at least one number is greater than a max value.
            if any(i > 10 for i in msg_success_array):
                # Clear buffers.
                serial_lock.acquire()
                flush_serial(serial_stream)
                serial_lock.release()
                # Reset array.
                msg_success_array = [0, 0, 0]

    except KeyboardInterrupt:
        print("Terminated by user")
        stop_event.set()

    # Cleanup.
    read_thread.join()
    rovecomm_node.close_thread()
    serial_stream.close()

if __name__ == "__main__":
    # Run main function.
    main()
