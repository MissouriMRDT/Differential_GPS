import argparse
import logging
import logging.config
import yaml
import utm
import rich
import serial
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
    yaml_conf = yaml.safe_load(open("logging.yaml", "r").read())
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
    serial_stream = serial.Serial(port=args.serial_path, baudrate=SERIAL_BAUD, timeout=0.001)
    # ubx_decode = pyubx2.UBXReader(serial_stream)

    # UBX enumerator constants.
    NAV_FIX_TYPE = Enum("NAV_FIX_TYPE", ["FIX_TYPE_NO_FIX", "FIX_TYPE_DEAD_RECKONING_ONLY", "FIX_TYPE_2D", "FIX_TYPE_3D", "FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED", "FIX_TYPE_TIME_ONLY"])

    # Check for user interupt.
    try:
        # Create instance variables.
        hAcc, vAcc, accurHeading, lat, lon = None, None, None, None, None
        # Loop forever.
        while True:
            # Wait until there is something in the serial port to read.
            if serial_stream.inWaiting() > 0:
                # Decode current message.
                # (raw_data, parsed_data) = ubx_decode.read()
                try:
                    parsed_data = pyubx2.UBXReader.parse(serial_stream.readline(), validate=pyubx2.VALCKSUM)
                except:
                    parsed_data = "UNKNOWN PROTOCOL"
                
                # Check if serial message was recieved properly.
                if isinstance(parsed_data, str) and "UNKNOWN PROTOCOL" in parsed_data:
                    # Print warning.
                    logger.warning("Serial Message not recieved properly.")
                    # Flush serial bus.
                    flush_serial(serial_stream)
                elif parsed_data is not None and not isinstance(parsed_data, str):
                    # Check if message is Navigation Position Velocity Time.
                    if parsed_data.identity == "NAV-PVT":
                        # Get data from parser.
                        lat, lon, alt, hAcc, vAcc, fix_type, diff = parsed_data.lat, parsed_data.lon, parsed_data.hMSL, parsed_data.hAcc, parsed_data.vAcc, parsed_data.fixType, parsed_data.difSoln
                        # Convert rover lat long to UTM.
                        meter_loc = utm.from_latlon(lat, lon)
                        # logger.info(f"UTM LATLON Pos: {meter_loc}")
                        # Send RoveComm Packets.
                        packet = RoveCommPacket(manifest["Nav"]["Telemetry"]["GPSLatLon"]["dataId"], "f", (lat, lon))
                        rovecomm_node.write(packet, False)
                        # Flush serial bus.
                        flush_serial(serial_stream)
                        # Logger info.
                        # logger.info(f"NAV_PVT: lat = {lat}, lon = {lon}, alt = {alt / 1000} m, horizontal_accur = {hAcc / 1000} m, vertical_accur = {vAcc / 1000} m, fix_type = {NAV_FIX_TYPE(fix_type + 1)}, diff? = {bool(diff)}")
                    # Check if message is Relative Positioning Information in NED frame
                    if parsed_data.identity == "NAV-RELPOSNED":
                        # Get data from parser.
                        relPosHeading, accurHeading = parsed_data.relPosHeading, parsed_data.accHeading
                        # Send RoveComm Packets.
                        packet = RoveCommPacket(manifest["Nav"]["Telemetry"]["CompassData"]["dataId"], "f", (-relPosHeading,))
                        rovecomm_node.write(packet, False)
                        packet = RoveCommPacket(manifest["Nav"]["Telemetry"]["IMUData"]["dataId"], "f", (0, -relPosHeading, 0))
                        rovecomm_node.write(packet, False)
                        # Flush serial bus.
                        flush_serial(serial_stream)
                        # Logger info.
                        logger.info(f"NAV-RELPOSNED: relative_position_heading = {relPosHeading}, heading_accur = {accurHeading}")
                    # Check if message is Satelite Information
                    if parsed_data.identity == "NAV-SAT":
                        # Get data from parser.
                        gps_time, numSvs = parsed_data.iTOW, parsed_data.numSvs
                        # Send RoveComm Packets.
                        packet = RoveCommPacket(manifest["Nav"]["Telemetry"]["SatelliteCountData"]["dataId"], "h", (numSvs,))
                        rovecomm_node.write(packet, False)
                        # Flush serial bus.
                        flush_serial(serial_stream)
                        # Logger info.
                        # logger.info(f"NAV-SAT: gps_time = {gps_time} ms, num_sats = {numSvs}")

                    # Check if all accuracy data has been retrieved at least once.
                    if None not in (hAcc, vAcc, accurHeading):
                        # Put send accuracy data over RoveComm.
                        packet = RoveCommPacket(manifest["Nav"]["Telemetry"]["AccuracyData"]["dataId"], "f", (hAcc / 1000, vAcc / 1000, accurHeading))
                        rovecomm_node.write(packet, False)

    except KeyboardInterrupt:
        print("Terminated by user")

    # Cleanup.
    rovecomm_node.close_thread()
    serial_stream.close()

if __name__ == "__main__":
    # Run main function.
    main()
