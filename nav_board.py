import argparse
import logging
import logging.config
import yaml
import rich
from serial import Serial
from enum import Enum
import pyubx2
from RoveComm_Python.rovecomm import RoveComm, RoveCommPacket, get_manifest

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
    serial_stream = Serial(args.serial_path, 38400, timeout=3)
    ubx_decode = pyubx2.UBXReader(serial_stream)

    # UBX enumerator constants.
    NAV_FIX_TYPE = Enum("NAV_FIX_TYPE", ["FIX_TYPE_NO_FIX", "FIX_TYPE_DEAD_RECKONING_ONLY", "FIX_TYPE_2D", "FIX_TYPE_3D", "FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED", "FIX_TYPE_TIME_ONLY"])

    # Check for user interupt.
    try:
        # Loop forever.
        while True:
            # Decode current message.
            (raw_data, parsed_data) = ubx_decode.read()
            
            # Check if serial message was recieved properly.
            if isinstance(parsed_data, str) and "UNKNOWN PROTOCOL" in parsed_data:
                logger.error("Serial Message not recieved properly.")
            else:
                # Check if message is Navigation Position Velocity Time.
                if parsed_data.identity == "NAV-PVT":
                    lat, lon, alt, hAcc, vAcc, headVeh, magDec, fix_type, diff = parsed_data.lat, parsed_data.lon, parsed_data.hMSL, parsed_data.hAcc, parsed_data.vAcc, parsed_data.headVeh, parsed_data.magDec, parsed_data.fixType, parsed_data.difSoln
                    logger.info(f"NAV_PVT: lat = {lat}, lon = {lon}, alt = {alt / 1000} m, horizontal_acc = {hAcc / 1000}, vertical_acc = {vAcc / 1000}, vehicle_heading = {headVeh / 1000}, magnetic_declination = {magDec}, fix_type = {NAV_FIX_TYPE(fix_type + 1)}, diff? = {bool(diff)}")
                # Check if message is Relative Positioning Information in NED frame
                if parsed_data.identity == "NAV-RELPOSNED":
                    relPosHPN, relPosHPE, relPosHPD = parsed_data.relPosHPN, parsed_data.relPosHPE, parsed_data.relPosHPD
                    logger.info(f"NAV-RELPOSNED: relative_pos_highAccNorth = {relPosHPN}, relative_pos_highAccEast = {relPosHPE}, relative_pos_highAccDown = {relPosHPD}")

            
    except KeyboardInterrupt:
        print("Terminated by user")

    # Cleanup.
    rovecomm_node.close_thread()
    serial_stream.close()

if __name__ == "__main__":
    # Run main function.
    main()