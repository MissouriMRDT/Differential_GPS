from logging.handlers import WatchedFileHandler
from datetime import datetime
import os


class CsvHandler(WatchedFileHandler):
    def __init__(self, filename, header, encoding=None, delay=False, new_file=False):
        """
        Initializes the handler
        """

        if new_file:
            timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            if not os.path.exists("/".join(f"{filename[:-4]}-{timestamp}{filename[-4:]}".split("/")[:-1])):
                os.makedirs("/".join(f"{filename[:-4]}-{timestamp}{filename[-4:]}".split("/")[:-1]))
            f = open(f"{filename[:-4]}-{timestamp}{filename[-4:]}", "w")
            f.write(header + "\n")
            f.close()
            WatchedFileHandler.__init__(
                self,
                f"{filename[:-4]}-{timestamp}{filename[-4:]}",
                "a",
                encoding,
                delay,
            )
        else:
            if not os.path.exists(filename):
                if not os.path.exists("/".join(filename.split("/")[:-1])):
                    os.makedirs("/".join(filename.split("/")[:-1]))
                f = open(filename, "w")
                f.write(header + "\n")
                f.close()
            WatchedFileHandler.__init__(self, filename, "a", encoding, delay)
