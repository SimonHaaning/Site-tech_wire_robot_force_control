import logging
import os

# creating a class that can same info to specific log file, that saves info in log folder
class Filelogger:
    def __init__(self, Foldername, unit_name):
        while len(logging.getLogger(unit_name).handlers) != 0:
            logging.getLogger(unit_name).removeHandler(logging.getLogger(unit_name).handlers[0])
        self.log_filename = None
        self.Folder = Foldername
        self.File = unit_name
        self.log_filename = 'logs\\'+self.Folder + '\\' + self.File + '.log'
        os.makedirs(os.path.dirname(self.log_filename), exist_ok=True)
        self.This_logger = logging.getLogger(self.File)
        self.Handler = logging.FileHandler(self.log_filename, mode='w')
        self.Handler.setFormatter(logging.Formatter('%(asctime)s , %(message)s'))
        self.This_logger.setLevel(logging.INFO)
        self.This_logger.addHandler(self.Handler)

# Function for saving data to the logger file
    def save(self, data):
        self.This_logger.info(data)




