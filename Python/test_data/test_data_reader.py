import csv
import numpy as np


def reader_function(folder_name, file_name):
    # list for read data
    data = []

    with open(folder_name + file_name) as csv_file:
        print(csv_file.name)
        reader = csv.reader(csv_file, delimiter=',')

        for row in reader:
            numpy_values = []
            for item in row:
                # Convert string to numpy float array
                numpy_values.append(np.array(item.strip('[]').split(), dtype=float))
            data.append(np.array(numpy_values))

    return np.array(data)
