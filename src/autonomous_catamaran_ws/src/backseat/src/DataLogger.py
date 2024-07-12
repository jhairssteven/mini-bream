import csv, time, os, pathlib

class DataLogger:
    def __init__(self, data_description, headers=[]):
        cur_file_path = str(pathlib.Path(__file__).parent.resolve())
        log_dir = cur_file_path + f'/log/{data_description}/'
        fn = time.strftime("%Y%m%d-%H%M%S") + '.csv'
        if not os.path.exists(log_dir):
                os.makedirs(log_dir)
        fn = log_dir + fn 

        self.filename = fn
        self.file = open(self.filename, 'w', newline='')
        self.csv_writer = csv.writer(self.file)
        self.csv_writer.writerow(['timestamp'] + headers)  # Write the headers

    def log_data(self, vars):
        timestamp = time.time()
        self.csv_writer.writerow([timestamp] + vars)
        self.file.flush()  # Ensure data is written to the file in real-time

    def close(self):
        self.file.close()

if __name__ == '__main__':
    dl = DataLogger("test", ["kk", 2])
    dl.log_data([1, "hola", 8, 0000])