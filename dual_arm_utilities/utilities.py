import os
import csv
import matplotlib.pyplot as plt
from datetime import datetime
import time
def create_csv(folder_path, data):
    os.makedirs(folder_path, exist_ok=True)
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    file_name = f"{data}_{timestamp}.csv"
    full_path = os.path.join(folder_path, file_name)
    if not os.path.exists(full_path):
            with open(full_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                #writer.writerow(['{data}_{}'.format(i) for i in range(14)])
    else:
         print(f"File already exists:{full_path}")
    return full_path