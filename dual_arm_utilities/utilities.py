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
    return full_path