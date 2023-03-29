import os
import re
import numpy as np

def scan_txt_files(folder_path):
    txt_files = [f for f in os.listdir(folder_path) if f.endswith('.txt')]
    return txt_files

def user_select_file(txt_files):
    print("Select a file:")
    for index, file in enumerate(txt_files):
        print(f"{index}: {file}")

    selected_index = int(input("Enter the index of the selected file: "))
    return txt_files[selected_index]

def process_file(file_path):
    latencies = {
        "Freq shift time": [],
        "Freq return time": [],
        "Srate change time": [],
        "Samples recv time": [],
        "Https req time": [],
        "FFT time": []
    }

    with open(file_path, "r") as f:
        contents = f.read()

    for key in latencies:
        pattern = re.compile(f"{key}: (\\d+) us")
        matches = pattern.findall(contents)
        latencies[key] = [int(match) for match in matches]

    return latencies

def print_statistics(latencies):
    for key, values in latencies.items():
        if len(values) > 0:
            num_samples = len(values)
            average = np.mean(values)
            std_dev = np.std(values)
            print(f"{key}:")
            print(f"  Number of samples: {num_samples}")
            print(f"  Average: {average:.2f} us")
            print(f"  Standard deviation: {std_dev:.2f} us")
        else:
            print(f"{key}:")
            print(f"  No data available")

if __name__ == "__main__":
    folder_path = '.'  # Set the folder path here
    txt_files = scan_txt_files(folder_path)

    if len(txt_files) > 0:
        selected_file = user_select_file(txt_files)
        file_path = os.path.join(folder_path, selected_file)
        latencies = process_file(file_path)

        print("\nStatistics:")
        print_statistics(latencies)
    else:
        print("No .txt files found in the specified folder.")
