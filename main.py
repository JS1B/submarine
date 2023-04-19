import re
import csv

# Get data txt file and parse it to csv
def parse_robot_data(filename, names = None):
    if names == None:
        names = ["position", "time", "speed", "direction"]
    with open(filename, mode="r") as f:
        data = f.read().replace('\r\n', '')
        data = re.findall(r"(\d+\.\d+), (\d+\.\d+), (\d+), (True|False)", data)
        data = [dict(zip(names, val)) for val in data]
        return data

def save_robot_data(filename, data, names = None):
    with open(filename, mode="w", newline='') as f:
        if names == None:
            names = ["position", "time", "speed", "direction"]
        writer = csv.DictWriter(f, fieldnames=names)
        writer.writeheader()
        writer.writerows(data)

def main():
    test_names = ["position", "time", "speed", "direction"]
    try:
        file_name = "model_no_syringe"
        data = parse_robot_data(file_name+".txt", test_names)
        save_robot_data(file_name+".csv", data)
    except FileNotFoundError:
        print("File not found!")
        exit(1)
    except Exception as e:
        print(e)
        exit(2)
    else:
        print("Done!")

if __name__ == "__main__":
    main()