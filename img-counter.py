import os
import sys
count = sum(1 for entry in os.scandir("data_collection/go around") if entry.is_file())
types = {1:"do not enter", 2:"fake go around", 3:"fake_stop", 4:"fake_yield", 
        5: "go around", 6: "one way left", 7:"one way right", 8:"stop_signs", 9:"yield"}
for i in range(1,10):
    count = sum(1 for entry in os.scandir(f"data_collection/{types[i]}") if entry.is_file())
    print(f"{types[i]} count: {count}")