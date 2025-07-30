"""
MIT BWSI Autonomous RACECAR
MIT License

File Name: img-counter.py

Title: Image Counter

Author: Claire Shen, Tianxi Liang, Aidan Wong, Krista Sebastian
"""

import os
import sys

# Dictionary of folder names w/ keys
types = {1: "do not enter", 2: "fake go around", 3: "fake_stop", 4: "fake_yield", 
        5: "go around", 6: "one way left", 7: "one way right", 8: "stop_signs", 9: "yield"}

for i in range(1,10): # runs loop once for each id in types
    # defines count as the number of existing files in folder corresponding to ID key
    count = sum(1 for entry in os.scandir(f"data_collection/{types[i]}") if entry.is_file())
    print(f"{types[i]} count: {count}") # prints folder name and count