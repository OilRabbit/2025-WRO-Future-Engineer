import numpy as np
import random

def determine_starting_sector():
	combo = tuple(np.random.randint(2, size = 2))
	zone_map = {
        (0, 0): "Up",
        (0, 1): "Right",
        (1, 0): "Left",
        (1, 1): "Down"
	}
	return zone_map[combo]

def determine_corridor_size(startzone):
	directions = ["Up", "Right", "Down", "Left"]
	combo = tuple(np.random.randint(2, size=4))
	start_idx = directions.index(startzone)
	rotated_dirs = directions[start_idx:] + directions[:start_idx]
	size_map = {dir_: "Wide" if bit else "Narrow" for dir_, bit in zip(rotated_dirs, combo)}
	return size_map	

def determine_starting_zone():
	return np.random.randint(7)

def determine_single_traffic():
	return np.random.randint(2)

def determine_traffic_colour():
	return np.random.randint(2)

which_oc = int(input("Input the number to randomize the field for the corresponding OC \n1. OC1   2. OC2 \n"))
print("\n")

if which_oc == 1:
	startsector = determine_starting_sector()
	print("Starting sector is {}".format(startsector))
	print("Inner barrier size:")
	print(determine_corridor_size(startsector))
	print("Starting zone is {} in {} direction".format(determine_starting_zone(), "clockwise" if not np.random.randint(2) else "anticlockwise"))
elif which_oc == 2:
	single_colourblk = np.random.randint(2)  # 0 = Red, 1 = Green
	single_blk_pos = determine_starting_sector()  # e.g., "Right"
	directions = ["Up", "Right", "Down", "Left"]
	exclude_number = 10 if single_colourblk == 0 else 9
	available_numbers = [i for i in range(1, 37) if i != exclude_number]
	selected_numbers = random.sample(available_numbers, 4)
	start_idx = directions.index(single_blk_pos)
	rotated_dirs = directions[start_idx:] + directions[:start_idx]
	number_assignment = {dir_: num for dir_, num in zip(rotated_dirs, selected_numbers)}
	print("Place a single {} block in the middle of {} furthest from the inner wall".format("Red" if not single_colourblk else "Green", single_blk_pos))
	print("Assigned numbers:", number_assignment)
else:
	print("Invalid input. Please enter 1 or 2.")



