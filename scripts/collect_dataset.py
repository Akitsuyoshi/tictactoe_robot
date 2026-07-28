import os
import random
import subprocess
import time
import glob
import argparse

parser = argparse.ArgumentParser()

parser.add_argument(
    "split",
    choices=["train", "valid"],
    help="Dataset split"
)

parser.add_argument(
    "count",
    type=int,
    help="Number of images to generate"
)

args = parser.parse_args()

TOTAL_IMAGES = args.count

IMAGE_DIR = f"/home/user/dataset/images/{args.split}"
LABEL_DIR = f"/home/user/dataset/labels/{args.split}"

os.makedirs(IMAGE_DIR, exist_ok=True)
os.makedirs(LABEL_DIR, exist_ok=True)


# Delete previous images
for image in glob.glob(os.path.join(IMAGE_DIR, "*.jpg")):
    os.remove(image)

# Delete previous labels
for label in glob.glob(os.path.join(LABEL_DIR, "*.txt")):
    os.remove(label)

print("Previous dataset deleted.")

IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080

cells = [
    # Top row
    (770, 390),
    (950, 390),
    (1130, 390),

    # Middle row
    (740, 530),
    (955, 530),
    (1160, 530),

    # Bottom row
    (710, 695),
    (950, 695),
    (1190, 695),
]

mapping = {
    None: "E",
    "cross": "X",
    "circle": "O"
}


def generate_random_board():
    return [
        random.choices(
            ["cross", "circle", None], 
            weights=[40, 35, 25], 
            k=1
        )[0]
        for _ in range(9)
    ]


def board_to_string(board):
    return "".join(
        mapping[x]
        for x in board
    )


def create_yolo_label(board, filename):
    labels = []

    table_cx = 955.0
    table_cy = 600.0
    table_w  = 1050.0
    table_h  = 800.0

    labels.append(
        f"0 "
        f"{table_cx / IMAGE_WIDTH:.6f} "
        f"{table_cy / IMAGE_HEIGHT:.6f} "
        f"{table_w / IMAGE_WIDTH:.6f} "
        f"{table_h / IMAGE_HEIGHT:.6f}"
    )

    for index, obj in enumerate(board):
        if obj is None:
            continue

        x, y = cells[index]

        if obj == "cross":
            cls = 1
        else:
            cls = 2
        
        if 0 <= index <= 2:
            w = 125
            h = 125
        elif 3 <= index <= 5:
            w = 135
            h = 135
        else:
            w = 150
            h = 150

        labels.append(
            f"{cls} "
            f"{x / IMAGE_WIDTH:.6f} "
            f"{y / IMAGE_HEIGHT:.6f} "
            f"{w / IMAGE_WIDTH:.6f} "
            f"{h / IMAGE_HEIGHT:.6f}"
        )

    with open(
        os.path.join(
            LABEL_DIR,
            filename + ".txt"
        ),
        "w"
    ) as f:
        f.write("\n".join(labels))

def spawn_objects(board_string):
    subprocess.run(
        [
            "ros2",
            "run",
            "helper_scripts",
            "spawn_objects_node",
            "--ros-args",
            "-p",
            f"board:={board_string}"
        ], 
        check = True
    )


def capture_image():
    subprocess.run(
        [
            "ros2",
            "run",
            "helper_scripts",
            "save_image_node",
            "--ros-args",
            "-p",
            f"split:={args.split}"
        ],
        check=True
    )

def delete_objects():
    subprocess.run(
        [
            "ros2",
            "run",
            "helper_scripts",
            "delete_objects_node"
        ],
        check=True
    )


for i in range(TOTAL_IMAGES):

    print("="*40)
    print(f"Image {i+1}/{TOTAL_IMAGES}")

    board = generate_random_board()
    print(board)

    board_string = board_to_string(board)
    print(board_string)

    spawn_objects(board_string)    
    time.sleep(1)

    capture_image()

    create_yolo_label(
        board,
        f"image_{i:05d}"
    )

    delete_objects()
    time.sleep(1)

print("Finished.")