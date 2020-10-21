import csv
import cv2
import matplotlib.pyplot as plt
import numpy as np

from pathlib import Path
from sklearn.utils import shuffle


def save_histogram_figure(data, xlabel, ylabel, output_path, bins=20):
    plt.figure()
    hist, bins = np.histogram(data, bins)
    plt.bar(
        x=(bins[:-1] + bins[1:]) / 2,  # center of the bins
        height=hist,
        width=0.9 * (bins[1] - bins[0]),
        align="center",
    )
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.savefig(output_path)


def load_driving_data(input_dir):
    """
    Function reads all "driving_log.csv" files from input directory and its subdirectories.
    It returns numpy array with driving data.
    It's presumed that paths to the camera images in "driving_log.csv" files are correct.
    Args:
        input_dir: Absolute path to the driving logs' parent directory.
    """

    driving_log_csv_files = []
    for path in Path(input_dir).rglob("driving_log.csv"):
        driving_log_csv_files.append(path)

    driving_data = []
    angle_data_total = []  # used for histogram
    angle_data_filtered = []  # used for histogram
    for driving_log in driving_log_csv_files:
        with open(driving_log) as f:
            reader = csv.reader(f)
            next(reader)  # skip first line
            for line in reader:
                # filter out majority of 0 degree angles to get more balanced dataset
                angle = float(line[3])
                angle_data_total.append(angle)
                if angle == 0 and np.random.uniform() <= 0.9:
                    continue
                angle_data_filtered.append(angle)
                driving_data.append(line)

    save_histogram_figure(
        angle_data_total,
        "Steering Angle",
        "Frequency",
        "./resources/angle_data_total_distribution.png",
    )

    save_histogram_figure(
        angle_data_filtered,
        "Steering Angle",
        "Frequency",
        "./resources/angle_data_filtered_distribution.png",
    )

    return np.array(driving_data)


def data_generator(samples, batch_size, steering_angle_correction=0.2):
    num_samples = len(samples)
    while True:  # loop forever so the generator never terminates
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset : offset + batch_size]

            images = []
            steering_angles = []
            for batch_sample in batch_samples:

                cent_img = cv2.cvtColor(cv2.imread(batch_sample[0]), cv2.COLOR_BGR2RGB)
                cent_flipped_img = cv2.flip(cent_img, flipCode=1)
                left_img = cv2.cvtColor(cv2.imread(batch_sample[1]), cv2.COLOR_BGR2RGB)
                left_flipped_img = cv2.flip(left_img, flipCode=1)
                right_img = cv2.cvtColor(cv2.imread(batch_sample[2]), cv2.COLOR_BGR2RGB)
                right_flipped_img = cv2.flip(right_img, flipCode=1)

                steering_center = float(batch_sample[3])
                steering_left = steering_center + steering_angle_correction
                steering_right = steering_center - steering_angle_correction

                # add original and flipped images to the list
                images.extend(
                    [
                        cent_img,
                        cent_flipped_img,
                        left_img,
                        left_flipped_img,
                        right_img,
                        right_flipped_img,
                    ]
                )
                # add steering angles for original and flipped images to the list
                steering_angles.extend(
                    [
                        steering_center,
                        -steering_center,
                        steering_left,
                        -steering_left,
                        steering_right,
                        -steering_right,
                    ]
                )

            X_train = np.array(images)
            y_train = np.array(steering_angles)
            yield shuffle(X_train, y_train)
