import argparse
import math

from data_utils import load_driving_data, data_generator
from keras.models import Sequential
from keras.layers import Conv2D, Cropping2D, Dense, Dropout, Flatten, Lambda
from sklearn.model_selection import train_test_split


def main(args):
    driving_data = load_driving_data(args.logs_input_dir)

    # shuffle and split driving data--use 80% for training and 20% for validation
    train_samples, validation_samples = train_test_split(driving_data, test_size=0.2)

    ######################
    ## Define the model ##
    ######################
    model = Sequential()
    # cropping layer--crop image 50 rows pixels from the top and 20 rows pixels from the bottom
    model.add(Cropping2D(cropping=((50, 20), (0, 0)), input_shape=(160, 320, 3)))
    # normalize data--divide each element by 255 and mean center it by subtracting 0.5
    model.add(Lambda(lambda x: (x / 255.0) - 0.5))
    # convolutional layers
    model.add(Conv2D(filters=24, kernel_size=(5, 5), strides=(2, 2), activation="elu"))
    model.add(Conv2D(filters=36, kernel_size=(5, 5), strides=(2, 2), activation="elu"))
    model.add(Conv2D(filters=48, kernel_size=(5, 5), strides=(2, 2), activation="elu"))
    model.add(Conv2D(filters=64, kernel_size=(3, 3), activation="elu"))
    model.add(Conv2D(filters=64, kernel_size=(3, 3), activation="elu"))
    # dropout layer to avoid overfitting
    model.add(Dropout(0.5))
    # flatten layer
    model.add(Flatten())
    # fully connected layers
    model.add(Dense(100, activation="elu"))
    model.add(Dense(50, activation="elu"))
    model.add(Dense(10, activation="elu"))
    model.add(Dense(1))

    model.summary()
    model.compile(optimizer="adam", loss="mean_squared_error")
    model.fit_generator(
        generator=data_generator(train_samples, args.batch_size),
        steps_per_epoch=math.ceil(len(train_samples) / args.batch_size),
        validation_data=data_generator(validation_samples, args.batch_size),
        validation_steps=math.ceil(len(validation_samples) / args.batch_size),
        epochs=args.epochs,
        verbose=1,
    )

    model.save("model.h5")
    print("Model saved.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Input arguments for the script.")
    parser.add_argument(
        "--logs_input_dir",
        type=str,
        required=True,
        help="Absolute path to the driving logs' parent directory.",
    )
    parser.add_argument(
        "--batch_size",
        type=int,
        required=False,
        default=64,
        help="Number of samples to propagate through the network.",
    )
    parser.add_argument(
        "--epochs",
        type=int,
        required=False,
        default=5,
        help="Number of cycles through the full training dataset.",
    )
    args = parser.parse_args()

    main(args)
