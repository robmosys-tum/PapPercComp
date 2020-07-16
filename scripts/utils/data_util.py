"""
Utils to create a TF2 Dataset for batch training.
"""
import os
from pathlib import Path

import numpy as np
import tensorflow as tf

AUTOTUNE = tf.data.experimental.AUTOTUNE


def get_label(file_path, classes):
    """
    Finds the label to given a filepath and list of classes like so:
    ../data/train/class1/img.jpg

    Args:
        file_path: The location of the image.
        classes: List of possible classes.

    Returns:
        label: The respective label.
    """
    parts = tf.strings.split(file_path, os.path.sep)
    return parts[-2] == classes


def decode_img(img, width, height):
    """
    Creates a tf usable image from soure image.

    Args:
        img: The given image path.
        width: The desired width.
        height: The desired hight.

    Returns:
        img: The prepared image.
    """
    img = tf.image.decode_image(img, channels=3, expand_animations=False)
    img = tf.image.convert_image_dtype(img, tf.float32)
    return tf.image.resize(img, [width, height])


def process_path(file_path, width, height, classes):
    """
    Creates a img with label from a given path.
    Wraps get_label and decode_img.

    Args:
        file_path: The location of the image.
        width: The desired width.
        height: The desired hight.
        classes: List of possible classes.

    Returns:
        img: The new image.
        label: The respective label.
    """
    label = get_label(file_path, classes)
    img = tf.io.read_file(file_path)
    img = decode_img(img, width, height)
    return img, label


def data_augment(img, label):
    """
    Augments the given image by flip and  saturation.

    Args:
        img: The given image.
        label: The respective label.

    Returns:
        img: The augmented image.
        label: The respective label.
    """
    img = tf.image.random_flip_left_right(img)
    img = tf.image.random_saturation(img, 0, 2)
    return img, label


def split_data(dataset, fraction):
    """
    Splits a dataset into train and validation set.

    Args:
        dataset: The initial dataset.
        fraction: The fraction of data to be used in the validation set.

    Returns:
        (t, t_len): Train set and its lenght.
        (v, v_len): Validation set an its length.
    """
    fraction = round(fraction * 100)

    dataset = dataset.enumerate()
    train_dataset = dataset.filter(lambda f, data: f % 100 > fraction)
    validation_dataset = dataset.filter(lambda f, data: f % 100 <= fraction)

    train_len = len(list(train_dataset.as_numpy_iterator()))
    validation_len = len(list(validation_dataset.as_numpy_iterator()))

    train_dataset = train_dataset.map(lambda f, data: data)
    validation_dataset = validation_dataset.map(lambda f, data: data)

    return (train_dataset, train_len), (validation_dataset, validation_len)


def create_train_val_set(data_dir,
                         split=0.2,
                         width=256,
                         height=256,
                         classes=None):
    """
    Wrapper function to prepare a dataset given a directory of data.
    Assumed directory structure ../data/train/class1/img.jpg
                                ../data/train/class2/img.png

    Args:
        data_dir: The path to the data.
        split: Fraction for validation split.
        width: Resize parameter for the images.
        height: Resize parameter for the images.
        classes: Possible list of classes. If not given will be read from directory.

    Returns:
        (t, t_len): Train set and its lenght.
        (v, v_len): Validation set an its length.
    """
    if not classes:
        classes = np.array([item.name for item in Path(data_dir).glob('*')])
    data_list = tf.data.Dataset.list_files(data_dir + '*/*')
    labeled_data = data_list.map(
        lambda x: process_path(x, width, height, classes),
        num_parallel_calls=AUTOTUNE)
    return split_data(labeled_data, split)


def prepare_for_training(dataset,
                         batchsize,
                         cache=True,
                         shuffle_buffer_size=1000,
                         train=True,
                         epochs=30):
    """
    Prepares a dataset for batched training.

    Args:
        dataset: The given dataset.
        batchsize: The desired batchsize
        cache: Use caching. If its a string uses a cache file.
        shuffle_buffer_size: Param for dataset shuffle, see tf2 documentation.
        train: Switch to skip data augmentation and shuffeling
        epochs: Number of repeats of the dataset

    Returns:
        dataset: A batched dataset, ready for training.
    """
    if cache:
        if isinstance(cache, str):
            dataset = dataset.cache(cache)
        else:
            dataset = dataset.cache()
    if train:
        dataset = dataset.map(data_augment, num_parallel_calls=AUTOTUNE)
        dataset = dataset.shuffle(buffer_size=shuffle_buffer_size,
                                  reshuffle_each_iteration=True)

    dataset = dataset.batch(batchsize)
    dataset = dataset.prefetch(buffer_size=AUTOTUNE)
    dataset = dataset.repeat(epochs+1)
    return dataset
