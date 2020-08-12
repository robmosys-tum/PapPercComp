"""Custom trained Squeezenez Model for disease classification."""
import math

import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf

from data_util import (create_testset, create_train_val_set, decode_img,
                       prepare_for_training)


def plot_history(history):
    """
    Plots a models train/validation accuracy and loss.

    Args:
        history: Output of fit() function call.
    """
    plt.subplots(figsize=(10, 10), facecolor='#F0F0F0')
    plt.tight_layout()
    sub = plt.subplot(211)
    sub.set_facecolor('#F8F8F8')
    sub.plot(history.history['accuracy'])
    sub.plot(history.history['val_accuracy'])
    sub.set_title('model accuracy')
    sub.set_ylabel('accuracy')
    sub.set_xlabel('epoch')
    sub.legend(['train', 'valid.'])
    sub = plt.subplot(212)
    sub.set_facecolor('#F8F8F8')
    sub.plot(history.history['loss'])
    sub.plot(history.history['val_loss'])
    sub.set_title('model loss')
    sub.set_ylabel('loss')
    sub.set_xlabel('epoch')
    sub.legend(['train', 'valid.'])
    plt.show()


DISEASES = ['blotch', 'canker', 'healthy', 'mold', 'rot', 'scab']


class SqueezeNet():
    """
    Small SqueezeNet model.

    Args:
        width: Input image width.
        height: Input image height.
        momentum: Momentum for batchnorm.
        for_train: Switch to init model.
        classes: Desired classes.
    """
    def __init__(self, width, height, momentum=0.9, for_train=True, classes=2):
        """
        Defines the model.
        """
        self.momentum = momentum
        self.width = width
        self.height = height
        if for_train:
            x = tf.keras.layers.Input(shape=[width, height, 3])

            y = tf.keras.layers.Conv2D(kernel_size=3,
                                       filters=32,
                                       padding='same',
                                       use_bias=True,
                                       activation='relu')(x)
            y = tf.keras.layers.BatchNormalization(momentum=self.momentum)(y)
            y = self.squeeze(24, 48)(y)
            y = tf.keras.layers.MaxPooling2D(pool_size=2)(y)
            y = self.squeeze(48, 96)(y)
            y = tf.keras.layers.MaxPooling2D(pool_size=2)(y)
            y = self.squeeze(64, 128)(y)
            y = tf.keras.layers.MaxPooling2D(pool_size=2)(y)
            y = self.squeeze(96, 192)(y)
            y = tf.keras.layers.MaxPooling2D(pool_size=2)(y)
            y = self.squeeze(64, 128)(y)
            y = tf.keras.layers.MaxPooling2D(pool_size=2)(y)
            y = self.squeeze(48, 96)(y)
            y = tf.keras.layers.MaxPooling2D(pool_size=2)(y)
            y = self.squeeze(24, 48)(y)
            y = tf.keras.layers.GlobalAveragePooling2D()(y)
            y = tf.keras.layers.Dense(classes, activation='softmax')(y)

            self.model = tf.keras.Model(x, y)
            self.model.compile(optimizer='adam',
                               loss='categorical_crossentropy',
                               metrics=['accuracy'])

            self.model.summary()

    def squeeze_layer(self, x, squeeze_, expand):
        """
        Squeeze layer described in the paper.

        Args:
            x_in: The input to the layer
            squeeze_: The squeeze filters.
            expand: The expand filters.

        Returns:
            layer: The according keras layer output.
        """
        y = tf.keras.layers.Conv2D(filters=squeeze_,
                                   kernel_size=1,
                                   activation='relu',
                                   padding='same')(x)
        y = tf.keras.layers.BatchNormalization(momentum=self.momentum)(y)
        y1 = tf.keras.layers.Conv2D(filters=expand // 2,
                                    kernel_size=1,
                                    activation='relu',
                                    padding='same')(y)
        y1 = tf.keras.layers.BatchNormalization(momentum=self.momentum)(y1)
        y3 = tf.keras.layers.Conv2D(filters=expand // 2,
                                    kernel_size=3,
                                    activation='relu',
                                    padding='same')(y)
        y3 = tf.keras.layers.BatchNormalization(momentum=self.momentum)(y3)
        return tf.keras.layers.concatenate([y1, y3])

    def squeeze(self, squeeze_, expand):
        """
        Wrapper for keras to use squeeze_layer.
        """
        return lambda x: self.squeeze_layer(x, squeeze_, expand)

    def train(self, data_dir, batchsize=32, epochs=30, plot=True):
        """
        Trains the squeezenet model.

        Args:
            data_dir: Directory of training data.
            batchsize: The batchsize in training.
            epochs: Number of epochs.
            plot: Switch to display accuracy and loss of training.
        """
        (train, t_len), (val, v_len) = create_train_val_set(data_dir,
                                                            width=self.width,
                                                            height=self.height,
                                                            split=0.1)
        steps_per_epoch = int(math.ceil(1. * t_len / batchsize))
        validation_steps = int(math.ceil(1. * v_len / batchsize))
        train = prepare_for_training(train,
                                     batchsize,
                                     train=True,
                                     epochs=epochs)
        val = prepare_for_training(val, batchsize, train=False)
        history = self.model.fit(train,
                                 epochs=epochs,
                                 validation_data=val,
                                 steps_per_epoch=steps_per_epoch)
        if plot:
            plot_history(history)

    def save(self, path):
        """
        Saves the trained model to a file.

        Args:
            path: The filepath.
        """
        self.model.save(path)

    def load(self, path):
        """
        Loads the trained model from a file.

        Args:
            path: The filepath.
        """
        self.model = tf.keras.models.load_model(path)

    def predict(self, filename):
        """
        Predicts the class to the model.

        Args:
            img: The desired image.

        Returns:
            pred: The prediced class
        """
        assert self.model, "Model not initialized"
        img = tf.io.read_file(filename)
        img = decode_img(img, self.width, self.height)
        img = tf.expand_dims(img, 0)
        pred = self.model.predict([img], batch_size=1)
        return DISEASES[np.argmax(pred)]


if __name__ == "__main__":

    MODEL = SqueezeNet(256, 256, classes=len(DISEASES), momentum=0.95)
    MODEL.train('D:/data/fruit/', epochs=50)  # was trained on windows
    MODEL.save('../../networks/squeezenet_v1.h5')
