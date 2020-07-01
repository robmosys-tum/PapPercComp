#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import tensorflow as tf
import numpy as np

from fruit_detection.srv import Classification

class DiseaseService(Node):

    def __init__(self):
        super().__init__('DiseaseService')
        self.srv = self.create_service(Classification, 'DiseaseService', self.predict)
        self.model = tf.keras.models.load_model('models/squeezenet.h5')
        self.diseases = ['healthy', 'rotten']

    def predict(self, request, response):
        image = decode_img_path(request.file, 256, 256)
        self.get_logger().info('Incoming request')
        prediction = self.model.predict([image], batch_size=1)
        resopnse.disease = self.disease[np.argmax(prediction)]
        return response

    def decode_img(img, width, height):
        img = tf.image.decode_raw(img, tf.float32)
        img = img / 255.0
        img = tf.image.resize(img, [width, height])
        img = tf.expand_dims(img, 0)
        return img

    def decode_img_path(img, width, height):
        img = tf.image.decode_image(img, channels=3, expand_animations=False)
        img = tf.cast(img, tf.float32) / 255.0
        img = tf.image.convert_image_dtype(img, tf.float32)
        img = tf.image.resize(img, [width, height])
        img = tf.expand_dims(img, 0)
        return img


def main(args=None):
    rclpy.init(args=args)
    disease_service = DiseaseService()
    rclpy.spin(disease_service)
    rclpy.shutdown()



if __name__ == '__main__':
    main()
