#!/usr/bin/python3

import numpy as np
import rclpy
import tensorflow as tf
from rclpy.node import Node

from fruit_detection.srv import Classification


class DiseaseService(Node):
    def __init__(self):
        super().__init__('DiseaseService')
        #TODO add correct model file
        self.get_logger().info('Loading Model')
        self.model = tf.keras.models.load_model(
            'src/fruit_detection/networks/squeezenet.h5')
        self.diseases = ['healthy', 'rotten']
        self.srv = self.create_service(Classification, 'DiseaseService',
                                       self.predict)
        self.get_logger().info('Service Started')

    def predict(self, request, response):
        self.get_logger().info('Incoming request: %s' % (request.file))
        image = self.decode_img_path(request.file, 256, 256)

        prediction = self.model.predict([image], batch_size=1)
        response.disease = self.diseases[np.argmax(prediction)]
        return response

    def decode_img(self, img, width, height):
        # TODO test
        img = tf.image.decode_raw(img, tf.float32)
        img = img / 255.0
        img = tf.image.resize(img, [width, height])
        img = tf.expand_dims(img, 0)
        return img

    def decode_img_path(self, filename, width, height):
        img = tf.io.read_file(filename)
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
    tf.get_logger().setLevel('ERROR')
    main()
