#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import tensorflow as tf
import numpy as np
import tensorflow_hub as hub

from fruit_detection.srv import Detection

class DetectionService(Node):

    def __init__(self):
        super().__init__('DetectionService')
        self.srv = self.create_service(Detection, 'DetectionService', self.predict)
        module_handle = "https://tfhub.dev/google/faster_rcnn/openimages_v4/inception_resnet_v2/1" 
        self.model =  hub.load(module_handle).signatures['default']
        self.fruits =   fruit_list = [
            'Fruit', 'Apple', 'Grape', 'Common fig', 'Pear', 'Strawberry',
            'Tomato', 'Lemon', 'Banana', 'Orange', 'Peach', 'Mango', 'Pineapple',
            'Grapefruit', 'Pomegranate', 'Watermelon', 'Cantaloup'
        ]

    def predict(self, request, response):
        image = decode_img_path(request.file)
        self.get_logger().info('Incoming request')
        prediction = self.model(image)
        prediction = {key:value.numpy() for key,value in prediction.items()}
        response.classes = create_box_array(prediction)
        return response

    def load_img(path):
        img = tf.io.read_file(path)
        img = tf.image.decode_jpeg(img, channels=3)
        return tf.image.convert_image_dtype(img, tf.float32)[tf.newaxis, ...]

    def create_box_array(prediction):
        #TODO filter 
        return []

def main(args=None):
    rclpy.init(args=args)
    detection_service = DetectionService()
    rclpy.spin(detection_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
