#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import tensorflow as tf
import numpy as np
import tensorflow_hub as hub

from fruit_detection.srv import Detection
from fruit_detection.msg import ClassBox

class DetectionService(Node):

    def __init__(self):
        super().__init__('DetectionService')
        self.get_logger().info('Loading Model')
        module_handle = "https://tfhub.dev/google/faster_rcnn/openimages_v4/inception_resnet_v2/1"
        self.model =  hub.load(module_handle).signatures['default']
        self.fruits =   fruit_list = [
            'Fruit', 'Apple', 'Grape', 'Common fig', 'Pear', 'Strawberry',
            'Tomato', 'Lemon', 'Banana', 'Orange', 'Peach', 'Mango', 'Pineapple',
            'Grapefruit', 'Pomegranate', 'Watermelon', 'Cantaloup'
        ]
        self.srv = self.create_service(Detection, 'DetectionService', self.predict)
        self.get_logger().info('Service Started')

    def predict(self, request, response):
        image = self.decode_img_path(request.file)
        converted_image = tf.image.convert_image_dtype(image, tf.float32)[tf.newaxis, ...]
        self.get_logger().info('Incoming request, starting detection')
        prediction = self.model(converted_image)
        self.get_logger().info('Detection complete')
        prediction = {key:value.numpy() for key,value in prediction.items()}
        response.classes = self.create_box_array(prediction)
        self.get_logger().info('Sending response')
        return response

    def decode_img_path(self, path):
        img = tf.io.read_file(path)
        img = tf.image.decode_jpeg(img, channels=3)
        return img

    def create_box_array(self, prediction):
        filtered_predictions = self.filter_prediction(prediction['detection_boxes'], prediction['detection_class_entities'], prediction['detection_scores'])
        classes = []
        for pred in filtered_predictions:
            box = ClassBox()
            box.ymin, box.xmin, box.ymax, box.xmax = tuple(pred[0].astype(type('float', (float,), {})))
            box.fruit_class = pred[1]
            box.score = int(100 * pred[2])
            classes.append(box)
        return classes

    def filter_prediction(self, boxes, classes, scores, max_boxes=10, min_score=0.3):
        predictions = []
        for i in range(min(boxes.shape[0], max_boxes)):
            predictions.append((boxes[i], classes[i].decode('utf-8'), scores[i]))
        predictions.sort(key = lambda x: x[2], reverse=True)
        predictions = [(b,c,s) for (b,c,s) in predictions if c in self.fruits and s > min_score]
        return predictions[:max_boxes]

def main(args=None):
    rclpy.init(args=args)
    detection_service = DetectionService()
    rclpy.spin(detection_service)
    rclpy.shutdown()


if __name__ == '__main__':
    tf.get_logger().setLevel('ERROR')
    main()
