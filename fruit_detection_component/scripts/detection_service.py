#!/usr/bin/python3
"""
ROS2 Service providing fruit detection in single images.
Returns a list of ClassBox containing boundingboxes and fruit class.
"""
import os

import cv2
import rclpy
import tensorflow as tf
import tensorflow_hub as hub
from cv_bridge import CvBridge
from rclpy.node import Node

from fruit_detection.msg import ClassBox
from fruit_detection.srv import Detection

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'


class DetectionService(Node):
    """
    Specification of object (fruit) detection service node.
    Args:
        None
    Params:
        module_handle (str): tensorflow hub model string.
    """
    def __init__(self):
        """
        Initialises the service by creating the cvBridge, loading the model from
        the given parameter and declaring the list of valid fruit classes.
        """
        super().__init__('DetectionService')
        self.declare_parameter('module_handle')
        module_handle = self.get_parameter(
            'module_handle').get_parameter_value().string_value
        self.get_logger().info('Loading Modle %s' % module_handle)
        self.model = hub.load(module_handle).signatures['default']
        self.bridge = CvBridge()
        self.fruits = [
            'Fruit', 'Apple', 'Grape', 'Common fig', 'Pear', 'Strawberry',
            'Tomato', 'Lemon', 'Banana', 'Orange', 'Peach', 'Mango',
            'Pineapple', 'Grapefruit', 'Pomegranate', 'Watermelon', 'Cantaloup'
        ]
        self.srv = self.create_service(Detection, 'DetectionService',
                                       self.predict)
        self.get_logger().info('Service Started')

    def predict(self, request, response):
        """
        Callback for handeling incomeing requests.
        Args:
            request: The incomeing service request defined in Detection.srv.
            response: The empty response for answering the request.
        Returns:
            response: The updated response defined in Detection.srv.
        """
        image = self.decode_img(request.img)
        converted_image = tf.image.convert_image_dtype(image,
                                                       tf.float32)[tf.newaxis,
                                                                   ...]
        self.get_logger().info('Incoming request, starting detection')
        prediction = self.model(converted_image)
        self.get_logger().info('Detection complete')
        prediction = {key: value.numpy() for key, value in prediction.items()}
        response.classes = self.create_boxes(prediction)
        self.get_logger().info('Sending response %s' % str(response.classes))
        return response

    def decode_img(self, img):
        """
        Decodes an sensor_msgs/Image using the cvBridge for use in tensorflow.
        Args:
            img: The binary input image.
        Returns:
            img: The image as tf.tensor
        """
        cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        image_data = cv2.imencode('.jpg', cv_image)[1].tostring()
        img = tf.image.decode_jpeg(image_data, channels=3)
        return img

    def create_boxes(self, prediction):
        """
        Creates the ClassBox objects according to the prediction.
        Args:
            prediction (dict): Prediction from detection model containing boxes
                scores and class names.
        Returns:
            class_boxes (list): List of valid ClassBoxes.
        """
        filtered_predictions = self.filter_prediction(prediction)
        class_boxes = []
        for pred in filtered_predictions:
            box = ClassBox()
            box.ymin, box.xmin, box.ymax, box.xmax = tuple(pred[0].astype(
                type('float', (float, ), {})))
            box.fruit = pred[1]
            box.fruit_score = int(100 * pred[2])
            class_boxes.append(box)
        return class_boxes

    def filter_prediction(self, prediction, max_boxes=10, min_score=0.5):
        """
        Filters given prediction by class, score and length.
        Args:
            prediction (dict): Prediction from detection model containing boxes
                scores and class names.
        Returns:
            predictions (list): List of filtered predicitons.
        """
        boxes = prediction['detection_boxes']
        classes = prediction['detection_class_entities']
        scores = prediction['detection_scores']
        predictions = []
        for i in range(boxes.shape[0]):
            predictions.append(
                (boxes[i], classes[i].decode('utf-8'), scores[i]))
        predictions.sort(key=lambda x: x[2], reverse=True)
        predictions = [(b, c, s) for (b, c, s) in predictions
                       if c in self.fruits and s > min_score]
        return predictions[:max_boxes]


def main(args=None):
    """
    Main method to start the service.
    """
    rclpy.init(args=args)
    detection_service = DetectionService()
    rclpy.spin(detection_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
