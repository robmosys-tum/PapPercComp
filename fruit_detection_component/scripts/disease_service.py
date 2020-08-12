#!/usr/bin/python3
"""
ROS2 Service providing disease classification in single images.
Returns a list of ClassBox containing boundingboxes and fruit class as well as
diseases.
"""
import os

import cv2
import numpy as np
import rclpy
import tensorflow as tf
from cv_bridge import CvBridge
from rclpy.node import Node
from fruit_detection.srv import Classification

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'


class DiseaseService(Node):
    """
    Specification of disease classification service node.
    Args:
        None
    Params:
        model_file (str): Pretrained TensorFlow model.
    """
    def __init__(self):
        """
        Initialises the service by creating the cvBridge, loading the model from
        the given parameter and loading the list of valid disease classes.
        """
        super().__init__('DiseaseService')
        self.declare_parameter('model_file')
        self.declare_parameter('diseases')

        model_file = self.get_parameter(
            'model_file').get_parameter_value().string_value
        self.get_logger().info('Loading Model %s' % model_file)
        self.model = tf.keras.models.load_model(model_file)
        self.bridge = CvBridge()
        self.width = 256
        self.height = 256
        self.diseases = self.get_parameter(
            'diseases').get_parameter_value().string_array_value
        self.srv = self.create_service(Classification, 'DiseaseService',
                                       self.predict)
        self.get_logger().info('Service Started')

    def predict(self, request, response):
        """
        Callback for handeling incomeing requests. Basically updates disease
        and disease_score fileds in the provided ClassBoxes
        Args:
            request: The incomeing service request defined in Classification.srv.
            response: The empty response for answering the request.
        Returns:
            response: The updated response defined in Classification.srv.
        """
        self.get_logger().info('Incoming request, starting classification')
        image = self.decode_request(request)
        predictions = self.model.predict(image)
        self.get_logger().info('Classification complete')
        new_boxes = request.boxes
        for index, box in enumerate(new_boxes):
            box.disease = self.diseases[np.argmax(predictions[index])]
            box.disease_score = int(100 * np.max(predictions[index]))
        response.new_boxes = new_boxes
        self.get_logger().info('Sending response %s' % new_boxes)
        return response

    def decode_request(self, request):
        """
        Decodes an reqeust containing sensor_msgs/Image and ClassBoxes for use
        in Tensorflow. The image gets converted and preprocessed. The image
        then gets split up into multiple crops representing one classbox each.
        Args:
            request: The given request.
        Returns:
            img: The image as tf.tensor containing mutliple crops.
        """
        cv_image = self.bridge.imgmsg_to_cv2(request.img, "bgr8")
        image_data = cv2.imencode('.jpg', cv_image)[1].tostring()
        img = tf.image.decode_jpeg(image_data, channels=3)
        img = tf.cast(img, tf.float32) / 255.0
        img = tf.image.convert_image_dtype(img, tf.float32)
        img = tf.expand_dims(img, 0)
        bounding_boxes = []
        box_indices = np.zeros(len(request.boxes))
        for box in request.boxes:
            bounding_boxes.append([box.ymin, box.xmin, box.ymax, box.xmax])
        bounding_boxes = tf.convert_to_tensor(bounding_boxes, tf.float32)
        box_indices = tf.convert_to_tensor(box_indices, tf.int32)
        crop = tf.convert_to_tensor([self.width, self.height], tf.int32)
        img = tf.image.crop_and_resize(img,
                                       bounding_boxes,
                                       box_indices,
                                       crop,
                                       extrapolation_value=0.)
        return img


def main(args=None):
    """
    Main method to start the service.
    """
    rclpy.init(args=args)
    disease_service = DiseaseService()
    rclpy.spin(disease_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
