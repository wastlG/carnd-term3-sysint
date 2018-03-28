from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import cv2

class TLClassifier(object):
    def __init__(self,real):
        self.real = real

        self.labels_dic = {
            'off':4,
            'green':2,
            'yellow':1,
            'red':0}

        if real:
            model_path = 'light_classification/models/frcnn_inference_graph.pb'
        else:
            model_path = 'light_classification/models/sim_inference_graph.pb'
            self.labels = [line.rstrip() for line in tf.gfile.GFile('light_classification/models/sim_output_labels.txt')]
        self.classification_graph = tf.Graph()
        with self.classification_graph.as_default():
            graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as f:
                serialized_graph = f.read()
                graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(graph_def,name='')

            if real:
                self.image_tensor = self.classification_graph.get_tensor_by_name('image_tensor:0')
                self.d_boxes = self.classification_graph.get_tensor_by_name('detection_boxes:0')
                self.d_scores = self.classification_graph.get_tensor_by_name('detection_scores:0')
                self.d_classes = self.classification_graph.get_tensor_by_name('detection_classes:0')
                self.num_d = self.classification_graph.get_tensor_by_name('num_detections:0')
            else:
                self.image_tensor   = self.classification_graph.get_tensor_by_name('input:0')
                self.softmax_tensor = self.classification_graph.get_tensor_by_name('final_result:0')
        self.sess = tf.Session(graph=self.classification_graph)
        print("Classifier initialized")


    def scale_image(self, img):
        image_data = cv2.resize(img, (224,224))
        image_data = (image_data - 128.)/128.
        image_data = np.reshape(image_data, (1, 224,224,3))
        return image_data


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.real:
            with self.classification_graph.as_default():
                img_expanded = np.expand_dims(image, axis=0)  
                (boxes, scores, classes, num) = self.sess.run([self.d_boxes, self.d_scores, self.d_classes, self.num_d],feed_dict={self.image_tensor: img_expanded})

                #TODO

        else:
            with self.classification_graph.as_default():
                image_data = self.scale_image(image)
                predictions = self.sess.run(self.softmax_tensor, {self.image_tensor: image_data})

                # Sort to show labels in order of confidence
                top_k = predictions.argsort()
                for node_id in top_k:
                    for i in node_id:
                        predict_label = self.labels[i]
                        score = predictions[0][i]
                        #print('%s (score = %.5f)' % (predict_label, score))

        return self.labels_dic[predict_label]




    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        if self.real:
            with self.classification_graph.as_default():
                img_expanded = np.expand_dims(image, axis=0)  
                (boxes, scores, classes, num) = self.sess.run([self.d_boxes, self.d_scores, self.d_classes, self.num_d],feed_dict={self.image_tensor: img_expanded})

                #TODO

        else:
            with self.classification_graph.as_default():
                image_data = self.scale_image(image)
                predictions = self.sess.run(self.softmax_tensor, {self.image_tensor: image_data})

                # Sort to show labels in order of confidence
                top_k = predictions.argsort()
                for node_id in top_k:
                    for i in node_id:
                        predict_label = self.labels[i]
                        score = predictions[0][i]
                        print('%s (score = %.5f)' % (predict_label, score))

        return self.labels_dic[predict_label]
