from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import cv2

class TLClassifier(object):
    def load_graph(self,model_file):
        graph = tf.Graph()
        graph_def = tf.GraphDef()

        with open(model_file, "rb") as f:
            graph_def.ParseFromString(f.read())
        with graph.as_default():
            tf.import_graph_def(graph_def)

        return graph
    
    def load_labels(self,label_file):
        label = []
        proto_as_ascii_lines = tf.gfile.GFile(label_file).readlines()
        for l in proto_as_ascii_lines:
            label.append(l.rstrip())
        return label
    
    def __init__(self):
        self.real = False

        self.labels_dic = {
            'none':4,
            'green':2,
            'yellow':1,
            'red':0}

        if self.real:
            model_path = 'light_classification/models/site_graph.pb'
            label_path = 'light_classification/models/site_labels.txt'
        else:
            model_path = 'light_classification/models/sim_graph.pb'
            label_path = 'light_classification/models/sim_labels.txt'
            
        self.labels = self.load_labels(label_path)
        self.graph = self.load_graph(model_path)
        
        input_layer = "input"
        output_layer = "final_result"

        input_name = "import/" + input_layer
        output_name = "import/" + output_layer
        
        self.input_operation = self.graph.get_operation_by_name(input_name)
        self.output_operation = self.graph.get_operation_by_name(output_name)
        self.sess = tf.Session(graph=self.graph)
        print("Classifier initialized")


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        img= cv2.resize(image,dsize=(224,224), interpolation = cv2.INTER_CUBIC)
        img=cv2.normalize(img.astype('float'), None, -0.5, .5, cv2.NORM_MINMAX)

        #Convert to tensor
        np_image_data = np.asarray(img)
        reshaped_img = np.expand_dims(np_image_data,axis=0)

        results = self.sess.run(self.output_operation.outputs[0],
                      {self.input_operation.outputs[0]: reshaped_img})
        results = np.squeeze(results)
        
        
        top_k = results.argsort()[-5:][::-1]
        predict_label = self.labels[top_k[0]]
        print(predict_label)

        return self.labels_dic[predict_label]
