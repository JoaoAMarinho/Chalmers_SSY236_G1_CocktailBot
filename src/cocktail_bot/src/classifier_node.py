from cocktail_bot.srv import ClassifyObject,ClassifyObjectResponse
import rospy
from joblib import load
import pandas as pd


class Classifier:
    def __init__(self):
        s = rospy.Service('classify_object', ClassifyObject, self.handle_classify)
        print("Classify node ready")
        path = './src/cocktail_bot/model/'
        self.model = load(path + 'classifier.joblib') 
        self.shapes = ['shape_crystal', 'shape_cube', 'shape_curved', 'shape_cylinder', 'shape_granule', 'shape_heart-shaped', 'shape_irregular', 'shape_liquid', 'shape_oval', 'shape_pyramid', 'shape_rectangle', 'shape_round','shape_sphere']


    def handle_classify(self,req):
        currShape = "shape_" + req.shape
        obj = {
            "mass": req.mass,
            "width": req.width,
            "height": req.height,
            "red": req.red,
            "green": req.green,
            "blue": req.blue,
            "alcohol": req.alcohol,
        }
        for shape in self.shapes:
            if shape == currShape:
                obj[shape] = 1
            else:
                obj[shape] = 0
        data = pd.DataFrame([obj], columns=list(obj.keys()))
        pred = self.model.predict(data)
        return ClassifyObjectResponse(pred[0])

if __name__ == "__main__":
    rospy.init_node('classifier_node')
    try:
        myClassifier = Classifier()
        rospy.spin()
    except rospy.ROSInterruptException: pass

