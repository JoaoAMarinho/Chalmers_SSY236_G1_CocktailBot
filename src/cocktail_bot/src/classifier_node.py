from cocktail_bot.srv import ClassifyObject, ClassifyObjectResponse
import rospy
from joblib import load
import pandas as pd

PATH = "/home/user/exchange/src/cocktail_bot/model/"


class Classifier:
    def __init__(self):
        rospy.logwarn("Created Classifier Node")

        self.srv_classify_obj_name_ = "classify_object"
        classify_obj_srv_ = rospy.Service(
            self.srv_classify_obj_name_, ClassifyObject, self.srv_classify_callback
        )

        # Load classification model
        self.model = load(PATH + "classifier.joblib")
        self.shapes = [
            "shape_crystal",
            "shape_cube",
            "shape_curved",
            "shape_cylinder",
            "shape_granule",
            "shape_heart-shaped",
            "shape_irregular",
            "shape_liquid",
            "shape_oval",
            "shape_pyramid",
            "shape_rectangle",
            "shape_round",
            "shape_sphere",
        ]

    """
    Callback function for classify object service call
    """

    def srv_classify_callback(self, req):
        # Create object to classify
        obj = {
            "mass": req.mass,
            "width": req.width,
            "height": req.height,
            "red": req.red,
            "green": req.green,
            "blue": req.blue,
            "alcohol": req.alcohol,
        }

        currShape = "shape_" + req.shape
        for shape in self.shapes:
            if shape == currShape:
                obj[shape] = 1
            else:
                obj[shape] = 0

        # Predict class
        data = pd.DataFrame([obj], columns=list(obj.keys()))
        pred = self.model.predict(data)

        return ClassifyObjectResponse(pred[0], True)


if __name__ == "__main__":
    rospy.init_node("classifier_node")
    try:
        myClassifier = Classifier()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
