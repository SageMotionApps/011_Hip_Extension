import time
import numpy as np
from sage.base_app import BaseApp
from .Rotation import Rotation as R
from .JointAngles import JointAngles

FEEDBACK_OFF = 0
FEEDBACK_ON = 1

def get_rotation(data, node_num):
    return R.from_quat(
        [
            data[node_num]["Quat1"],
            data[node_num]["Quat2"],
            data[node_num]["Quat3"],
            data[node_num]["Quat4"],
        ],
        scalar_first=True,
    )


class Core(BaseApp):
    ###########################################################
    # INITIALIZE APP
    ###########################################################
    def __init__(self, my_sage):
        BaseApp.__init__(self, my_sage, __file__)

        self.DATARATE = self.info["datarate"]
        self.pulse_length = self.info["pulse_length"]
        self.min_threshold = float(self.config["min_threshold"])
        self.max_threshold = float(self.config["max_threshold"])

        self.NodeNum_thigh = self.info["sensors"].index('thigh')
        self.NodeNum_pelvis = self.info["sensors"].index('pelvis')
        self.NodeNum_feedback_min = self.info["feedback"].index('feedback_min')
        self.NodeNum_feedback_max = self.info["feedback"].index('feedback_max')

        self.iteration = 0
        self.joint_angles = JointAngles(self.config["which_leg"] == "Right Leg")        
        self.Hip_flex = 0.0

        self.thigh_quat = None

        self.alreadyGivenFeedback = 0        
        self.min_feedback_state = FEEDBACK_OFF
        self.max_feedback_state = FEEDBACK_OFF

    ###########################################################
    # CHECK NODE CONNECTIONS
    ###########################################################
    def check_status(self):
        sensors_count = self.get_sensors_count()
        feedback_count = self.get_feedback_count()
        err_msg = ""
        if sensors_count < len(self.info["sensors"]):
            err_msg += "App requires {} sensors but only {} are connected".format(
                len(self.info["sensors"]), sensors_count
            )
        if self.config["feedback_enabled"] and feedback_count < len(
            self.info["feedback"]
        ):
            err_msg += "App require {} feedback but only {} are connected".format(
                len(self.info["feedback"]), feedback_count
            )
        if err_msg != "":
            return False, err_msg
        return True, "Now running Hip Extension App"
    #############################################################
    # UPON STARTING THE APP
    # If you have anything that needs to happen before the app starts
    # collecting data, you can uncomment the following lines
    # and add the code in there. This function will be called before the
    # run_in_loop() function below.
    #############################################################
    # def on_start_event(self, start_time):
    #     print("In On Start Event: {start_time}")

    ###########################################################
    # RUN APP IN LOOP
    ###########################################################
    def run_in_loop(self):
        data = self.my_sage.get_next_data()

        # Get Quaternion Data
        self.pelvis_quat = get_rotation(data, self.NodeNum_pelvis)
        self.thigh_quat = get_rotation(data, self.NodeNum_thigh)        

        # Calibrate to find BS_q, sensor to body segment alignment quaternions on 1st iteration
        if self.iteration == 0:
            self.joint_angles.calibrate(self.pelvis_quat, self.thigh_quat)

        # Calculate Extension angles
        self.Hip_flex = self.joint_angles.calculate_Hip_Flex(
            self.pelvis_quat, self.thigh_quat
        )

        # Give haptic feedback (turn feedback nodes on/off)
        if self.config["feedback_enabled"]:
            self.give_feedback()
        else:
            self.min_feedback_state = FEEDBACK_OFF
            self.max_feedback_state = FEEDBACK_OFF

        time_now = self.iteration / self.DATARATE  # time in seconds

        my_data = {
            'time': [time_now],
            "min_threshold": [self.min_threshold],
            "max_threshold": [self.max_threshold],
            "min_feedback_state": [self.min_feedback_state],
            "max_feedback_state": [self.max_feedback_state],
            "Hip_ext": [self.Hip_flex],
        }

        self.my_sage.save_data(data, my_data)
        self.my_sage.send_stream_data(data, my_data)

        self.iteration += 1
        return True
    #############################################################
    # MANAGE FEEDBACK FOR APP
    #############################################################
    def toggle_feedback(self, feedbackNode=0, duration=1, feedback_state=FEEDBACK_OFF):
        if feedback_state:
            self.my_sage.feedback_on(feedbackNode, duration)
        else:
            self.my_sage.feedback_off(feedbackNode)

    def give_feedback(self):        
        self.min_feedback_state = int(self.Hip_flex < self.min_threshold)
        self.max_feedback_state = int(self.Hip_flex > self.max_threshold)

        self.toggle_feedback(
            self.NodeNum_feedback_min,
            duration=self.pulse_length,
            feedback_state=self.min_feedback_state,
        )
        self.toggle_feedback(
            self.NodeNum_feedback_max,
            duration=self.pulse_length,
            feedback_state=self.max_feedback_state,
        )

        # if no feedback should be given, make sure to toggle all feedback off
        # Note: This can only happen if both are 0
        if self.min_feedback_state == self.max_feedback_state:
            self.toggle_all_feedback_off()

    def toggle_all_feedback_off(self):
        self.toggle_feedback(self.NodeNum_feedback_min, feedback_state=FEEDBACK_OFF)
        self.toggle_feedback(self.NodeNum_feedback_max, feedback_state=FEEDBACK_OFF)

    #############################################################
    # UPON STOPPING THE APP
    # If you have anything that needs to happen after the app stops,
    # you can uncomment the following lines and add the code in there.
    # This function will be called after the data file is saved and
    # can be read back in for reporting purposes if needed.
    #############################################################
    # def on_stop_event(self, stop_time):
    #     print(f"In On Stop Event: {stop_time}")
