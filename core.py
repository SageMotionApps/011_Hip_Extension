from sage.base_app import BaseApp
import math


#############################################################
# Is Prime
# This is a sample function to tell if a number is a prime number.     
#############################################################
def is_prime(second):
    if second < 2:
        return False
    for i in range(2, int(second**0.5) + 1):
        if second % i == 0:
            return False
    return True

class Core(BaseApp):
    def __init__(self, my_sage):
        BaseApp.__init__(self, my_sage, __file__)
        self.iteration = 0

        self.sensor_node = self.info["sensors"].index("sensor")
        self.feedback_node = self.info["feedback"].index("feedback")
        self.time_limit = self.config.get('stop_time')

    ###########################################################
    # CHECK NODE CONNECTIONS
    # Make sure all the nodes needed for sensing and feedback
    # are present before starting the app. 
    # 
    # If you do not need to check for feedback nodes, you can 
    # comment or delete this function. The BaseApp will ensure
    # the correct number of sensing nodes are present and 
    # throw an exception if they are not. 
    ###########################################################
    # def check_status(self):
    #     sensors_count = self.get_sensors_count()
    #     feedback_count = self.get_feedback_count()
    #     logging.debug("config pulse length {}".format(self.info["pulse_length"]))
    #     err_msg = ""
    #     if sensors_count < len(self.info['sensors']):
    #         err_msg += "App requires {} sensors but only {} are connected".format(
    #                 len(self.info['sensors']), sensors_count)
    #     if self.config['feedback_enabled'] and feedback_count < len(self.info['feedback']):
    #         err_msg += "App require {} feedback but only {} are connected".format(
    #                 len(self.info['feedback']), feedback_count)
    #     if err_msg != "":
    #         return False, err_msg
    #     return True, "Now running Angle Template App"

    
    #############################################################
    # UPON STARTING THE APP
    # If you have anything that needs to happen before the app starts 
    # collecting data, you can uncomment the following lines
    # and add the code in there. This function will be called before the
    # run_in_loop() function below. 
    #############################################################
    # def on_start_event(self, start_time):
    #     print("In On Start Event: {start_time}")

    #############################################################
    # RUN APP IN LOOP
    #############################################################
    def run_in_loop(self):
        #GET RAW SENSOR DATA
        data = self.my_sage.get_next_data()        

        #Create Annotations to be passed to the chart
        time_now = self.iteration / self.info["datarate"]  # time in seconds
        annotations = ""
        if math.isclose(5, time_now, rel_tol= 0.1):
            annotations = {"x": str(5), "y": '.9', "xref": 'x', "yref": 'paper', "text": 'Look here!'}

        # Create custom user defined string based on the time being less than 10 seconds
        user_defined_status = ""
        if time_now < 10:
            user_defined_status = "Please Start Moving."


        # Create Audio Feedback string
        audio_feedback = ""
        if self.config["audio_prompts_enabled"]:
            audio_feedback = "Around the World"

        # Example Feedback Logic and Call
        if self.config["feedback_enabled"]:
            if is_prime(Int(time_now)) and self.get_sensors_count() > 0:
                # Example of how to send vibration to a FEEDBACK node (this is usually done)
                self.toggle_sensor_vibration(sensorNodeNum=self.sensor_node, duration = 1, pulse_strength = 4, feedback_state = False)
                self.toggle_feedback_vibration(sensorNodeNum=self.feedback_node, duration = 1, pulse_strength = 4, feedback_state = True)
                user_defined_status = "Lucky You! (This Second is a prime number) (Vibrating Sensor Node!)"
            elif is_prime(Int(time_now)) and Int(time_now) % 5 == 0:
                # Example of how to send vibration to a FEEDBACK & SENSOR node (this is usually done)
                self.toggle_sensor_vibration(sensorNodeNum=self.sensor_node, duration = 1, pulse_strength = 4, feedback_state = True)
                self.toggle_feedback_vibration(sensorNodeNum=self.feedback_node, duration = 1, pulse_strength = 4, feedback_state = True)
                user_defined_status = "Oh No! Prime number and divisible by 5! (Vibrating ALL Nodes!)"
            else:                
                # Example of how to turn off all vibration
                self.toggle_sensor_vibration(sensorNodeNum=self.sensor_node, duration = 1, pulse_strength = 4, feedback_state = False)
                self.toggle_feedback_vibration(sensorNodeNum=self.feedback_node, duration = 1, pulse_strength = 4, feedback_state = False)
                user_defined_status = "Phew! Safe Number (No Vibration)"
        
        else:
            #Turn off Vibration
            self.toggle_sensor_vibration(sensorNodeNum=self.feedback_node, duration = 1, pulse_strength = 4, feedback_state = False)

        
        #CREATE CUSTOM DATA PACKET
        my_data = {
            "time": [time_now],                
                'example_int': [1],
                # This is an exmaple of how you can grab the value from the app configuration
                # and store pass it to be stored in the data file (as a user field).
                'example_bool': [self.config["audio_prompts_enabled"]],   
                'user_defined_status': [user_defined_status],
                'feedback_enabled': [self.config["feedback_enabled"]],
                'audio_prompts_enabled': [self.config["audio_prompts_enabled"]],
                "annotations": [annotations],
                'audio_feedback': [audio_feedback]
                }


        # SAVE DATA
        self.my_sage.save_data(data, my_data)

        # STREAM DATA
        self.my_sage.send_stream_data(data, my_data)

        # Increment iteration count
        self.iteration += 1

        # If you want to stop the app on some condition, you can add login in to return a False value. 
        # This will stop the app and move to the post processing state.
        if self.time_limit != None:
            if self.time_limit > time_now: 
                return True
            else:
                return False

        else: 
            return True

    

    

    #############################################################
    # Toggle Feedback
    # This function is used to toggle the feedback on and off for 
    # any given node. You can also change the feedback intensity
    # by passing in a value to the pulse_strength field. 
    #############################################################
    def toggle_sensor_vibration(self, sensorNodeNum=0, duration = 1, pulse_strength = 4, feedback_state = False):
        if feedback_state:
            self.my_sage.sensor_vibration_on(sensorNodeNum, duration, strength=pulse_strength)
        else:
            self.my_sage.sensor_vibration_off(sensorNodeNum)

    def toggle_feedback_vibration(self, feedbackNode=0, duration = 1, pulse_strength = 4, feedback_state = False):
        if feedback_state:
            self.my_sage.feedback_vibration_on(feedbackNode, duration, strength=pulse_strength)
            
        else:
            self.my_sage.feedback_vibration_off(feedbackNode)

    #############################################################
    # UPON STOPPING THE APP
    # If you have anything that needs to happen after the app stops, 
    # you can uncomment the following lines and add the code in there.
    # This function will be called after the data file is saved and 
    # can be read back in for reporting purposes if needed.
    #############################################################
    def on_stop_event(self, stop_time):

        # This will show in the Notification Box that overlays the app settings.
        self.logger.info("App Finished Running! You can use this to display summary information, or give custom status update during post-processing.")
