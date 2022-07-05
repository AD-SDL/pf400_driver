import os.path
import time
import logging
import json

import pf400_client
from pf400_client import PF400

#Log Configuration
file_path = os.path.join(os.path.split(os.path.dirname(__file__))[0]  + '/pf400_logs/rpl_pf400_logs.log')

logging.basicConfig(filename = '/pf400_logs/rpl_pf400_logs.log', level=logging.DEBUG, format = '[%(levelname)s] [%(asctime)s] [%(name)s] %(message)s', datefmt = '%Y-%m-%d %H:%M:%S')

class RPL_PF400(PF400):
    """
    Python interface to socket interface of PF400. This is a subclass of the PF400 class, 
    which contains specific functions for Rapid Prototyping Lab setup.

    """
    def __init__(self, data_file_path = "robot_data.json"):
        super().__init__(data_file_path)
 
        self.OT2_ID = {"bob":1, "alex":2, "anna":3 , "peeler":4, "sealer":5}

    def command_handler(self, msg):

        """
        Decription: This function handles the commands that were sent through ZMQ messaging and ROS ARM node. 
                    It parses the message into job, robot ID_1 and robot ID_2. 
                    Depending on the type of the transfer job this funtion programs the PF400 robot to complete the transfers. 
        Parameters: 
                - msg: ZeroMQ message that coints job name and robot IDs in a single concatenated string.
        """
        msg = msg.split("@")
        self.force_initialize_robot()

        if len(msg) == 3 and msg[0].lower() == "transfer":
            if msg[1].lower() == "plate_rack":
                output_msg = self.program_rpl_robot(msg[1], msg[1], self.OT2_ID[msg[2]])
            elif msg[2].lower() == "completed":  
                print("completed")
                output_msg = self.program_rpl_robot(msg[2], self.OT2_ID[msg[1]], msg[2])
            else:
                output_msg = self.program_rpl_robot(msg[0],self.OT2_ID[msg[1]],self.OT2_ID[msg[2]])

        elif len(msg) == 2 and msg[0].lower() == "rack":
            output_msg = self.pick_plate_from_rack(self.OT2_ID[msg[1]])
        elif len(msg) == 1 and msg[0].lower() == "complete":
            output_msg = robot.drop_complete_plate()
        else:
            self.logger.error("User sent invalid command")
            return "Invalid command requested by the client!!!"    

        return output_msg
    
    def pick_plate_ot2(self, ot2_ID:int , profile:int = 0, wait:int = 0.1):

        """
        Decription: This function executes a series of motion commands to pick the plate from the OpenTrone2 robots
        Parameters: 
                - ot2_ID: ID number of the OpenTrone robot that the PF400 will be picking up the plate from
                - profile: Motion profile number. Use "3" for custom motion profile otherwise defult profiles will be used.
                - wait: Add a wait time between each motion command. Defult is 0.1 seconds.  
        """
        
        # TODO:ADD Motion profile index
        
        if profile == 3: 
            slow, fast = 3, 3
        else:
            slow, fast = 1, 2
            


        # Set movement commands to complete a pick_plate_ot2 operation
        move_front = self.set_move_command("ot2_" + str(ot2_ID) + "_front",fast)
        above_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_above_plate",fast)
        approach_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_pick_plate",fast)
        pick_up_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_pick_plate", slow, True, False)
        above_with_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_above_plate", slow, True, False)
        front_with_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_front", slow, True, False)

        pick_up_commands = [move_front, above_plate, approach_plate, pick_up_plate, above_with_plate, front_with_plate] 

        for count, cmd in enumerate(pick_up_commands):
                   
            input_msg = "[pick_plate_ot2 ID:{}] Robot is moved to the {}th location".format(str(ot2_ID), count+1)
            err_msg = 'Failed move the robot:'

            out_msg = self.send_command(cmd, input_msg, err_msg, wait)
              
        return out_msg


    def drop_plate_ot2(self, ot2_ID, profile = 0, wait:int = 0.1):
       
        """
        Decription: This function executes a series of motion commands to place the plate into the OpenTrone2 robots
        Parameters: 
                - ot2_ID: ID number of the OpenTrone robot that the PF400 will be placing the plate 
                - profile: Motion profile number. Use "3" for custom motion profile otherwise defult profiles will be used.
                - wait: Add a wait time between each motion command. Defult is 0.1 seconds.  
        """

        # Set movement commands to complete a drop_plate_ot2 operation
        if profile == 3: 
            slow, fast = 3, 3
        else:
            slow, fast = 1, 2
            

        front_with_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_front", slow, True, False)
        above_with_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_above_plate", slow, True, False)
        approach_with_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_pick_plate", slow, True, False)
        drop_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_pick_plate", slow, False, True)
        above_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_above_plate", fast)
        front_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_front", fast)


        drop_to_ot2 = [front_with_plate, above_with_plate, approach_with_plate, drop_plate, above_plate, front_plate]
        
        for count, cmd in enumerate(drop_to_ot2):

            input_msg = "[drop_plate_ot2 ID:{}] Robot is moved to the {}th location".format(str(ot2_ID), count+1)
            err_msg = 'Failed move the robot:'

            out_msg = self.send_command(cmd, input_msg, err_msg, wait)
              
        return out_msg
                
    def pick_plate_from_rack(self, ot2_ID, profile = 0, wait:int = 0.1):

        """
        Decription: This function executes a series of motion commands to pick the plate from the plate rack. 
        Parameters: 
                - ot2_ID: ID number of the OpenTrone robot is used to specify the plate rack number. PF400 will pick up the plate from the plate rack that is on top of the same OT2, specified in this paramiter. 
                - profile: Motion profile number. Use "3" for custom motion profile otherwise defult profiles will be used.
                - wait: Add a wait time between each motion command. Defult is 0.1 seconds.  
        """
        
        if profile == 3: 
            slow, fast = 3, 3
        else:
            slow, fast = 1, 2

        approach_plate_rack = self.set_move_command("ot2_" + str(ot2_ID) + "_approach_plate_rack", fast, False, False)
        front_rack = self.set_move_command("ot2_" + str(ot2_ID) + "_front_plate_rack", slow, False, False)
        plate_rack = self.set_move_command("ot2_" + str(ot2_ID) + "_plate_rack", slow, False, False)
        pick_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_plate_rack", slow, True, False)
        front_with_plate = self.set_move_command("ot2_" + str(ot2_ID) + "_front_plate_rack", slow, True, False)
        approach_rack_back = self.set_move_command("ot2_" + str(ot2_ID) + "_approach_plate_rack", slow, True, False)

        pick_plate_rack = [approach_plate_rack, front_rack, plate_rack, pick_plate, front_with_plate, approach_rack_back]
       
        for count, cmd in enumerate(pick_plate_rack):
                   
            input_msg = "[pick_plate_from_rack ID:{}] Robot is moved to the {}th location".format(str(ot2_ID), count+1)
            err_msg = 'Failed move the robot:'

            out_msg = self.send_command(cmd, input_msg, err_msg, wait)
              
        return out_msg
        

    def drop_complete_plate(self, profile = 0, wait:int = 0.1):

        """
        Decription: This function executes a series of motion commands to place the 96 well plate to the completed plate location. Program assumes that the PF400 is already carrying the plate with it's gripper. 
        Parameters: 
                - profile: Motion profile number. Use "3" for custom motion profile otherwise defult profiles will be used.
                - wait: Add a wait time between each motion command. Defult is 0.1 seconds.  
        """
        
        if profile == 3: 
            slow, fast = 3, 3
        else:
            slow, fast = 1, 2

        completed_plate_above = self.set_move_command("completed_plate_above", slow, True, False)
        drop_with_plate = self.set_move_command("completed_plate", slow, True, False)
        drop_plate = self.set_move_command("completed_plate", slow, False, True)
        completed_plate = self.set_move_command("completed_plate_above", slow, False, False)
       

        complete_plate = [completed_plate_above, drop_with_plate, drop_plate, completed_plate]
       
        for count, cmd in enumerate(complete_plate):
                   
            input_msg = "[drop_complete_plate] Robot is moved to the {}th location".format(count+1)
            err_msg = 'Failed move the robot:'

            out_msg = self.send_command(cmd, input_msg, err_msg, wait)
              
        return out_msg

    def rpl_teach_location(self, location:str = None):

        """
        Decription: A function to edit the robot locoations data and save the new coordinates in joint angles format. 
                    When this function is called, robot joints will be released and then it can be moved manually to the desired locations. 
                    Program will ask the user if they want to save the new joint angles to the given location. Once the new angles are saved, robot will be initilized again and remote command interface will be available.
        Parameters: 
                - location: Location name that the new joint angles will be saved. This name should already be exist in the robot location data.
        """

        output = self.disable_power()
        if output[0]  == "-":
            raise Exception("Falied disabling power. Aborting teach location!")

        self.logger.info("Please set the robot location by manually moving the arm to the disered location.\n" \
            "!!! ------------------------------------ IMPORTANT ------------------------------------ !!! \n" \
            "To release the vertical rail, HOLD the arm to make sure it does not fall down and " \
            "then press the black release button underneath the second joint.")

        user_save_input = str(input("Do you want to save the current location? (y/n): "))
        
        if user_save_input.lower() == "y":
            output = self.rpl_save_location(location)
            self.logger.info(output)
        elif user_save_input.lower() == "n":
            self.logger.warning("Location is not saved!")
        else:
            self.logger.warning("Please enter 'y' to save or 'n' to not save the new location")
            return self.rpl_teach_location(location)
        
        self.force_initialize_robot()

    def rpl_save_location(self, location:str = None):

        """
        Decription: A sub function of the rpl_teach_location funtion to save the new coordinates. This function reads and writes into the robot_data.json file.   
        Parameters: 
                - location: Location name that the new joint angles will be saved. This name should already be exist in the robot location data.
        """

        if location == None:
            self.logger.error("[rpl_save_location] Location name was not provided by the user")
            raise Exception("Please enter a location name to save the new joint values")

        location = location.lower()
        # TODO: Check if location exist in the robot data
        if self.check_loc_data(location) == False:
            return "Invalid location entered"
        
        # # Setting parent file directory 
        # current_directory = os.path.dirname(__file__)
        # parent_directory = os.path.split(current_directory)[0] 
        # file_path = os.path.join(parent_directory + '/utils/robot_data.json')
        
        current_location = self.locate_robot()

        # Save the current robot location to the given location
        loc_list = list(map(float,current_location.split(" ")))
        self.location_dictionary[location] = [loc_list[1], loc_list[2], loc_list[3], loc_list[4], loc_list[5], loc_list[6]]
    
        # Write the new location the data file
        try:
            with open(file_path, "w") as jsonFile:
                json.dump(self.robot_data, jsonFile, indent=4)
        except IOError as err:
            self.logger.error(err)

        return "Current location is saved to {}".format(location)
    
    def program_rpl_robot(self, job:str, robot_ID_1: int = 0, robot_ID_2:int = 0):
        
        """
        Decription: A function to program the PF400 to execute transfer movements between two location. This funtion will call multiple sub funtions to complete full transfer operations, depending on the starting and ending points.
        Parameters: 
                - job: This parameter defines, what kind of transfer job will be executed.
                    .Transfer: Plate transfer between two OT2 robots.
                    .PLATE_RACK: Plate transfer between the plate rack and OT2 robot.
                    .CompletedL Plate transfer between OT2 robot and completed plate location.
                - robot_ID_1: First robot ID, which is goig to be the starting point of the transfer.
                - robot_ID_2: Second robot ID, which is goig to be the ending point of the transfer.

        """
        # TODO: Find robot location in the environment and plan movements depending on the surrounding obstacles

        self.move_single("homeall", 2)

        if job.upper() == "TRANSFER":

            self.logger.info("Executing plate transfer between OT2 ID: {} and OT2 ID: {}".format(robot_ID_1, robot_ID_2))
            self.pick_plate_ot2(robot_ID_1)
            time.sleep(2)
            self.drop_plate_ot2(robot_ID_2)
            
        elif job.upper() == "PLATE_RACK":
            self.logger.info("Executing plate transfer between plate_rack and OT2 ID: {}".format(robot_ID_2))
            self.pick_plate_from_rack(1)
            time.sleep(2)
            self.drop_plate_ot2(robot_ID_2)

        elif job.upper() == "COMPLETED":
            self.logger.info("Executing plate transfer OT2 ID: {} and completed plate location".format(robot_ID_1))
            self.pick_plate_ot2(robot_ID_1)
            self.drop_complete_plate()

        elif job.upper() == "FULL_TRANSFER":

            self.logger.info("Executing full transfer")
            self.pick_plate_from_rack(1)
            self.drop_plate_ot2(1)
            time.sleep(50)
            self.pick_plate_ot2(1)
            self.drop_plate_ot2(2)
            time.sleep(50)
            self.pick_plate_ot2(2)
            self.drop_plate_ot2(3)
            time.sleep(50)
            self.pick_plate_ot2(3)
            self.drop_complete_plate()
   
if __name__ == "__main__":
    robot = RPL_PF400()
    # robot.command_handler("rack@bob")
    robot.rpl_teach_location("transfer_d")
