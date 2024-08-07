import time
import math
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Session_pb2
from kortex_api.RouterClient import RouterClient
from kortex_api.TransportClientTcp import TransportClientTcp
from kortex_api.TransportClientUdp import TransportClientUdp

# Replace "utilities" with the actual path or logic you have for argument parsing and connection setup
import utilities

def move_to_home_position(base):
    """
    Moves the robot arm to the home position.
    """
    print("Moving the arm to a home position.")
    # Assuming 'Home' action is properly configured in the robot's action list
    action_request = Base_pb2.ReadActionRequest()
    action_request.name = "Home"
    action_response = base.ReadAction(action_request)

    if action_response.action.name == "Home":
        base.ExecuteActionFromReference(action_response.action.handle)
        print("Home action executed.")
        time.sleep(10)  # Wait for the action to complete
    else:
        print("Home action not found.")

def setup_low_level_servoing(base):
    """
    Sets up low-level servoing mode for the robot.
    """
    servoing_mode = Base_pb2.ServoingModeInformation()
    servoing_mode.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
    base.SetServoingMode(servoing_mode)
    print("Low-level servoing mode set.")

def example_actuator_low_level_velocity_control(base, base_cyclic, duration=5, velocity=20.0):
    """
    Demonstrates low-level velocity control of the robot's actuators.
    """
    move_to_home_position(base)
    setup_low_level_servoing(base)

    start_time = time.time()
    while time.time() - start_time < duration:
        feedback = base_cyclic.RefreshFeedback()
        command = BaseCyclic_pb2.Command()

        # Prepare command based on feedback to maintain velocity control
        for actuator_feedback in feedback.actuators:
            actuator_command = command.actuators.add()
            actuator_command.position = actuator_feedback.position
            actuator_command.velocity = velocity

        base_cyclic.Refresh(command)
        time.sleep(0.01)  # Sleep to throttle command rate

    # Restore original servoing mode
    restore_servoing_mode(base)

def restore_servoing_mode(base):
    """
    Restores the original servoing mode.
    """
    servoing_mode = Base_pb2.ServoingModeInformation()
    servoing_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(servoing_mode)
    print("Servoing mode restored.")

def main():
    args = utilities.parse_arguments()

    # Setup API connection
    with utilities.DeviceConnection.createTcpConnection(args) as router, \
         utilities.DeviceConnection.createUdpConnection(args) as router_real_time:

        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router_real_time)

        example_actuator_low_level_velocity_control(base, base_cyclic)

if __name__ == "__main__":
    main()
