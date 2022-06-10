import compas_rrc as rrc

if __name__ == '__main__':

    # Create Ros Client
    ros = rrc.RosClient()
    ros.run()

    # Create ABB Client
    abb = rrc.RrcClient(ros, '/rob1')
    print('Connected.')

    try:
        while True:
            abb.send(rrc.PrintText("doing something with robots"))
            # clamp control
    except ClampFailedException:
        # control & state & load

class GetJoints(NormalInstruction):
    pass

class SystemStop(SystemInstruction):
    pass

import compas_rrc as rrc
# import compas_rrc.sys as sys

    GetJoints()

    abb.send(rrc.SystemStop)
    abb.send(rrc.SystemStatus)
    abb.send_and_wait(SystemCustomInstruction)
    abb.send_and_subscribe(rrc.SystemJoints)



        # Unified API, but dedicated instructions for web service
        abb.send(rrc.DirectSystemStop())
        abb.send(rrc.DirectCustomInstruction(values=[1,2,7]))
        data = abb.send_and_wait(rrc.DirectGetMotorsState())
        abb.send_and_subscribe(rrc.DirectGetJoints())

        # Unified API, but dedicated instructions for external guided motion
        abb.send(rrc.egm.SystemStop())
        abb.send(rrc.egm.CustomInstruction(values=[1,2,7]))
        data = abb.send_and_wait(rrc.egm.GetMotorsState())
        abb.send_and_subscribe(rrc.egm.GetJoints())






        # Unified API, but dedicated instructions
        abb.send(rrc.control.SystemStop())
        abb.send(rrc.control.CustomInstruction(values=[1,2,7]))
        data = abb.send_and_wait(rrc.state.GetMotorsState())
        abb.send_and_subscribe(rrc.state.GetJoints())

        # (Alternative) Unified API, but dedicated instructions
        abb.send(rrc.SystemStop())
        abb.send(rrc.CustomInstruction(values=[1,2,7]))
        data = abb.send_and_wait(rrc.GetMotorsState())
        abb.send_and_subscribe(rrc.GetJoints())

        # Unified API, but dedicated instructions
        abb.control(rrc.SystemStop())
        abb.control(rrc.CustomInstruction(values=[1,2,7]))
        data = abb.state(rrc.GetMotorsState())
        abb.state(rrc.GetJoints())

        # Totally split API with dedicated methods
        abb.control.system_stop()
        abb.control.custom_instruction(dict(values=[1,2,7]))
        data = abb.state.get_motors_state()
        abb.state.subscribe_joints()

        # Control functions
        - System Stop
        - System Start (MotOnProgStart)
        - PP To Main
        - Set Digital Output/Input
        - Set RAPID Variables

        # State functions
        - Get Joints
        - Get Controller State
        - Get Operation Mode (Auto|Manual)
        - Get RAPID Variables
        - Get Digital Output/Input

    # while True:
    #     # abb.send(rrc.PrintText('Welcome to COMPAS_RRC'))

    #     robot_joints, external_axes = abb.send(rrc.GetJoints())
    #     robot_joints.rax_1 += 15
    #     done = abb.send(rrc.MoveToJoints(robot_joints, external_axes, 100, rrc.Zone.FINE))

    #     if user_pressed_cancel:
    #         # stop robot
    #         abb.send(rrc.SystemStop(), priority=HIGH)

    #     time.sleep(2)

    # Print feedback
    print('Feedback = ', done)

    # End of Code
    print('Finished')

    # Close client
    ros.close()
    ros.terminate()
