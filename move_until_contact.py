from rtde_control import RTDEControlInterface as RTDEControl

rtde_c = RTDEControl("192.168.5.5")
speed = [0, 0, -0.100, 0, 0, 0]
rtde_c.moveUntilContact(speed)

rtde_c.stopScript()
