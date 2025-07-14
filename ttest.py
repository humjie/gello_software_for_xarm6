from xarm.wrapper import XArmAPI
arm = XArmAPI('192.168.1.221')
print(arm.get_state())
arm.disconnect()
