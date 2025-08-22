from dataclasses import dataclass
from pathlib import Path
import threading
import time

import tyro

from gello.robots.robot import BimanualRobot, PrintRobot
from gello.zmq_core.robot_node import ZMQServerRobot


@dataclass
class Args:
    robot: str = "xarm6"  # can be "xarm", "xarm6", "ur", etc.
    robot_port: int = 6001
    hostname: str = "127.0.0.1"
    robot_ip: str = "192.168.1.221"


def launch_robot_server(args: Args):
    port = args.robot_port
    if args.robot == "sim_ur":
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        from gello.robots.sim_robot import MujocoRobotServer

        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()
    elif args.robot == "sim_panda":
        from gello.robots.sim_robot import MujocoRobotServer

        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "franka_emika_panda" / "panda.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()
    elif args.robot == "sim_xarm":
        from gello.robots.sim_robot import MujocoRobotServer

        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "ufactory_xarm7" / "xarm7.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()
    elif args.robot == "sim_xarm6":
        from gello.robots.sim_robot import MujocoRobotServer

        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "ufactory_xarm6" / "xarm6.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()

    else:
        if args.robot == "xarm6":
            from gello.robots.xarm_robot import XArmRobot
            print(f"Initializing XArm6 robot with IP: {args.robot_ip}")
            robot = XArmRobot(ip=args.robot_ip, model="xarm6")
            
            # Start server in a separate thread so we can continue initialization
            print(f"Starting robot server on port {port}, host {args.hostname}")
            server = ZMQServerRobot(robot, port=port, host=args.hostname)
            server_thread = threading.Thread(target=server.serve)
            server_thread.daemon = True
            server_thread.start()
            
            # Wait for robot initialization to complete before setting up monitoring
            print(f"Waiting for robot initialization (press 's' then Enter in terminal to initialize)...")
            robot.wait_for_initialization()
            print("Robot initialization completed!")
            
            # Only start position monitoring after initialization
            print("Starting position monitoring with custom boundaries...")
            robot.start_position_monitoring(
                x_min=0, x_max=745,
                y_min=-400, y_max=610,
                z_min=-32, z_max=450
            )
            print(f"Position monitoring started with boundaries: {robot.get_position_boundaries()}")
            print("Robot server is fully operational and ready for connections.")
            
            # Keep main thread alive
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                print("Shutting down robot server...")
                robot.stop()
                print("Robot server stopped.")
        
        elif args.robot == "xarm":
            from gello.robots.xarm_robot import XArmRobot
            robot = XArmRobot(ip=args.robot_ip, model="xarm7")
        elif args.robot == "ur":
            from gello.robots.ur import URRobot
            robot = URRobot(robot_ip=args.robot_ip)
        elif args.robot == "panda":
            from gello.robots.panda import PandaRobot
            robot = PandaRobot(robot_ip=args.robot_ip)
        elif args.robot == "bimanual_ur":
            from gello.robots.ur import URRobot
            # IP for the bimanual robot setup is hardcoded
            _robot_l = URRobot(robot_ip="192.168.2.10")
            _robot_r = URRobot(robot_ip="192.168.1.10")
            robot = BimanualRobot(_robot_l, _robot_r)
        elif args.robot == "none" or args.robot == "print":
            robot = PrintRobot(7)  # 7 DOFs for xarm6 controller (6 arm + 1 gripper)
        else:
            raise NotImplementedError(
                f"Robot {args.robot} not implemented, choose one of: sim_ur, xarm, xarm6, ur, bimanual_ur, none"
            )
        server = ZMQServerRobot(robot, port=port, host=args.hostname)
        print(f"Starting robot server on port {port}")
        server.serve()


def main(args):
    launch_robot_server(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
