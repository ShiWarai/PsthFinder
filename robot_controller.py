import socket


class RobotController:
    def __init__(self, ip: str, port: int):
        self.ip = ip
        self.port = port
        self.__socket__ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_command(self, command: str):
        self.__socket__.sendto(command.encode(), (self.ip, self.port))
