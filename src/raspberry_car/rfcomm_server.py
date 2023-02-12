#!/usr/bin/env python3
"""A simple Python script to receive messages from a client over
Bluetooth using Python sockets (with Python 3.3 or above).
"""
# https://community.appinventor.mit.edu/t/raspberry-pi-bluetooth-send-receive/59846
# https://people.csail.mit.edu/albert/bluez-intro/
import signal
import socket
import sys
import time
import argparse
from multiprocessing import Process, queues
from types import NoneType
from typing import NoReturn

import dbus

import sdp


def get_default_bluetooth() -> str:
    """Gets the default bluetooth device of the OS.

    This function relies on bluez to get the default bluetooth device. Please
    ensure bluez is installed in the system. It also uses dbus to interface
    with bluez. (dbus should be installed if bluez is instaleld)

    :return: The default bluetooth device address.
    """
    # Fetching bus
    bus = dbus.SystemBus()
    bluez_bus = bus.get_object('org.bluez', '/org/bluez/hci0')

    # Fetching Address property
    props_iface = dbus.Interface(bluez_bus, 'org.freedesktop.DBus.Properties')
    address = props_iface.Get('org.bluez.Adapter1', 'Address')

    return address


def recieve_data(bluetooth_address: str | NoneType = "", channel: int = 1,
                 output: queues.Queue | NoneType = None) -> NoReturn:
    """Create a scoket to receive data from a bluetooth device from channel 1.

    The RFCOMM protocol is used for the communication.

    The function will raise a ValueError if the RFCOMM is not within 1-30.
    It also raises a ValueError if channel is not an integer.

    :param bluetooth_address: The address of the bluetooth device to be used.
    If an empty string or None is provided, the default bluetooth device of the
    OS is used. (bluez is required for device discovery)
    :param output: The output queue to transfer data to. If None is provided,
    the data would be printed to standard output.
    """
    # Setting up bluetooth address
    if bluetooth_address is None or bluetooth_address == "":
        HOST = get_default_bluetooth()
    else:
        HOST = bluetooth_address

    # Setting up RFCOMM channel
    if not isinstance(channel, int):
        raise ValueError("channels needs to be an integer.")
    if channel < 1 or channel > 30:
        raise ValueError("RFCOMM channel must be between 1 and 30.")
    PORT = channel

    print(f"Recieving Data from channel {channel}")

    with socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                       socket.BTPROTO_RFCOMM) as s:
        # Connecting to client
        s.bind((HOST, PORT))
        s.listen(1)
        client, addr = s.accept()
        print(f"Connected with {addr}")

        while True:
            # Recieving and decoding data
            data = client.recv(1)
            data = data.decode()

            # Outputting data
            if output is None:
                print(f"Recived {data} from client")
            else:
                output.put(data)


if __name__ == '__main__':
    def parse_arguments():
        parser = argparse.ArgumentParser(description="""\
        A simple Python script to receive messages from a client over
        Bluetooth using Python sockets (with Python 3.3 or above).""")

        parser.add_argument('-c', '--channel', default=1, type=int,
                            help="RFCOMM channel to use.")
        parser.add_argument('-b', '--baddr', default="",
                            help="Bluetooth device address to use.")

        return parser.parse_args()

    args = parse_arguments()

    thread_advertise = Process(target=sdp.advertise_service,
                               args=(args.channel,))
    thread_socket = Process(target=recieve_data,
                            args=(args.baddr, args.channel))

    signal.signal(signal.SIGTERM, lambda a, b: sys.exit(0))
    signal.signal(signal.SIGINT, lambda a, b: sys.exit(0))

    try:
        thread_advertise.start()
        thread_socket.start()
        while thread_advertise.is_alive() and thread_socket.is_alive():
            time.sleep(1)
    finally:
        print("Shutting Down")
        thread_advertise.terminate()
        thread_socket.terminate()
