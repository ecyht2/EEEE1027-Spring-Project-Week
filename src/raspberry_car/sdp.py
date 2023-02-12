#!/usr/bin/env python3
"""A simple script that registers a bluez profile that enables SPP.

This script is based on spp.py script on
https://scribles.net/setting-up-bluetooth-serial-port-profile-on-raspberry-pi-using-d-bus-api/
"""
# https://stackoverflow.com/questions/53069598/bluez-adding-services-attributes-and-profiles-without-sdptool-command
# https://stackoverflow.com/questions/41290114/bluetooth-sdp-where-is-sdpd
# https://scribles.net/setting-up-bluetooth-serial-port-profile-on-raspberry-pi-using-d-bus-api/
import signal
import sys
import time
from multiprocessing import Process

import dbus


def advertise_service(channel: int = 1):
    """Sets-up a bluez profile to advertise an RFCOMM service at a given
    channel.

    The function will raise a ValueError if the RFCOMM is not within 1-30.
    It also raises a ValueError if channel is not an integer.

    :param channel: The channel of the RFCOMM service.
    """
    if not isinstance(channel, int):
        raise ValueError("channels needs to be an integer.")
    if channel < 1 or channel > 30:
        raise ValueError("RFCOMM channel must be between 1 and 30.")

    print(f"Advertising RFCOMM Service in channel {channel}")

    # https://github.com/AnesBenmerzoug/Bluetooth_HID/blob/master/sdp_record.xml
    # https://btprodspecificationrefs.blob.core.windows.net/assigned-numbers/Assigned%20Number%20Types/Assigned%20Numbers.pdf
    # http://www.bluecove.org/bluecove/apidocs/javax/bluetooth/ServiceRecord.html
    service_record = f"""
    <?xml version="1.0" encoding="UTF-8" ?>
    <record>
    <attribute id="0x0001">
        <sequence>
        <uuid value="0x1101"/>
        </sequence>
    </attribute>
    <attribute id="0x0004">
        <sequence>
        <sequence>
            <uuid value="0x0003"/>
            <uint8 value="{channel}" name="channel"/>
        </sequence>
        </sequence>
    </attribute>
    </record>
    """

    bus = dbus.SystemBus()
    manager = dbus.Interface(bus.get_object("org.bluez", "/org/bluez"),
                             "org.bluez.ProfileManager1")
    manager.RegisterProfile("/bluez",
                            "00001101-0000-1000-8000-00805f9b34fb",
                            {
                                "AutoConnect": True,
                                "ServiceRecord": service_record
                            })
    while True:
        time.sleep(1)


if __name__ == "__main__":
    process = Process(target=advertise_service)
    signal.signal(signal.SIGTERM, lambda a, b: sys.exit(0))
    signal.signal(signal.SIGINT, lambda a, b: sys.exit(0))

    try:
        process.start()
        process.join()
    finally:
        print("Stopped Advertising Service")
        process.terminate()
