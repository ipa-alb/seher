#!/usr/bin/env python
import datetime
import threading
import uuid
import rospy

from msb_client.ComplexDataFormat import ComplexDataFormat
from msb_client.DataType import DataType
from msb_client.Event import Event
from msb_client.Function import Function
from msb_client.MsbClient import MsbClient


def printMsg(msg):
    print(str(msg["dataObject"]))

if __name__ == "__main__":


    
    myMsbClient = MsbClient()

    myMsbClient.enableDebug(True)
    myMsbClient.disableHostnameVerification(True)
    myMsbClient.disableEventCache(False)

    event1 = Event("event1", "Event Name", "Event Description", DataType.FLOAT)
    myMsbClient.addEvent(event1)

    function1 = Function("function1", "Function Name", "Function Description", DataType.FLOAT, printMsg, False, None)
    myMsbClient.addFunction(function1)

    print(myMsbClient.objectToJson(myMsbClient.getSelfDescription()))

    myMsbClient.connect()

    myMsbClient.register()

    