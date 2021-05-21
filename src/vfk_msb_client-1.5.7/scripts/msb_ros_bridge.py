#!/usr/bin/env python2
import rospy
from rospy_message_converter import message_converter, json_message_converter
from sys import version
from time import sleep

from vfk_msb_ros.msb_ros_helper import MsbRosHelper
from vfk_msb_py.msb_classes import *
from vfk_msb_py.msb_communicate import MsbCommunicate
from vfk_msb_py.msb_ws4py_client import MsbWsClient


class MsbRosBridge:

    def rosCallback(self, data, args):
        """method to be registered as callback to any ROS event"""
        rospy.logdebug(data)
        try:
            if self.wsConnected:
                [s, e] = args
                if self.json_only:
                    data = {'data': json_message_converter.convert_ros_message_to_json(data).replace('"', "'") }
                    data = message_converter.convert_dictionary_to_ros_message(
                        "std_msgs/String",
                        data
                    )
 
                self.mwc.emit_event(service=s, event=e, data=data, prio=2)
        except Exception, ex:
            rospy.logerr('unable to emit event: ' + str(ex))
            self.wsConnected = False

    def checkTopicsAndImport(
            self,
            topics,
            confTopicsPub,
            confTopicsSub,
            changed
    ):
        """check if configured topics are published and import their classes"""
        (
            namesTopicsPub,
            namesTopicsSub
        ) = self.getTopicNamesFromConf(
            confTopicsPub,
            confTopicsSub
        )

        rospy.logdebug('checking topics for ..')
        rospy.logdebug(str(namesTopicsPub + namesTopicsSub))

        topicsPubd = self.mrh.getPubdTopics()
        rospy.logdebug("topicsPubd: " + str(topicsPubd))
        for topic in (namesTopicsPub):
            if topic not in topics:
                for t in confTopicsPub:
                    if t[0] is topic:
                        changed = True
                        topics[topic] = {}
                        topics[topic]['type'] = t[1].split("/")
        for topic in (namesTopicsSub):
            if topic not in topics:
                for t in confTopicsSub:
                    if(len(t) == 2 and (t[0] is topic)):
                        changed = True
                        topics[topic] = {}
                        topics[topic]['type'] = t[1].split("/")
            if topic not in topics:
                if topic in topicsPubd:
                    changed = True
                    topics[topic] = {}
                    topics[topic]['type'] = self.mrh.runCommand(
                        ["rostopic", "type", topic]).replace(
                        "\n", "").split(
                        "/"
                    )
        if changed:
            for topic in topics:
                try:
                    globalcom = (# make import global first
                        "global " + 
                        topics[topic]['type'][1]
                    )
                    rospy.loginfo('exec\'ing: >' + globalcom + '<')
                    exec (globalcom)
                    importcom = (# import type
                        "from " + 
                        topics[topic]['type'][0] + 
                        ".msg import " + 
                        topics[topic]['type'][1]
                    )
                    rospy.loginfo('exec\'ing: >' + importcom + '<')
                    exec (importcom, globals())
                except Exception, e:
                    rospy.logerr(
                        '>' + importcom + "< not succesful:" + e.message)
                    raise ImportError(
                        '>' + 
                        importcom + 
                        "< not succesful:" + 
                        e.message
                    )
                # Saving a ROS Publisher ===============================
                try:
                    if topic in namesTopicsPub:
                        for t in confTopicsPub:
                            if t[0] is topic:
                                msgClass = eval(topics[topic]['type'][1])
                                topicRos = topic.replace("-", "/")
                                topics[topic]['rosPublisher'] = (
                                    rospy.Publisher(
                                        topicRos,
                                        msgClass,
                                        queue_size=10
                                    )
                                )
                except Exception, e:
                    rospy.logerr(e.message)
                    raise e

                # Creating message dict ================================
                try:
                    if self.json_only:
                        topics[topic]['dict'] = self.mrh.getDictForMsgType(
                            "std_msgs/String"
                        )
                    else:
                        topics[topic]['dict'] = self.mrh.getDictForMsgType(
                          "/".join(topics[topic]['type'])
                        )   
                except IOError, e:
                    rospy.logerr(str(e))
                    raise e
        return topics, changed, topicsPubd

    def getTopicNamesFromConf(self, confTopicsPub, confTopicsSub):
        """return names for configured topics"""
        namesTopicsPub = [entry[0] for entry in confTopicsPub]
        namesTopicsSub = [entry[0] for entry in confTopicsSub]
        return (namesTopicsPub, namesTopicsSub)

    def getEvents(self, topics, confTopicsSub, pubdTopics):
        """return event definitions for configured topics"""
        es = {}
        for topic in confTopicsSub:
            if not (topic in topics):
                continue
            if topic in pubdTopics or "type" in topics[topic]:
                topicName = topic.replace("/", "-")
                es[topic] = (Event(
                    eventId=topicName,
                    name=topicName + " ROS topic",
                    description="ROS event of type " + 
                                "/".join(topics[topic]['type']) + 
                                " (in ROS)",
                    dataFormat=topics[topic]['dict']
                ))
        return es

    def getFunctions(self, topics, confTopicsPub):
        """return function definitions for configured topics"""
        fs = []
        for topic in confTopicsPub:
            topicName = topic.replace("/", "-")
            fs.append(Function(
                functionId="/topic/" + topicName,
                name=topicName + " ROS topic",
                description="/".join(topics[topic]['type']) + 
                            " (in ROS)",
                dataFormat=topics[topic]['dict'],
                callback=False
            ))
        return fs

    def subscribeTo(self, service, topics, subbed, topics_tosub, events):
        """subscribe to certain ros topics"""
        rospy.loginfo("topics.keys()")
        rospy.loginfo(topics.keys())
        rospy.loginfo("subbed")
        rospy.loginfo(subbed)
        rospy.loginfo("topics_tosub")
        rospy.loginfo(topics_tosub)
        
        for topic in topics.keys():
            if (
                (topic not in subbed) & 
                (topic in topics_tosub)
            ):
                topicName = topic.replace("-", "/")
                rospy.loginfo("subscribing to " + topicName)
                rospy.Subscriber(
                    topicName,
                    eval(topics[topic]['type'][1]),
                    callback=self.rosCallback,
                    callback_args=[service, events[topic]]
                )
                subbed.append(topic)
        return subbed

    def checkForStar(self, confTopics):
        """check if a star is in the condifgured topics
        (which means to sub all)"""
        if confTopics == ["*"]:
            pubdTopics = self.mrh.getPubdTopics()
            return pubdTopics
        else:
            return confTopics

    def getParamTopics(self):
        """get conifgured topics"""
        try:
            confTopicsSub = self.mrh.getTopics(
                rospy.get_param('~topics_sub', '')
            )
            confTopicsSub = self.checkForStar(confTopicsSub)

            confTopicsPub = self.mrh.getTopics(
                rospy.get_param('~topics_pub', '')
            )

            return (confTopicsPub, confTopicsSub)
        except Exception, e:
            rospy.logerr(
                'no topics defined to publish and / or subscribe ' + 
                str(e))
            raise RuntimeError('no topics defined to subscribe ' + str(e))

    def wsCallback(self, m):
        """callback to messages recieved over websocket"""
        m = str(m)
        if (m[0:5] == 'a["C '):  # CALLBACK
            try:
                msgDic = self.mc.handleCallback(m)
                topic = msgDic["functionId"].split('/')[-1]
                topic = topic.replace("-", "/")
                if self.json_only:
                    msgString = message_converter.convert_dictionary_to_ros_message(
		            "std_msgs/String",
		            msgDic["functionParameters"]["dataObject"])
                    msg = json_message_converter.convert_json_to_ros_message(
		            "/".join(self.topics[topic]['type']),
		            msgString.data.replace("'",'"'))
                else:
                    msg = message_converter.convert_dictionary_to_ros_message(
		            "/".join(self.topics[topic]['type']),
		            msgDic["functionParameters"]["dataObject"])

                self.topics[topic]['rosPublisher'].publish(msg)
                rospy.loginfo(
                    'published topic: ' + 
                    topic + 
                    ' .. ' + 
                    str(msg).replace('\n', '/')
                )
            except Exception, e:
                rospy.logerr("Can not parse this message:" + m + 
                             str(e.__class__) + str(e))
                raise e
        elif (
                            m == 'o' or
                            m == 'h' or
                    m.startswith('a["IO')
        ):  # OK
            rospy.logdebug("OK")
        else:  # N OK
            rospy.logwarn("Not OK")
            rospy.logwarn(m)
            self.wsConnected = False

    def __init__(self):
        """starting the bridge ...."""
        # ROS STUFF
        rospy.init_node('msb_ros_bridge', anonymous=True)
        rospy.loginfo("initialized ...")

        # HELPERS
        self.mrh = MsbRosHelper()
        self.mc = MsbCommunicate()

        # GETTING PARAMS
        try:
            host = rospy.get_param('~host')
        except Exception:
            host = 'ws://demo.virtualfortknox.de/msb2'  # DEFAULT
            rospy.logwarn(
                'unable to get param ~host (using default: ' + 
                host + 
                ')'
            )
        try:
            token = rospy.get_param('~token')
        except Exception:
            token = 'rossecret'  # DEFAULT
            rospy.logwarn(
                'unable to get param ~token (using default: \'' + 
                token + 
                '\')'
            )
        try:
            name = rospy.get_param('~name')
        except Exception:
            name = 'ROS Bridge'  # DEFAULT
            rospy.logwarn(
                'unable to get param ~name (using default: \'' + 
                name + 
                '\')'
            )
        try:
            uuid = rospy.get_param('~uuid')
        except Exception:
            uuid = 'rosbridge'  # DEFAULT
            rospy.logwarn(
                'unable to get param ~uuid (using default: \'' + 
                uuid + 
                '\')'
            )
        try:
            desc = rospy.get_param('~desc')
        except Exception:
            desc = 'A ROS Bridge'  # DEFAULT
            rospy.logwarn(
                'unable to get param ~desc (using default: \'' + 
                desc + 
                '\')'
            )
        
        try:
            self.json_only = rospy.get_param('~json_only')
        except Exception:
            self.json_only = False  # DEFAULT
            rospy.logwarn(
                'unable to get param ~json_only (using default: \'' +
                str(self.json_only) +
                '\')'
            )        

        (self.confTopicsPub, self.confTopicsSub) = self.getParamTopics()

        # CONNECTING TO WS
        self.mwc = MsbWsClient(host, self.wsCallback)
        self.wsConnected = True

        self.topics = {}
        self.subbedTopics = []

        self.changed = True

        while not rospy.is_shutdown():
            try:
                if self.wsConnected:
                    # GET PARAM CONFIGS
                    (
                        self.confTopicsPub,
                        self.confTopicsSub
                    ) = self.getParamTopics()

                    (
                        namesTopicsPub,
                        namesTopicsSub
                    ) = self.getTopicNamesFromConf(
                        self.confTopicsPub,
                        self.confTopicsSub
                    )
                    # TOPIC DEFINITIONS
                    (
                        self.topics,
                        self.changed,
                        self.pubdTopics
                    ) = self.checkTopicsAndImport(
                        self.topics,
                        self.confTopicsPub,
                        self.confTopicsSub,
                        self.changed
                    )
                    if self.changed:
                        # GET EVENTS
                        es = self.getEvents(
                            self.topics,
                            namesTopicsSub,
                            self.pubdTopics
                        )
                        # GET FUNCTIONS
                        fs = self.getFunctions(
                            self.topics,
                            namesTopicsPub
                        )
                        # REGISTER SMO WITH MSB
                        self.s = SmartObject(
                            uuid,  # a robot
                            name,
                            desc,
                            es.values(),
                            fs,
                            token
                        )
                        self.mwc.register(self.s)
                        rospy.loginfo(
                            'REGISTER: ' + 
                            str(self.s.toJson())
                        )

                        sleep(.1)  # waiting for a little bit

                        # REGISTER LISTENER TO ROS
                        self.subbedTopics = self.subscribeTo(
                            self.s,
                            self.topics,
                            self.subbedTopics,
                            namesTopicsSub,
                            es
                        )

                    self.changed = False

                else:  # self.wsConnected
                    rospy.loginfo("currently not connected, retrying ...")
                    self.mwc = MsbWsClient(host, self.wsCallback)
                    self.wsConnected = True

                # MAKE ROS SPIN
                sleep(1)
                # while not rospy.is_shutdown()
            except Exception, ex:
                rospy.logerr("Exception:", Exception, " ..")
                self.mwc.disconnect()


if __name__ == '__main__':
    """where all the magic happens"""
    assert int(version.split('.')[0]) == 2, \
        ("Please use Python 2, using " + version)
    mrb = MsbRosBridge()
