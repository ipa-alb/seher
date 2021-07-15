// Auto-generated. Do not edit!

// (in-package ros_openpose.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BodyPart = require('./BodyPart.js');

//-----------------------------------------------------------

class Person {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bodyParts = null;
      this.leftHandParts = null;
      this.rightHandParts = null;
    }
    else {
      if (initObj.hasOwnProperty('bodyParts')) {
        this.bodyParts = initObj.bodyParts
      }
      else {
        this.bodyParts = [];
      }
      if (initObj.hasOwnProperty('leftHandParts')) {
        this.leftHandParts = initObj.leftHandParts
      }
      else {
        this.leftHandParts = [];
      }
      if (initObj.hasOwnProperty('rightHandParts')) {
        this.rightHandParts = initObj.rightHandParts
      }
      else {
        this.rightHandParts = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Person
    // Serialize message field [bodyParts]
    // Serialize the length for message field [bodyParts]
    bufferOffset = _serializer.uint32(obj.bodyParts.length, buffer, bufferOffset);
    obj.bodyParts.forEach((val) => {
      bufferOffset = BodyPart.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [leftHandParts]
    // Serialize the length for message field [leftHandParts]
    bufferOffset = _serializer.uint32(obj.leftHandParts.length, buffer, bufferOffset);
    obj.leftHandParts.forEach((val) => {
      bufferOffset = BodyPart.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [rightHandParts]
    // Serialize the length for message field [rightHandParts]
    bufferOffset = _serializer.uint32(obj.rightHandParts.length, buffer, bufferOffset);
    obj.rightHandParts.forEach((val) => {
      bufferOffset = BodyPart.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Person
    let len;
    let data = new Person(null);
    // Deserialize message field [bodyParts]
    // Deserialize array length for message field [bodyParts]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.bodyParts = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.bodyParts[i] = BodyPart.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [leftHandParts]
    // Deserialize array length for message field [leftHandParts]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.leftHandParts = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.leftHandParts[i] = BodyPart.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [rightHandParts]
    // Deserialize array length for message field [rightHandParts]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.rightHandParts = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.rightHandParts[i] = BodyPart.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.bodyParts.length;
    length += 24 * object.leftHandParts.length;
    length += 24 * object.rightHandParts.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ros_openpose/Person';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5cbfeaba995a09efdb2c52e51374390b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # A person has some body parts. That is why we have created
    # an array of body parts.
    BodyPart[] bodyParts
    BodyPart[] leftHandParts
    BodyPart[] rightHandParts
    
    ================================================================================
    MSG: ros_openpose/BodyPart
    # The location and score of body parts are stored in a float array.
    # Therefore we are using 32 bits instead of 64 bits.
    # src: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/include/openpose/core/datum.hpp
    float32 score
    Pixel pixel
    geometry_msgs/Point32 point
    
    ================================================================================
    MSG: ros_openpose/Pixel
    # The location and score of body parts are stored in a float array.
    # Therefore we are using 32 bits instead of 64 bits.
    # src: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/include/openpose/core/datum.hpp
    # The location has been resized to the desired output
    # resolution (e.g., `resolution` flag in the demo).
    float32 x
    float32 y
    
    ================================================================================
    MSG: geometry_msgs/Point32
    # This contains the position of a point in free space(with 32 bits of precision).
    # It is recommeded to use Point wherever possible instead of Point32.  
    # 
    # This recommendation is to promote interoperability.  
    #
    # This message is designed to take up less space when sending
    # lots of points at once, as in the case of a PointCloud.  
    
    float32 x
    float32 y
    float32 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Person(null);
    if (msg.bodyParts !== undefined) {
      resolved.bodyParts = new Array(msg.bodyParts.length);
      for (let i = 0; i < resolved.bodyParts.length; ++i) {
        resolved.bodyParts[i] = BodyPart.Resolve(msg.bodyParts[i]);
      }
    }
    else {
      resolved.bodyParts = []
    }

    if (msg.leftHandParts !== undefined) {
      resolved.leftHandParts = new Array(msg.leftHandParts.length);
      for (let i = 0; i < resolved.leftHandParts.length; ++i) {
        resolved.leftHandParts[i] = BodyPart.Resolve(msg.leftHandParts[i]);
      }
    }
    else {
      resolved.leftHandParts = []
    }

    if (msg.rightHandParts !== undefined) {
      resolved.rightHandParts = new Array(msg.rightHandParts.length);
      for (let i = 0; i < resolved.rightHandParts.length; ++i) {
        resolved.rightHandParts[i] = BodyPart.Resolve(msg.rightHandParts[i]);
      }
    }
    else {
      resolved.rightHandParts = []
    }

    return resolved;
    }
};

module.exports = Person;
