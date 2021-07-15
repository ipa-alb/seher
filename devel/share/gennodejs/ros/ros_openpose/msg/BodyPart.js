// Auto-generated. Do not edit!

// (in-package ros_openpose.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Pixel = require('./Pixel.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class BodyPart {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.score = null;
      this.pixel = null;
      this.point = null;
    }
    else {
      if (initObj.hasOwnProperty('score')) {
        this.score = initObj.score
      }
      else {
        this.score = 0.0;
      }
      if (initObj.hasOwnProperty('pixel')) {
        this.pixel = initObj.pixel
      }
      else {
        this.pixel = new Pixel();
      }
      if (initObj.hasOwnProperty('point')) {
        this.point = initObj.point
      }
      else {
        this.point = new geometry_msgs.msg.Point32();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BodyPart
    // Serialize message field [score]
    bufferOffset = _serializer.float32(obj.score, buffer, bufferOffset);
    // Serialize message field [pixel]
    bufferOffset = Pixel.serialize(obj.pixel, buffer, bufferOffset);
    // Serialize message field [point]
    bufferOffset = geometry_msgs.msg.Point32.serialize(obj.point, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BodyPart
    let len;
    let data = new BodyPart(null);
    // Deserialize message field [score]
    data.score = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pixel]
    data.pixel = Pixel.deserialize(buffer, bufferOffset);
    // Deserialize message field [point]
    data.point = geometry_msgs.msg.Point32.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ros_openpose/BodyPart';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '243edb92e04d1e621047e9d565b8e64f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new BodyPart(null);
    if (msg.score !== undefined) {
      resolved.score = msg.score;
    }
    else {
      resolved.score = 0.0
    }

    if (msg.pixel !== undefined) {
      resolved.pixel = Pixel.Resolve(msg.pixel)
    }
    else {
      resolved.pixel = new Pixel()
    }

    if (msg.point !== undefined) {
      resolved.point = geometry_msgs.msg.Point32.Resolve(msg.point)
    }
    else {
      resolved.point = new geometry_msgs.msg.Point32()
    }

    return resolved;
    }
};

module.exports = BodyPart;
