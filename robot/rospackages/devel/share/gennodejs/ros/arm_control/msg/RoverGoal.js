// Auto-generated. Do not edit!

// (in-package arm_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class RoverGoal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.desiredDir = null;
      this.distToGoal = null;
    }
    else {
      if (initObj.hasOwnProperty('desiredDir')) {
        this.desiredDir = initObj.desiredDir
      }
      else {
        this.desiredDir = 0.0;
      }
      if (initObj.hasOwnProperty('distToGoal')) {
        this.distToGoal = initObj.distToGoal
      }
      else {
        this.distToGoal = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RoverGoal
    // Serialize message field [desiredDir]
    bufferOffset = _serializer.float64(obj.desiredDir, buffer, bufferOffset);
    // Serialize message field [distToGoal]
    bufferOffset = _serializer.float64(obj.distToGoal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RoverGoal
    let len;
    let data = new RoverGoal(null);
    // Deserialize message field [desiredDir]
    data.desiredDir = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [distToGoal]
    data.distToGoal = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'arm_control/RoverGoal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4aa12af78877a5824d18dc450fa46f2d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 desiredDir
    float64 distToGoal
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RoverGoal(null);
    if (msg.desiredDir !== undefined) {
      resolved.desiredDir = msg.desiredDir;
    }
    else {
      resolved.desiredDir = 0.0
    }

    if (msg.distToGoal !== undefined) {
      resolved.distToGoal = msg.distToGoal;
    }
    else {
      resolved.distToGoal = 0.0
    }

    return resolved;
    }
};

module.exports = RoverGoal;
