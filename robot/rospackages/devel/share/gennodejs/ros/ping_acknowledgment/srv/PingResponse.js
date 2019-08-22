// Auto-generated. Do not edit!

// (in-package ping_acknowledgment.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class PingResponseRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ping = null;
    }
    else {
      if (initObj.hasOwnProperty('ping')) {
        this.ping = initObj.ping
      }
      else {
        this.ping = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PingResponseRequest
    // Serialize message field [ping]
    bufferOffset = _serializer.string(obj.ping, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PingResponseRequest
    let len;
    let data = new PingResponseRequest(null);
    // Deserialize message field [ping]
    data.ping = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.ping.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ping_acknowledgment/PingResponseRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b8f8c3756a1817228220197729a1b95e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string ping
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PingResponseRequest(null);
    if (msg.ping !== undefined) {
      resolved.ping = msg.ping;
    }
    else {
      resolved.ping = ''
    }

    return resolved;
    }
};

class PingResponseResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.response = null;
    }
    else {
      if (initObj.hasOwnProperty('response')) {
        this.response = initObj.response
      }
      else {
        this.response = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PingResponseResponse
    // Serialize message field [response]
    bufferOffset = _serializer.string(obj.response, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PingResponseResponse
    let len;
    let data = new PingResponseResponse(null);
    // Deserialize message field [response]
    data.response = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.response.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ping_acknowledgment/PingResponseResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6de314e2dc76fbff2b6244a6ad70b68d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string response
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PingResponseResponse(null);
    if (msg.response !== undefined) {
      resolved.response = msg.response;
    }
    else {
      resolved.response = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: PingResponseRequest,
  Response: PingResponseResponse,
  md5sum() { return '2c33f5a2bff59c326ae8656e9ffa7758'; },
  datatype() { return 'ping_acknowledgment/PingResponse'; }
};
