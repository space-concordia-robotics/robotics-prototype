// Auto-generated. Do not edit!

// (in-package tf2_web_republisher.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class RepublishTFsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.source_frames = null;
      this.target_frame = null;
      this.angular_thres = null;
      this.trans_thres = null;
      this.rate = null;
      this.timeout = null;
    }
    else {
      if (initObj.hasOwnProperty('source_frames')) {
        this.source_frames = initObj.source_frames
      }
      else {
        this.source_frames = [];
      }
      if (initObj.hasOwnProperty('target_frame')) {
        this.target_frame = initObj.target_frame
      }
      else {
        this.target_frame = '';
      }
      if (initObj.hasOwnProperty('angular_thres')) {
        this.angular_thres = initObj.angular_thres
      }
      else {
        this.angular_thres = 0.0;
      }
      if (initObj.hasOwnProperty('trans_thres')) {
        this.trans_thres = initObj.trans_thres
      }
      else {
        this.trans_thres = 0.0;
      }
      if (initObj.hasOwnProperty('rate')) {
        this.rate = initObj.rate
      }
      else {
        this.rate = 0.0;
      }
      if (initObj.hasOwnProperty('timeout')) {
        this.timeout = initObj.timeout
      }
      else {
        this.timeout = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RepublishTFsRequest
    // Serialize message field [source_frames]
    bufferOffset = _arraySerializer.string(obj.source_frames, buffer, bufferOffset, null);
    // Serialize message field [target_frame]
    bufferOffset = _serializer.string(obj.target_frame, buffer, bufferOffset);
    // Serialize message field [angular_thres]
    bufferOffset = _serializer.float32(obj.angular_thres, buffer, bufferOffset);
    // Serialize message field [trans_thres]
    bufferOffset = _serializer.float32(obj.trans_thres, buffer, bufferOffset);
    // Serialize message field [rate]
    bufferOffset = _serializer.float32(obj.rate, buffer, bufferOffset);
    // Serialize message field [timeout]
    bufferOffset = _serializer.duration(obj.timeout, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RepublishTFsRequest
    let len;
    let data = new RepublishTFsRequest(null);
    // Deserialize message field [source_frames]
    data.source_frames = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [target_frame]
    data.target_frame = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [angular_thres]
    data.angular_thres = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [trans_thres]
    data.trans_thres = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rate]
    data.rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [timeout]
    data.timeout = _deserializer.duration(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.source_frames.forEach((val) => {
      length += 4 + val.length;
    });
    length += object.target_frame.length;
    return length + 28;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tf2_web_republisher/RepublishTFsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f13b5a5a70ee41b437384d6715cbcd91';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    string[] source_frames
    string target_frame
    float32 angular_thres
    float32 trans_thres
    float32 rate
    duration timeout
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RepublishTFsRequest(null);
    if (msg.source_frames !== undefined) {
      resolved.source_frames = msg.source_frames;
    }
    else {
      resolved.source_frames = []
    }

    if (msg.target_frame !== undefined) {
      resolved.target_frame = msg.target_frame;
    }
    else {
      resolved.target_frame = ''
    }

    if (msg.angular_thres !== undefined) {
      resolved.angular_thres = msg.angular_thres;
    }
    else {
      resolved.angular_thres = 0.0
    }

    if (msg.trans_thres !== undefined) {
      resolved.trans_thres = msg.trans_thres;
    }
    else {
      resolved.trans_thres = 0.0
    }

    if (msg.rate !== undefined) {
      resolved.rate = msg.rate;
    }
    else {
      resolved.rate = 0.0
    }

    if (msg.timeout !== undefined) {
      resolved.timeout = msg.timeout;
    }
    else {
      resolved.timeout = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

class RepublishTFsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.topic_name = null;
    }
    else {
      if (initObj.hasOwnProperty('topic_name')) {
        this.topic_name = initObj.topic_name
      }
      else {
        this.topic_name = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RepublishTFsResponse
    // Serialize message field [topic_name]
    bufferOffset = _serializer.string(obj.topic_name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RepublishTFsResponse
    let len;
    let data = new RepublishTFsResponse(null);
    // Deserialize message field [topic_name]
    data.topic_name = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.topic_name.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tf2_web_republisher/RepublishTFsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b38cc2f19f45368c2db7867751ce95a9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    string topic_name
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RepublishTFsResponse(null);
    if (msg.topic_name !== undefined) {
      resolved.topic_name = msg.topic_name;
    }
    else {
      resolved.topic_name = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: RepublishTFsRequest,
  Response: RepublishTFsResponse,
  md5sum() { return 'ec8570dea2e6015c309eb6611d1a57d0'; },
  datatype() { return 'tf2_web_republisher/RepublishTFs'; }
};
