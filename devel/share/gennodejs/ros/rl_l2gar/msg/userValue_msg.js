// Auto-generated. Do not edit!

// (in-package rl_l2gar.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class userValue_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.lx = null;
      this.ly = null;
      this.rx = null;
      this.ry = null;
      this.L2 = null;
    }
    else {
      if (initObj.hasOwnProperty('lx')) {
        this.lx = initObj.lx
      }
      else {
        this.lx = 0.0;
      }
      if (initObj.hasOwnProperty('ly')) {
        this.ly = initObj.ly
      }
      else {
        this.ly = 0.0;
      }
      if (initObj.hasOwnProperty('rx')) {
        this.rx = initObj.rx
      }
      else {
        this.rx = 0.0;
      }
      if (initObj.hasOwnProperty('ry')) {
        this.ry = initObj.ry
      }
      else {
        this.ry = 0.0;
      }
      if (initObj.hasOwnProperty('L2')) {
        this.L2 = initObj.L2
      }
      else {
        this.L2 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type userValue_msg
    // Serialize message field [lx]
    bufferOffset = _serializer.float64(obj.lx, buffer, bufferOffset);
    // Serialize message field [ly]
    bufferOffset = _serializer.float64(obj.ly, buffer, bufferOffset);
    // Serialize message field [rx]
    bufferOffset = _serializer.float64(obj.rx, buffer, bufferOffset);
    // Serialize message field [ry]
    bufferOffset = _serializer.float64(obj.ry, buffer, bufferOffset);
    // Serialize message field [L2]
    bufferOffset = _serializer.float64(obj.L2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type userValue_msg
    let len;
    let data = new userValue_msg(null);
    // Deserialize message field [lx]
    data.lx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ly]
    data.ly = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rx]
    data.rx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ry]
    data.ry = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [L2]
    data.L2 = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rl_l2gar/userValue_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c5f52d6674a24214f88c039b8afc2bfa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 lx
    float64 ly
    float64 rx
    float64 ry
    float64 L2
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new userValue_msg(null);
    if (msg.lx !== undefined) {
      resolved.lx = msg.lx;
    }
    else {
      resolved.lx = 0.0
    }

    if (msg.ly !== undefined) {
      resolved.ly = msg.ly;
    }
    else {
      resolved.ly = 0.0
    }

    if (msg.rx !== undefined) {
      resolved.rx = msg.rx;
    }
    else {
      resolved.rx = 0.0
    }

    if (msg.ry !== undefined) {
      resolved.ry = msg.ry;
    }
    else {
      resolved.ry = 0.0
    }

    if (msg.L2 !== undefined) {
      resolved.L2 = msg.L2;
    }
    else {
      resolved.L2 = 0.0
    }

    return resolved;
    }
};

module.exports = userValue_msg;
