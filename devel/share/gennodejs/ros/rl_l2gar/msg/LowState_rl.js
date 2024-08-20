// Auto-generated. Do not edit!

// (in-package rl_l2gar.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let userValue_msg = require('./userValue_msg.js');
let unitree_legged_msgs = _finder('unitree_legged_msgs');

//-----------------------------------------------------------

class LowState_rl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.imu = null;
      this.motorState = null;
      this.userValue = null;
      this.userCmd = null;
    }
    else {
      if (initObj.hasOwnProperty('imu')) {
        this.imu = initObj.imu
      }
      else {
        this.imu = new unitree_legged_msgs.msg.IMU();
      }
      if (initObj.hasOwnProperty('motorState')) {
        this.motorState = initObj.motorState
      }
      else {
        this.motorState = new Array(20).fill(new unitree_legged_msgs.msg.MotorState());
      }
      if (initObj.hasOwnProperty('userValue')) {
        this.userValue = initObj.userValue
      }
      else {
        this.userValue = new userValue_msg();
      }
      if (initObj.hasOwnProperty('userCmd')) {
        this.userCmd = initObj.userCmd
      }
      else {
        this.userCmd = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LowState_rl
    // Serialize message field [imu]
    bufferOffset = unitree_legged_msgs.msg.IMU.serialize(obj.imu, buffer, bufferOffset);
    // Check that the constant length array field [motorState] has the right length
    if (obj.motorState.length !== 20) {
      throw new Error('Unable to serialize array field motorState - length must be 20')
    }
    // Serialize message field [motorState]
    obj.motorState.forEach((val) => {
      bufferOffset = unitree_legged_msgs.msg.MotorState.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [userValue]
    bufferOffset = userValue_msg.serialize(obj.userValue, buffer, bufferOffset);
    // Serialize message field [userCmd]
    bufferOffset = _serializer.int32(obj.userCmd, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LowState_rl
    let len;
    let data = new LowState_rl(null);
    // Deserialize message field [imu]
    data.imu = unitree_legged_msgs.msg.IMU.deserialize(buffer, bufferOffset);
    // Deserialize message field [motorState]
    len = 20;
    data.motorState = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.motorState[i] = unitree_legged_msgs.msg.MotorState.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [userValue]
    data.userValue = userValue_msg.deserialize(buffer, bufferOffset);
    // Deserialize message field [userCmd]
    data.userCmd = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 123;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rl_l2gar/LowState_rl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b77f6762640229fefac4685d9f05c4e0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    unitree_legged_msgs/IMU imu
    unitree_legged_msgs/MotorState[20] motorState
    userValue_msg userValue
    int32 userCmd
    ================================================================================
    MSG: unitree_legged_msgs/IMU
    float32[4] quaternion
    float32[3] gyroscope
    float32[3] accelerometer
    int8 temperature
    ================================================================================
    MSG: unitree_legged_msgs/MotorState
    uint8 mode           # motor current mode 
    float32 q            # motor current position（rad）
    float32 dq           # motor current speed（rad/s）
    float32 ddq          # motor current speed（rad/s）
    float32 tauEst       # current estimated output torque（N*m）
    float32 q_raw        # motor current position（rad）
    float32 dq_raw       # motor current speed（rad/s）
    float32 ddq_raw      # motor current speed（rad/s）
    int8 temperature     # motor temperature（slow conduction of temperature leads to lag）
    uint32[2] reserve
    ================================================================================
    MSG: rl_l2gar/userValue_msg
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
    const resolved = new LowState_rl(null);
    if (msg.imu !== undefined) {
      resolved.imu = unitree_legged_msgs.msg.IMU.Resolve(msg.imu)
    }
    else {
      resolved.imu = new unitree_legged_msgs.msg.IMU()
    }

    if (msg.motorState !== undefined) {
      resolved.motorState = new Array(20)
      for (let i = 0; i < resolved.motorState.length; ++i) {
        if (msg.motorState.length > i) {
          resolved.motorState[i] = unitree_legged_msgs.msg.MotorState.Resolve(msg.motorState[i]);
        }
        else {
          resolved.motorState[i] = new unitree_legged_msgs.msg.MotorState();
        }
      }
    }
    else {
      resolved.motorState = new Array(20).fill(new unitree_legged_msgs.msg.MotorState())
    }

    if (msg.userValue !== undefined) {
      resolved.userValue = userValue_msg.Resolve(msg.userValue)
    }
    else {
      resolved.userValue = new userValue_msg()
    }

    if (msg.userCmd !== undefined) {
      resolved.userCmd = msg.userCmd;
    }
    else {
      resolved.userCmd = 0
    }

    return resolved;
    }
};

module.exports = LowState_rl;
