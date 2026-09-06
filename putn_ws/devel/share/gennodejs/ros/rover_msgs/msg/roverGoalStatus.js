// Auto-generated. Do not edit!

// (in-package rover_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class roverGoalStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.z = null;
      this.orientation_x = null;
      this.orientation_y = null;
      this.orientation_z = null;
      this.orientation_w = null;
      this.status = null;
      this.goal_id = null;
      this.text = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('orientation_x')) {
        this.orientation_x = initObj.orientation_x
      }
      else {
        this.orientation_x = 0.0;
      }
      if (initObj.hasOwnProperty('orientation_y')) {
        this.orientation_y = initObj.orientation_y
      }
      else {
        this.orientation_y = 0.0;
      }
      if (initObj.hasOwnProperty('orientation_z')) {
        this.orientation_z = initObj.orientation_z
      }
      else {
        this.orientation_z = 0.0;
      }
      if (initObj.hasOwnProperty('orientation_w')) {
        this.orientation_w = initObj.orientation_w
      }
      else {
        this.orientation_w = 0.0;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
      if (initObj.hasOwnProperty('goal_id')) {
        this.goal_id = initObj.goal_id
      }
      else {
        this.goal_id = 0;
      }
      if (initObj.hasOwnProperty('text')) {
        this.text = initObj.text
      }
      else {
        this.text = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type roverGoalStatus
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float64(obj.z, buffer, bufferOffset);
    // Serialize message field [orientation_x]
    bufferOffset = _serializer.float64(obj.orientation_x, buffer, bufferOffset);
    // Serialize message field [orientation_y]
    bufferOffset = _serializer.float64(obj.orientation_y, buffer, bufferOffset);
    // Serialize message field [orientation_z]
    bufferOffset = _serializer.float64(obj.orientation_z, buffer, bufferOffset);
    // Serialize message field [orientation_w]
    bufferOffset = _serializer.float64(obj.orientation_w, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.uint8(obj.status, buffer, bufferOffset);
    // Serialize message field [goal_id]
    bufferOffset = _serializer.uint8(obj.goal_id, buffer, bufferOffset);
    // Serialize message field [text]
    bufferOffset = _serializer.string(obj.text, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type roverGoalStatus
    let len;
    let data = new roverGoalStatus(null);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [orientation_x]
    data.orientation_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [orientation_y]
    data.orientation_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [orientation_z]
    data.orientation_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [orientation_w]
    data.orientation_w = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [goal_id]
    data.goal_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [text]
    data.text = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.text);
    return length + 62;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rover_msgs/roverGoalStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '06fdaffd0dd12907d78dce66dec8b35a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 PENDING=0
    uint8 ACTIVE=1
    uint8 PREEMPTED=2
    uint8 SUCCEEDED=3
    uint8 ABORTED=4
    uint8 REJECTED=5
    uint8 PREEMPTING=6
    uint8 RECALLING=7
    uint8 RECALLED=8
    uint8 LOST=9
    
    
    float64 x
    float64 y
    float64 z
    float64 orientation_x
    float64 orientation_y
    float64 orientation_z
    float64 orientation_w
    uint8 status
    uint8 goal_id
    string text
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new roverGoalStatus(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.orientation_x !== undefined) {
      resolved.orientation_x = msg.orientation_x;
    }
    else {
      resolved.orientation_x = 0.0
    }

    if (msg.orientation_y !== undefined) {
      resolved.orientation_y = msg.orientation_y;
    }
    else {
      resolved.orientation_y = 0.0
    }

    if (msg.orientation_z !== undefined) {
      resolved.orientation_z = msg.orientation_z;
    }
    else {
      resolved.orientation_z = 0.0
    }

    if (msg.orientation_w !== undefined) {
      resolved.orientation_w = msg.orientation_w;
    }
    else {
      resolved.orientation_w = 0.0
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    if (msg.goal_id !== undefined) {
      resolved.goal_id = msg.goal_id;
    }
    else {
      resolved.goal_id = 0
    }

    if (msg.text !== undefined) {
      resolved.text = msg.text;
    }
    else {
      resolved.text = ''
    }

    return resolved;
    }
};

// Constants for message
roverGoalStatus.Constants = {
  PENDING: 0,
  ACTIVE: 1,
  PREEMPTED: 2,
  SUCCEEDED: 3,
  ABORTED: 4,
  REJECTED: 5,
  PREEMPTING: 6,
  RECALLING: 7,
  RECALLED: 8,
  LOST: 9,
}

module.exports = roverGoalStatus;
