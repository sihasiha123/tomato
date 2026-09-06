// Auto-generated. Do not edit!

// (in-package rover_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class roverRCStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.connect = null;
      this.CH1 = null;
      this.CH2 = null;
      this.CH3 = null;
      this.CH4 = null;
      this.CH5 = null;
      this.CH6 = null;
      this.CH7 = null;
      this.CH8 = null;
      this.CH9 = null;
      this.CH10 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('connect')) {
        this.connect = initObj.connect
      }
      else {
        this.connect = false;
      }
      if (initObj.hasOwnProperty('CH1')) {
        this.CH1 = initObj.CH1
      }
      else {
        this.CH1 = 0;
      }
      if (initObj.hasOwnProperty('CH2')) {
        this.CH2 = initObj.CH2
      }
      else {
        this.CH2 = 0;
      }
      if (initObj.hasOwnProperty('CH3')) {
        this.CH3 = initObj.CH3
      }
      else {
        this.CH3 = 0;
      }
      if (initObj.hasOwnProperty('CH4')) {
        this.CH4 = initObj.CH4
      }
      else {
        this.CH4 = 0;
      }
      if (initObj.hasOwnProperty('CH5')) {
        this.CH5 = initObj.CH5
      }
      else {
        this.CH5 = 0;
      }
      if (initObj.hasOwnProperty('CH6')) {
        this.CH6 = initObj.CH6
      }
      else {
        this.CH6 = 0;
      }
      if (initObj.hasOwnProperty('CH7')) {
        this.CH7 = initObj.CH7
      }
      else {
        this.CH7 = 0;
      }
      if (initObj.hasOwnProperty('CH8')) {
        this.CH8 = initObj.CH8
      }
      else {
        this.CH8 = 0;
      }
      if (initObj.hasOwnProperty('CH9')) {
        this.CH9 = initObj.CH9
      }
      else {
        this.CH9 = 0;
      }
      if (initObj.hasOwnProperty('CH10')) {
        this.CH10 = initObj.CH10
      }
      else {
        this.CH10 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type roverRCStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [connect]
    bufferOffset = _serializer.bool(obj.connect, buffer, bufferOffset);
    // Serialize message field [CH1]
    bufferOffset = _serializer.int8(obj.CH1, buffer, bufferOffset);
    // Serialize message field [CH2]
    bufferOffset = _serializer.int8(obj.CH2, buffer, bufferOffset);
    // Serialize message field [CH3]
    bufferOffset = _serializer.int8(obj.CH3, buffer, bufferOffset);
    // Serialize message field [CH4]
    bufferOffset = _serializer.int8(obj.CH4, buffer, bufferOffset);
    // Serialize message field [CH5]
    bufferOffset = _serializer.int8(obj.CH5, buffer, bufferOffset);
    // Serialize message field [CH6]
    bufferOffset = _serializer.int8(obj.CH6, buffer, bufferOffset);
    // Serialize message field [CH7]
    bufferOffset = _serializer.int8(obj.CH7, buffer, bufferOffset);
    // Serialize message field [CH8]
    bufferOffset = _serializer.int8(obj.CH8, buffer, bufferOffset);
    // Serialize message field [CH9]
    bufferOffset = _serializer.int8(obj.CH9, buffer, bufferOffset);
    // Serialize message field [CH10]
    bufferOffset = _serializer.int8(obj.CH10, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type roverRCStatus
    let len;
    let data = new roverRCStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [connect]
    data.connect = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [CH1]
    data.CH1 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [CH2]
    data.CH2 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [CH3]
    data.CH3 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [CH4]
    data.CH4 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [CH5]
    data.CH5 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [CH6]
    data.CH6 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [CH7]
    data.CH7 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [CH8]
    data.CH8 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [CH9]
    data.CH9 = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [CH10]
    data.CH10 = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 11;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rover_msgs/roverRCStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c06b6cb58b0a81562a8993101c944318';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    bool connect
    int8 CH1
    int8 CH2
    int8 CH3
    int8 CH4
    int8 CH5
    int8 CH6
    int8 CH7
    int8 CH8
    int8 CH9
    int8 CH10
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new roverRCStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.connect !== undefined) {
      resolved.connect = msg.connect;
    }
    else {
      resolved.connect = false
    }

    if (msg.CH1 !== undefined) {
      resolved.CH1 = msg.CH1;
    }
    else {
      resolved.CH1 = 0
    }

    if (msg.CH2 !== undefined) {
      resolved.CH2 = msg.CH2;
    }
    else {
      resolved.CH2 = 0
    }

    if (msg.CH3 !== undefined) {
      resolved.CH3 = msg.CH3;
    }
    else {
      resolved.CH3 = 0
    }

    if (msg.CH4 !== undefined) {
      resolved.CH4 = msg.CH4;
    }
    else {
      resolved.CH4 = 0
    }

    if (msg.CH5 !== undefined) {
      resolved.CH5 = msg.CH5;
    }
    else {
      resolved.CH5 = 0
    }

    if (msg.CH6 !== undefined) {
      resolved.CH6 = msg.CH6;
    }
    else {
      resolved.CH6 = 0
    }

    if (msg.CH7 !== undefined) {
      resolved.CH7 = msg.CH7;
    }
    else {
      resolved.CH7 = 0
    }

    if (msg.CH8 !== undefined) {
      resolved.CH8 = msg.CH8;
    }
    else {
      resolved.CH8 = 0
    }

    if (msg.CH9 !== undefined) {
      resolved.CH9 = msg.CH9;
    }
    else {
      resolved.CH9 = 0
    }

    if (msg.CH10 !== undefined) {
      resolved.CH10 = msg.CH10;
    }
    else {
      resolved.CH10 = 0
    }

    return resolved;
    }
};

module.exports = roverRCStatus;
