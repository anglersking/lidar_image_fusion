// Auto-generated. Do not edit!

// (in-package lidar_camera.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class calib_envluate {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.ave_deviation_z = null;
      this.ave_horizontal_reprojecterr = null;
      this.ave_vertical_reprojecterr = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('ave_deviation_z')) {
        this.ave_deviation_z = initObj.ave_deviation_z
      }
      else {
        this.ave_deviation_z = 0.0;
      }
      if (initObj.hasOwnProperty('ave_horizontal_reprojecterr')) {
        this.ave_horizontal_reprojecterr = initObj.ave_horizontal_reprojecterr
      }
      else {
        this.ave_horizontal_reprojecterr = 0.0;
      }
      if (initObj.hasOwnProperty('ave_vertical_reprojecterr')) {
        this.ave_vertical_reprojecterr = initObj.ave_vertical_reprojecterr
      }
      else {
        this.ave_vertical_reprojecterr = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type calib_envluate
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [ave_deviation_z]
    bufferOffset = _serializer.float32(obj.ave_deviation_z, buffer, bufferOffset);
    // Serialize message field [ave_horizontal_reprojecterr]
    bufferOffset = _serializer.float32(obj.ave_horizontal_reprojecterr, buffer, bufferOffset);
    // Serialize message field [ave_vertical_reprojecterr]
    bufferOffset = _serializer.float32(obj.ave_vertical_reprojecterr, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type calib_envluate
    let len;
    let data = new calib_envluate(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [ave_deviation_z]
    data.ave_deviation_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ave_horizontal_reprojecterr]
    data.ave_horizontal_reprojecterr = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ave_vertical_reprojecterr]
    data.ave_vertical_reprojecterr = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lidar_camera/calib_envluate';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1be80c75fd6a554f8f54217e7f0d9349';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    float32 ave_deviation_z
    float32 ave_horizontal_reprojecterr
    float32 ave_vertical_reprojecterr
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new calib_envluate(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.ave_deviation_z !== undefined) {
      resolved.ave_deviation_z = msg.ave_deviation_z;
    }
    else {
      resolved.ave_deviation_z = 0.0
    }

    if (msg.ave_horizontal_reprojecterr !== undefined) {
      resolved.ave_horizontal_reprojecterr = msg.ave_horizontal_reprojecterr;
    }
    else {
      resolved.ave_horizontal_reprojecterr = 0.0
    }

    if (msg.ave_vertical_reprojecterr !== undefined) {
      resolved.ave_vertical_reprojecterr = msg.ave_vertical_reprojecterr;
    }
    else {
      resolved.ave_vertical_reprojecterr = 0.0
    }

    return resolved;
    }
};

module.exports = calib_envluate;
