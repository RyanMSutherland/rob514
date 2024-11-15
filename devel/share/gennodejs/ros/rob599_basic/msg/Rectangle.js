// Auto-generated. Do not edit!

// (in-package rob599_basic.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Rectangle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.height = null;
      this.width = null;
    }
    else {
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0;
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Rectangle
    // Serialize message field [height]
    bufferOffset = _serializer.int64(obj.height, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.int64(obj.width, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Rectangle
    let len;
    let data = new Rectangle(null);
    // Deserialize message field [height]
    data.height = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rob599_basic/Rectangle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f2cea01c6068f229f4d26f3c1c2bd39c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 height
    int64 width
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Rectangle(null);
    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0
    }

    return resolved;
    }
};

module.exports = Rectangle;
