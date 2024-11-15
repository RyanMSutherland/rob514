// Auto-generated. Do not edit!

// (in-package rob599_basic.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class DoublerRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.number = null;
    }
    else {
      if (initObj.hasOwnProperty('number')) {
        this.number = initObj.number
      }
      else {
        this.number = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DoublerRequest
    // Serialize message field [number]
    bufferOffset = _serializer.int64(obj.number, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DoublerRequest
    let len;
    let data = new DoublerRequest(null);
    // Deserialize message field [number]
    data.number = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rob599_basic/DoublerRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2a1b249ff0900a6eb941a743f359017a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # inputs
    int64 number
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DoublerRequest(null);
    if (msg.number !== undefined) {
      resolved.number = msg.number;
    }
    else {
      resolved.number = 0
    }

    return resolved;
    }
};

class DoublerResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.doubled = null;
    }
    else {
      if (initObj.hasOwnProperty('doubled')) {
        this.doubled = initObj.doubled
      }
      else {
        this.doubled = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DoublerResponse
    // Serialize message field [doubled]
    bufferOffset = _serializer.int64(obj.doubled, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DoublerResponse
    let len;
    let data = new DoublerResponse(null);
    // Deserialize message field [doubled]
    data.doubled = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rob599_basic/DoublerResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ff7f570e39c1297ac8f5722d035ee11c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # outputs
    int64 doubled
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DoublerResponse(null);
    if (msg.doubled !== undefined) {
      resolved.doubled = msg.doubled;
    }
    else {
      resolved.doubled = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: DoublerRequest,
  Response: DoublerResponse,
  md5sum() { return 'cd11c21fe194ec0173733fddd2cd457b'; },
  datatype() { return 'rob599_basic/Doubler'; }
};
