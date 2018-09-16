// Auto-generated. Do not edit!

// (in-package action_primitive_variation.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class PushButtonRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.button_name = null;
    }
    else {
      if (initObj.hasOwnProperty('button_name')) {
        this.button_name = initObj.button_name
      }
      else {
        this.button_name = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PushButtonRequest
    // Serialize message field [button_name]
    bufferOffset = _serializer.string(obj.button_name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PushButtonRequest
    let len;
    let data = new PushButtonRequest(null);
    // Deserialize message field [button_name]
    data.button_name = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.button_name.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'action_primitive_variation/PushButtonRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c456d5069b47e76bc7a03939bbb2aaf4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string button_name
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PushButtonRequest(null);
    if (msg.button_name !== undefined) {
      resolved.button_name = msg.button_name;
    }
    else {
      resolved.button_name = ''
    }

    return resolved;
    }
};

class PushButtonResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success_bool = null;
    }
    else {
      if (initObj.hasOwnProperty('success_bool')) {
        this.success_bool = initObj.success_bool
      }
      else {
        this.success_bool = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PushButtonResponse
    // Serialize message field [success_bool]
    bufferOffset = _serializer.int64(obj.success_bool, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PushButtonResponse
    let len;
    let data = new PushButtonResponse(null);
    // Deserialize message field [success_bool]
    data.success_bool = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'action_primitive_variation/PushButtonResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6e72685e7a815ec68561aa624b91e276';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 success_bool
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PushButtonResponse(null);
    if (msg.success_bool !== undefined) {
      resolved.success_bool = msg.success_bool;
    }
    else {
      resolved.success_bool = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: PushButtonRequest,
  Response: PushButtonResponse,
  md5sum() { return '06b0fa47749790f71d50d61e72fea1cc'; },
  datatype() { return 'action_primitive_variation/PushButton'; }
};
