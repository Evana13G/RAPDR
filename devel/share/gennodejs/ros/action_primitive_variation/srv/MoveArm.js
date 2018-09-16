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

class MoveArmRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.location = null;
    }
    else {
      if (initObj.hasOwnProperty('location')) {
        this.location = initObj.location
      }
      else {
        this.location = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MoveArmRequest
    // Serialize message field [location]
    bufferOffset = _serializer.string(obj.location, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveArmRequest
    let len;
    let data = new MoveArmRequest(null);
    // Deserialize message field [location]
    data.location = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.location.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'action_primitive_variation/MoveArmRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '03da474bc61cfeb81a8854b4ca05bafa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string location
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MoveArmRequest(null);
    if (msg.location !== undefined) {
      resolved.location = msg.location;
    }
    else {
      resolved.location = ''
    }

    return resolved;
    }
};

class MoveArmResponse {
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
    // Serializes a message object of type MoveArmResponse
    // Serialize message field [success_bool]
    bufferOffset = _serializer.int64(obj.success_bool, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MoveArmResponse
    let len;
    let data = new MoveArmResponse(null);
    // Deserialize message field [success_bool]
    data.success_bool = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'action_primitive_variation/MoveArmResponse';
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
    const resolved = new MoveArmResponse(null);
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
  Request: MoveArmRequest,
  Response: MoveArmResponse,
  md5sum() { return 'dabfe91d481ed6e7759f42648158bcae'; },
  datatype() { return 'action_primitive_variation/MoveArm'; }
};
