
"use strict";

let HeadPanCommand = require('./HeadPanCommand.js');
let DigitalIOState = require('./DigitalIOState.js');
let AnalogIOStates = require('./AnalogIOStates.js');
let CameraSettings = require('./CameraSettings.js');
let EndpointStates = require('./EndpointStates.js');
let CollisionAvoidanceState = require('./CollisionAvoidanceState.js');
let NavigatorState = require('./NavigatorState.js');
let EndEffectorCommand = require('./EndEffectorCommand.js');
let AnalogOutputCommand = require('./AnalogOutputCommand.js');
let EndEffectorState = require('./EndEffectorState.js');
let DigitalIOStates = require('./DigitalIOStates.js');
let JointCommand = require('./JointCommand.js');
let RobustControllerStatus = require('./RobustControllerStatus.js');
let SEAJointState = require('./SEAJointState.js');
let EndEffectorProperties = require('./EndEffectorProperties.js');
let EndpointState = require('./EndpointState.js');
let URDFConfiguration = require('./URDFConfiguration.js');
let AssemblyStates = require('./AssemblyStates.js');
let DigitalOutputCommand = require('./DigitalOutputCommand.js');
let NavigatorStates = require('./NavigatorStates.js');
let CameraControl = require('./CameraControl.js');
let AssemblyState = require('./AssemblyState.js');
let CollisionDetectionState = require('./CollisionDetectionState.js');
let AnalogIOState = require('./AnalogIOState.js');
let HeadState = require('./HeadState.js');

module.exports = {
  HeadPanCommand: HeadPanCommand,
  DigitalIOState: DigitalIOState,
  AnalogIOStates: AnalogIOStates,
  CameraSettings: CameraSettings,
  EndpointStates: EndpointStates,
  CollisionAvoidanceState: CollisionAvoidanceState,
  NavigatorState: NavigatorState,
  EndEffectorCommand: EndEffectorCommand,
  AnalogOutputCommand: AnalogOutputCommand,
  EndEffectorState: EndEffectorState,
  DigitalIOStates: DigitalIOStates,
  JointCommand: JointCommand,
  RobustControllerStatus: RobustControllerStatus,
  SEAJointState: SEAJointState,
  EndEffectorProperties: EndEffectorProperties,
  EndpointState: EndpointState,
  URDFConfiguration: URDFConfiguration,
  AssemblyStates: AssemblyStates,
  DigitalOutputCommand: DigitalOutputCommand,
  NavigatorStates: NavigatorStates,
  CameraControl: CameraControl,
  AssemblyState: AssemblyState,
  CollisionDetectionState: CollisionDetectionState,
  AnalogIOState: AnalogIOState,
  HeadState: HeadState,
};
