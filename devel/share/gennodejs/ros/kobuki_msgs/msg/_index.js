
"use strict";

let ButtonEvent = require('./ButtonEvent.js');
let Sound = require('./Sound.js');
let DigitalOutput = require('./DigitalOutput.js');
let DigitalInputEvent = require('./DigitalInputEvent.js');
let BumperEvent = require('./BumperEvent.js');
let Led = require('./Led.js');
let MotorPower = require('./MotorPower.js');
let PowerSystemEvent = require('./PowerSystemEvent.js');
let DockInfraRed = require('./DockInfraRed.js');
let VersionInfo = require('./VersionInfo.js');
let SensorState = require('./SensorState.js');
let ScanAngle = require('./ScanAngle.js');
let RobotStateEvent = require('./RobotStateEvent.js');
let ControllerInfo = require('./ControllerInfo.js');
let CliffEvent = require('./CliffEvent.js');
let ExternalPower = require('./ExternalPower.js');
let KeyboardInput = require('./KeyboardInput.js');
let WheelDropEvent = require('./WheelDropEvent.js');
let AutoDockingAction = require('./AutoDockingAction.js');
let AutoDockingActionFeedback = require('./AutoDockingActionFeedback.js');
let AutoDockingGoal = require('./AutoDockingGoal.js');
let AutoDockingActionGoal = require('./AutoDockingActionGoal.js');
let AutoDockingResult = require('./AutoDockingResult.js');
let AutoDockingFeedback = require('./AutoDockingFeedback.js');
let AutoDockingActionResult = require('./AutoDockingActionResult.js');

module.exports = {
  ButtonEvent: ButtonEvent,
  Sound: Sound,
  DigitalOutput: DigitalOutput,
  DigitalInputEvent: DigitalInputEvent,
  BumperEvent: BumperEvent,
  Led: Led,
  MotorPower: MotorPower,
  PowerSystemEvent: PowerSystemEvent,
  DockInfraRed: DockInfraRed,
  VersionInfo: VersionInfo,
  SensorState: SensorState,
  ScanAngle: ScanAngle,
  RobotStateEvent: RobotStateEvent,
  ControllerInfo: ControllerInfo,
  CliffEvent: CliffEvent,
  ExternalPower: ExternalPower,
  KeyboardInput: KeyboardInput,
  WheelDropEvent: WheelDropEvent,
  AutoDockingAction: AutoDockingAction,
  AutoDockingActionFeedback: AutoDockingActionFeedback,
  AutoDockingGoal: AutoDockingGoal,
  AutoDockingActionGoal: AutoDockingActionGoal,
  AutoDockingResult: AutoDockingResult,
  AutoDockingFeedback: AutoDockingFeedback,
  AutoDockingActionResult: AutoDockingActionResult,
};
