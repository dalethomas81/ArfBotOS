/**
  Copyright (C) 2012-2023 by Autodesk, Inc.
  All rights reserved.

  Deposition sample post processor configuration.

  $Revision: 44063 b4957549e0a71dca83e72413d3bbd824a1846569 $
  $Date: 2023-04-28 12:14:21 $

  FORKID {5CC0B63E-B1A0-4272-BD17-29A7C354EE2B}
*/

description = "ArfBotOS post processor";
vendor = "Autodesk";
vendorUrl = "http://www.autodesk.com";
legal = "Copyright (C) 2012-2023 by Autodesk, Inc.";
certificationLevel = 2;
minimumRevision = 45834;
longDescription = "ArfBotOS post processor.";

extension = "cnc";
setCodePage("ascii");

capabilities = CAPABILITY_MILLING;
tolerance = spatial(0.002, MM);

minimumChordLength = spatial(0.25, MM);
minimumCircularRadius = spatial(0.01, MM);
maximumCircularRadius = spatial(1000, MM);
minimumCircularSweep = toRad(0.01);
maximumCircularSweep = toRad(180);
allowHelicalMoves = true;
allowedCircularPlanes = undefined; // allow any circular motion

var gFormat = createFormat({prefix:"G", decimals:1});
var mFormat = createFormat({prefix:"M", decimals:0});
var xyzFormat = createFormat({decimals:(unit == MM ? 3 : 4)});
var abcFormat = createFormat({decimals:3, forceDecimal:true, scale:DEG});
var feedFormat = createFormat({decimals:(unit == MM ? 1 : 2)});

var xOutput = createVariable({prefix:"X"}, xyzFormat);
var yOutput = createVariable({prefix:"Y"}, xyzFormat);
var zOutput = createVariable({prefix:"Z"}, xyzFormat);
var aOutput = createVariable({prefix:"A"}, abcFormat);
var bOutput = createVariable({prefix:"B"}, abcFormat);
var cOutput = createVariable({prefix:"C"}, abcFormat);
var feedOutput = createVariable({prefix:"F"}, feedFormat);

var gMotionModal = createModal({}, gFormat);

function onSection() {
  // #### Add the code below into the onSection function of your postprocessor ####
  if (isDepositionOperation()) {
    setDepositionCommands(); // setup for deposition process parameters, specify your commands in that function
    writeDepositionHeader(tool);
    writeProcessEquipmentCommands(true);
  }

  // Important note, make sure that you disable the spindle speed output for deposition operations in your postprocessor.
}

function onSectionEnd() {
  // #### Add the code below into the onSectionEnd function of your postprocessor ####
  if (isDepositionOperation()) {
    writeProcessEquipmentCommands(false);
  }
}

// #### If your postprocessor does not have the onMovement function, you have to add the entire function below. ####
function onMovement(movement) {
  // #### Add the code below into the onMovement function of your postprocessor. ####
  if (isDepositionOperation()) {
    onMovementDeposition(movement);
  }
}

// #### Add the entire code beginning from 'Start of Deposition logic' to 'End of Deposition logic' at the end of your postprocessor ####

// Start of Deposition logic
minimumRevision = minimumRevision < 45834 ? 45834 : minimumRevision;
capabilities |= CAPABILITY_ADDITIVE;

if (typeof properties != "object") {
  properties = {};
}

properties.depositOnTransitions = {
  title      : "Deposit during transitions",
  description: "Keep Deposition ON during transition moves.",
  type       : "boolean",
  value      : false,
  scope      : "post",
  group      : "preferences"
};

var commands = {};
function setDepositionCommands() {
  getProcessParameters(); // update process parameters for the current section
  switch (tool.type) {
  case TOOL_DEPOSITING_ELECTRIC_ARC_WIRE:
    // insert startup codes for electric arc wire here
    commands = {
      deposition      : {on:mFormat.format(101), off:mFormat.format(103)},
      processEquipment: {
        on: [  // commands to turn on process equipment
          formatWords(gFormat.format(90), formatComment("ABSOLUTE MODE")),
          formatWords(gFormat.format(300), "F" + processParameters.gasFlowRate, formatComment("SHIELD GAS FLOW RATE")),
          formatWords(gFormat.format(301), "V" + processParameters.arcCurrent, formatComment("ARC VOLTAGE")),
          formatWords(gFormat.format(302), "A" + processParameters.arcVoltage, formatComment("ARC CURRENT")),
          formatWords(gFormat.format(303), "S" + processParameters.wireSpeed, formatComment("WIRE SPEED")),
          formatWords(mFormat.format(304), formatComment("PROCESS ON"))],
        off: [  // commands to turn off process equipment
          formatWords(mFormat.format(305), formatComment("PROCESS OFF")),
          formatWords(gFormat.format(303), "S0", formatComment("WIRE STOP")),
          formatWords(gFormat.format(300), "F0000", formatComment("GAS OFF"))]
      }
    };
    break;
  case TOOL_DEPOSITING_LASER_POWDER:
    // insert startup codes for laser powder here
    commands = {
      deposition      : {on:mFormat.format(441), off:mFormat.format(442)},
      processEquipment: {
        on: [  // commands to turn on process equipment
          formatWords(mFormat.format(444), formatComment("CLADDING DOOR OPEN")),
          formatWords(gFormat.format(0), gFormat.format(90), "W-400.", formatComment("POWDER HEAD TO POSITION")),
          formatWords(gFormat.format(910),
            "E" + processParameters.depositionPower,
            "I" + processParameters.gasFlowRate,
            "J" + processParameters.auxGasFlowRate,
            "R" + processParameters.powderFlowRate,
            "U" + processParameters.carrierGasFlowRate),
          formatWords(mFormat.format(440), formatComment("NOZZLE GAS ON")),
          formatWords(mFormat.format(441), formatComment("SHIELD GAS ON")),
          formatWords(gFormat.format(4), gFormat.format(94), "X" + spatialFormat.format(3), formatComment("WAIT 3s")),
          formatWords(gFormat.format(441), "H1", formatComment("HOPPER 1 ON")),
          formatWords(gFormat.format(4), gFormat.format(94), "X" + spatialFormat.format(20), formatComment("WAIT 20s")),
          formatWords(gFormat.format(460), formatComment("WORK SHIFT FOR AM HEAD"))],
        off: [  // commands to turn off process equipment
          formatWords(mFormat.format(452), formatComment("DISC STOP")),
          formatWords(mFormat.format(426), formatComment("CARRIER GAS OFF")),
          formatWords(gFormat.format(442), formatComment("NOZZLE SHIELD GAS OFF")),
          formatWords(gFormat.format(4), gFormat.format(94), "X" + spatialFormat.format(15), formatComment("WAIT 15s")),
          formatWords(gFormat.format(0), gFormat.format(90), "W0.", formatComment("CLADDING HEAD HOME POSITION")),
          formatWords(mFormat.format(445), formatComment("CLADDING DOOR CLOSE")),
          formatWords(mFormat.format(461), formatComment("WORK SHIFT FOR AM HEAD CANCEL"))]
      }
    };
    break;
  case TOOL_DEPOSITING_LASER_WIRE:
    // insert startup codes for laser wire here
    commands = {
      deposition      : {on:mFormat.format(120), off:mFormat.format(121)},
      processEquipment: {
        on: [  // commands to turn on process equipment
          formatWords(gFormat.format(90), formatComment("ABSOLUTE MODE")),
          formatWords(gFormat.format(500), "A" + processParameters.gasFlowRate, formatComment("SHIELDING GAS FLOW RATE")),
          formatWords(gFormat.format(500), "B" + processParameters.auxGasFlowRate, formatComment("AUXILIARY GAS FLOW RATE")),
          formatWords(gFormat.format(500), "S" + processParameters.wireSpeed, formatComment("WIRE SPEED")),
          formatWords(mFormat.format(504), "K" + processParameters.depositionPower, formatComment("POWER"))],
        off: [  // commands to turn off process equipment
          formatWords(gFormat.format(500), "A0", formatComment("SHIELD GAS FLOW RATE")),
          formatWords(gFormat.format(500), "B0", formatComment("AUXILIARY GAS")),
          formatWords(gFormat.format(500), "S0", formatComment("WIRE STOP")),
          formatWords(mFormat.format(504), "K0", formatComment("POWER"))]
      }
    };
    break;
  default:
    error(localize("Unsupported deposition tool type."));
    return;
  }
}

if (typeof spatialFormat == "undefined") {
  var spatialFormat = createFormat({decimals:(unit == MM ? 3 : 4)});
}

/** Get process parameters from tool parameters, post properties */
var processParameters = {};
function getProcessParameters() {
  processParameters = {
    beadWidth         : spatialFormat.format(getParameter("operation:tool_beadWidth", 0)),
    layerThickness    : spatialFormat.format(getParameter("operation:tool_layerThickness", 0)),
    stepover          : spatialFormat.format(getParameter("operation:tool_stepover", 0)),
    toolSpeed         : spatialFormat.format(getParameter("operation:tool_feedDepositing", 0)),
    depositionLength  : spatialFormat.format(currentSection.getCuttingDistance()),
    gasFlowRate       : spatialFormat.format(getParameter("operation:tool_depositingShieldGasFlowRate", 0)),
    arcCurrent        : spatialFormat.format(getParameter("operation:tool_depositingCurrent", 0)),
    arcVoltage        : spatialFormat.format(getParameter("operation:tool_depositingVoltage", 0)),
    wireSpeed         : spatialFormat.format(getParameter("operation:tool_feedWire", 0)),
    auxGasFlowRate    : spatialFormat.format(getParameter("operation:tool_depositingAuxiliaryGasFlowRate", 0)),
    carrierGasFlowRate: spatialFormat.format(getParameter("operation:tool_depositingCarrierGasFlowRate", 0)),
    powderFlowRate    : spatialFormat.format(getParameter("operation:tool_powderFlowRate", 0)),
    depositionPower   : spatialFormat.format(getParameter("operation:tool_depositingPower", 0))
  };
}

var _unit = unit == MM ? " mm" : " in";
var _unitRate = unit == MM ? " lpm" : " cfm";
function writeDepositionHeader(tool) {
  writeln("");
  writeComment(getToolTypeName(tool.type).toUpperCase() + " DEPOSITION OPERATION");

  writeComment("BEAD WIDTH = " + processParameters.beadWidth + _unit);
  writeComment("LAYER THICKNESS = " + processParameters.layerThickness + _unit);
  writeComment("STEPOVER = " + processParameters.stepover + _unit);
  writeComment("TOOL SPEED = " + processParameters.toolSpeed + _unit + "/min");
  writeComment("DEPOSITED LENGTH = " + processParameters.depositionLength + _unit);
  writeln("");
  writeComment("PROCESS PARAMETERS");
  writeComment("SHIELDING GAS FLOW RATE = " + processParameters.gasFlowRate + _unitRate);

  if (tool.type != TOOL_DEPOSITING_ELECTRIC_ARC_WIRE) {
    writeComment("AUXILIARY GAS FLOW RATE = " + processParameters.auxGasFlowRate + _unitRate);
    writeComment("POWER = " + processParameters.depositionPower + " W");
  } else {
    writeComment("CURRENT = " + processParameters.arcCurrent + " A");
    writeComment("VOLTAGE = " + processParameters.arcVoltage + " V");
  }
  if (tool.type != TOOL_DEPOSITING_LASER_POWDER) {
    writeComment("WIRE SPEED = " + processParameters.wireSpeed + _unit + "/min");
  } else {
    writeComment("CARRIER GAS FLOW RATE = " + processParameters.carrierGasFlowRate + _unitRate);
    writeComment("POWDER FLOW RATE = " + processParameters.powderFlowRate  + _unitRate);
  }
  writeln("");
}

function writeProcessEquipmentCommands(activate) {
  if (!activate) {
    if (hasNextSection() &&
    (isDepositionOperation(getNextSection()) && (tool.type == getNextSection().getTool().type))) {
      return; // do not turn off deposition equipment when the next operation is using the same technology
    }
  }

  writeComment("PROCESS EQUIPMENT" + (activate ? " ON" : " OFF"));
  var _processEquipment = activate ? commands.processEquipment.on : commands.processEquipment.off;
  for (var i in _processEquipment) {
    writeBlock(_processEquipment[i]);
  }
}

/** Output layer information */
function onLayer(index) {
  var layerNumber = index + 1;
  writeComment("LAYER " + layerNumber + " START");
}

function onLayerEnd(index) {
  var layerNumber = index + 1;
  writeComment("LAYER " + layerNumber + " END");
}

var powerOn = false; // track power state
function onMovementDeposition(movement) {
  if (highFeedMapping != HIGH_FEED_NO_MAPPING) {
    error(localize("High feed mapping is not supported."));
    return;
  }
  if (movement == MOVEMENT_LINK_TRANSITION && getProperty("depositOnTransitions")) { // leave linking moves in active state
    return;
  }
  switch (movement) {
  case MOVEMENT_RAPID:
  case MOVEMENT_LEAD_IN:
  case MOVEMENT_LEAD_OUT:
  case MOVEMENT_LINK_TRANSITION:
    if (powerOn) {
      writeBlock(commands.deposition.off, "(STOP DEPOSITION)");
      powerOn = false;
    }
    break;
  case MOVEMENT_DEPOSITING:
    if (!powerOn) {
      writeBlock(commands.deposition.on, "(START DEPOSITION)");
      powerOn = true;
    }
    break;
  }
}
// End of Deposition logic

// ####
// #### The code below is only utilized to generate sample output and can be ignored ####
// ####

/**
  Writes the specified block.
*/
function writeBlock() {
  writeWords(arguments);
}

function formatComment(text) {
  return "(" + String(text).replace(/[()]/g, "") + ")";
}

/**
  Output a comment.
*/
function writeComment(text) {
  writeln(formatComment(text));
}

function onComment(message) {
  writeComment(message);
}

var blockNumber = 0;
function onOpen() {
  //writeComment("##########");
  //writeComment("THIS POSTPROCESSOR IS A TEMPLATE FOR DEPOSITION SUPPORT, IT DOES NOT OUTPUT A FUNCTIONAL NC PROGRAM.");
  //writeComment("##########");
  writeBlock("N" + blockNumber + " G56 X0 Y0 Z0");
  blockNumber = blockNumber + 10;
}

function onRapid(_x, _y, _z) {
  var x = xOutput.format(_x);
  var y = yOutput.format(_y);
  var z = zOutput.format(_z);
  if (x || y || z) {
    writeBlock("N" + blockNumber + " " + gMotionModal.format(0), x, y, z);
	blockNumber = blockNumber + 10;
    feedOutput.reset();
  }
}

function onLinear(_x, _y, _z, feed) {
  var x = xOutput.format(_x);
  var y = yOutput.format(_y);
  var z = zOutput.format(_z);
  var f = feedOutput.format(feed);
  if (x || y || z) {
    writeBlock("N" + blockNumber + " " + gMotionModal.format(1), x, y, z, f);
	blockNumber = blockNumber + 10;
  } else if (f) {
    if (getNextRecord().isMotion()) { // try not to output feed without motion
      feedOutput.reset(); // force feed on next line
    } else {
      writeBlock("N" + blockNumber + " " + gMotionModal.format(1), f);
	  blockNumber = blockNumber + 10;
    }
  }
}

function onRapid5D(_x, _y, _z, _a, _b, _c) {
  var x = xOutput.format(_x);
  var y = yOutput.format(_y);
  var z = zOutput.format(_z);
  var a = aOutput.format(_a);
  var b = bOutput.format(_b);
  var c = cOutput.format(_c);
  if (x || y || z || a || b || c) {
    writeBlock("N" + blockNumber + " " + gMotionModal.format(0), x, y, z, a, b, c);
	blockNumber = blockNumber + 10;
    feedOutput.reset();
  }
}

function onLinear5D(_x, _y, _z, _a, _b, _c, feed, feedMode) {
  var x = xOutput.format(_x);
  var y = yOutput.format(_y);
  var z = zOutput.format(_z);
  var a = aOutput.format(_a);
  var b = bOutput.format(_b);
  var c = cOutput.format(_c);
  var f = feedOutput.format(feed);
  if (x || y || z || a || b || c) {
    writeBlock("N" + blockNumber + " " + gMotionModal.format(1), x, y, z, a, b, c, f);
	blockNumber = blockNumber + 10;
  }
}
