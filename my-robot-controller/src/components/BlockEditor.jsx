import React, { useEffect, useRef } from 'react';
import * as Blockly from 'blockly/core';
import 'blockly/blocks';
import 'blockly/python';
import * as En from 'blockly/msg/en';
import { pythonGenerator } from 'blockly/python';

// Set the language
Blockly.setLocale(En);

// Define custom blocks
Blockly.Blocks['robot_move'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Move Robot");
    this.appendValueInput("LEFT")
        .setCheck("Number")
        .appendField("Left Speed (%)");
    this.appendValueInput("RIGHT")
        .setCheck("Number")
        .appendField("Right Speed (%)");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("Move the robot motors");
    this.setHelpUrl("");
  }
};

pythonGenerator.forBlock['robot_move'] = function(block, generator) {
  var value_left = generator.valueToCode(block, 'LEFT', pythonGenerator.ORDER_ATOMIC) || '0';
  var value_right = generator.valueToCode(block, 'RIGHT', pythonGenerator.ORDER_ATOMIC) || '0';
  var code = `robot.move(${value_left}, ${value_right})\n`;
  return code;
};

Blockly.Blocks['robot_check_sensor'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Check Sensor")
        .appendField(new Blockly.FieldDropdown([["IMU","imu"], ["GPS","gps"], ["WiFi","wifi"], ["Battery","battery"]]), "SENSOR_TYPE");
    this.setOutput(true, null);
    this.setColour(230);
    this.setTooltip("Get sensor data");
    this.setHelpUrl("");
  }
};

pythonGenerator.forBlock['robot_check_sensor'] = function(block, generator) {
  var dropdown_sensor_type = block.getFieldValue('SENSOR_TYPE');
  var code = `robot.get_sensor('${dropdown_sensor_type}')`;
  return [code, pythonGenerator.ORDER_ATOMIC];
};

Blockly.Blocks['robot_display_text'] = {
  init: function() {
    this.appendValueInput("TEXT")
        .setCheck(null)
        .appendField("Display Text");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip("Display text on the robot screen");
    this.setHelpUrl("");
  }
};

pythonGenerator.forBlock['robot_display_text'] = function(block, generator) {
  var value_text = generator.valueToCode(block, 'TEXT', pythonGenerator.ORDER_ATOMIC) || "''";
  var code = `robot.display(${value_text})\n`;
  return code;
};

// Toolbox definition
const toolbox = {
  kind: "categoryToolbox",
  contents: [
    {
      kind: "category",
      name: "Robot",
      colour: "230",
      contents: [
        { kind: "block", type: "robot_move" },
        { kind: "block", type: "robot_check_sensor" },
        { kind: "block", type: "robot_display_text" },
      ]
    },
    {
      kind: "category",
      name: "Logic",
      colour: "210",
      contents: [
        { kind: "block", type: "controls_if" },
        { kind: "block", type: "logic_compare" },
        { kind: "block", type: "logic_operation" },
        { kind: "block", type: "logic_boolean" },
      ]
    },
    {
      kind: "category",
      name: "Loops",
      colour: "120",
      contents: [
        { kind: "block", type: "controls_repeat_ext" },
        { kind: "block", type: "controls_whileUntil" },
      ]
    },
    {
      kind: "category",
      name: "Math",
      colour: "230",
      contents: [
        { kind: "block", type: "math_number" },
        { kind: "block", type: "math_arithmetic" },
      ]
    },
    {
      kind: "category",
      name: "Text",
      colour: "160",
      contents: [
        { kind: "block", type: "text" },
        { kind: "block", type: "text_print" },
      ]
    },
    {
      kind: "category",
      name: "Variables",
      custom: "VARIABLE",
      colour: "330"
    },
  ]
};

const BlockEditor = ({ onCodeChange }) => {
  const blocklyDiv = useRef(null);
  const workspaceRef = useRef(null);

  useEffect(() => {
    if (!blocklyDiv.current) return;

    // Inject Blockly
    workspaceRef.current = Blockly.inject(blocklyDiv.current, {
      toolbox: toolbox,
      scrollbars: true,
      trashcan: true,
    });

    // Add change listener
    const onWorkspaceChange = () => {
      const code = pythonGenerator.workspaceToCode(workspaceRef.current);
      if (onCodeChange) {
        onCodeChange(code);
      }
    };
    workspaceRef.current.addChangeListener(onWorkspaceChange);

    // Initial resize
    Blockly.svgResize(workspaceRef.current);

    return () => {
      if (workspaceRef.current) {
        workspaceRef.current.dispose();
      }
    };
  }, [onCodeChange]);

  return (
    <div style={{ width: '100%', height: '100%' }}>
      <div ref={blocklyDiv} style={{ width: '100%', height: '100%' }} />
    </div>
  );
};

export default BlockEditor;
