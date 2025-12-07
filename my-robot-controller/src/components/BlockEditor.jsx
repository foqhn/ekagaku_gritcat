import React, { useEffect, useRef } from 'react';
import * as Blockly from 'blockly';
import { pythonGenerator } from 'blockly/python';
import * as Jp from 'blockly/msg/ja';


// Set the language
Blockly.setLocale(Jp);

// Define custom blocks
Blockly.Blocks['robot_move'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("ロボットを動かす");
        this.appendValueInput("LEFT")
            .setCheck("Number")
            .appendField("左モーター (%)");
        this.appendValueInput("RIGHT")
            .setCheck("Number")
            .appendField("右モーター (%)");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(230);
        this.setTooltip("ロボットのモーターを制御します");
        this.setHelpUrl("");
    }
};

pythonGenerator.forBlock['robot_move'] = function (block, generator) {
    var value_left = generator.valueToCode(block, 'LEFT', pythonGenerator.ORDER_ATOMIC) || '0';
    var value_right = generator.valueToCode(block, 'RIGHT', pythonGenerator.ORDER_ATOMIC) || '0';
    var code = `robot.move(${value_left}, ${value_right})\n`;
    return code;
};

Blockly.Blocks['robot_check_sensor'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("センサーを確認")
            .appendField(new Blockly.FieldDropdown([
                ["IMU: 加速度", "imu_accel"],
                ["IMU: 角速度", "imu_gyro"],
                ["IMU: 姿勢角", "imu_orientation"],
                ["地磁気: 方角", "mag_heading"],
                ["温湿度: 温度", "env_temp"],
                ["温湿度: 湿度", "env_humidity"],
                ["温湿度: 気圧", "env_pressure"],
                ["GPS: 緯度", "gps_lat"],
                ["GPS: 経度", "gps_lon"],
                ["GPS: 高度", "gps_alt"],
                ["測距: 距離", "distance"],
                ["WiFi", "wifi"],
                ["バッテリー", "battery"]
            ]), "SENSOR_TYPE");
        this.setOutput(true, null);
        this.setColour(230);
        this.setTooltip("センサーの値を取得します");
        this.setHelpUrl("");
    }
};

pythonGenerator.forBlock['robot_check_sensor'] = function (block, generator) {
    var dropdown_sensor_type = block.getFieldValue('SENSOR_TYPE');
    var code = `robot.get_sensor('${dropdown_sensor_type}')`;
    return [code, pythonGenerator.ORDER_ATOMIC];
};

Blockly.Blocks['robot_display_text'] = {
    init: function () {
        this.appendValueInput("TEXT")
            .setCheck(null)
            .appendField("文字を表示");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(230);
        this.setTooltip("ロボットの画面に文字を表示します");
        this.setHelpUrl("");
    }
};

pythonGenerator.forBlock['robot_display_text'] = function (block, generator) {
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
            name: "ロボット",
            colour: "230",
            contents: [
                { kind: "block", type: "robot_move" },
                { kind: "block", type: "robot_check_sensor" },
                { kind: "block", type: "robot_display_text" },
            ]
        },
        {
            kind: "category",
            name: "論理",
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
            name: "繰り返し",
            colour: "120",
            contents: [
                { kind: "block", type: "controls_repeat_ext" },
                { kind: "block", type: "controls_whileUntil" },
            ]
        },
        {
            kind: "category",
            name: "数学",
            colour: "230",
            contents: [
                { kind: "block", type: "math_number" },
                { kind: "block", type: "math_arithmetic" },
            ]
        },
        {
            kind: "category",
            name: "テキスト",
            colour: "160",
            contents: [
                { kind: "block", type: "text" },
                { kind: "block", type: "text_print" },
            ]
        },
        {
            kind: "category",
            name: "変数",
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
