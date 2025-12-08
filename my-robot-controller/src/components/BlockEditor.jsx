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
                ["IMU: 加速度 X", "imu_accel_x"],
                ["IMU: 加速度 Y", "imu_accel_y"],
                ["IMU: 加速度 Z", "imu_accel_z"],

                ["IMU: 角速度 X", "imu_gyro_x"],
                ["IMU: 角速度 Y", "imu_gyro_y"],
                ["IMU: 角速度 Z", "imu_gyro_z"],

                ["IMU: 姿勢角 X", "imu_orientation_x"],
                ["IMU: 姿勢角 Y", "imu_orientation_y"],
                ["IMU: 姿勢角 Z", "imu_orientation_z"],
                ["IMU: 姿勢角 W", "imu_orientation_w"],

                ["地磁気: X", "mag_x"],
                ["地磁気: Y", "mag_y"],
                ["地磁気: Z", "mag_z"],

                ["温湿度: 温度", "env_temp"],
                ["温湿度: 湿度", "env_humidity"],
                ["温湿度: 気圧", "env_pressure"],

                ["GPS: 緯度", "gps_lat"],
                ["GPS: 経度", "gps_lon"],
                ["GPS: 高度", "gps_alt"],

                ["WiFi強度", "wifi_rssi"],
                ["バッテリー", "battery_voltage"]
            ]), "SENSOR_TYPE");
        this.setOutput(true, "Number");
        this.setColour(230);
        this.setTooltip("センサーの特定の値（数値）を取得します");
        this.setHelpUrl("");
    }
};

pythonGenerator.forBlock['robot_check_sensor'] = function (block, generator) {
    var dropdown_sensor_type = block.getFieldValue('SENSOR_TYPE');
    var code = '0';

    switch (dropdown_sensor_type) {
        case 'imu_accel_x': code = "robot.get_sensor('imu')['linear_acceleration']['x']"; break;
        case 'imu_accel_y': code = "robot.get_sensor('imu')['linear_acceleration']['y']"; break;
        case 'imu_accel_z': code = "robot.get_sensor('imu')['linear_acceleration']['z']"; break;

        case 'imu_gyro_x': code = "robot.get_sensor('imu')['angular_velocity']['x']"; break;
        case 'imu_gyro_y': code = "robot.get_sensor('imu')['angular_velocity']['y']"; break;
        case 'imu_gyro_z': code = "robot.get_sensor('imu')['angular_velocity']['z']"; break;

        case 'mag_x': code = "robot.get_sensor('mag')['magnetic_field']['x']"; break;
        case 'mag_y': code = "robot.get_sensor('mag')['magnetic_field']['y']"; break;
        case 'mag_z': code = "robot.get_sensor('mag')['magnetic_field']['z']"; break;

        case 'imu_orientation_x': code = "robot.get_sensor('imu')['orientation']['x']"; break;
        case 'imu_orientation_y': code = "robot.get_sensor('imu')['orientation']['y']"; break;
        case 'imu_orientation_z': code = "robot.get_sensor('imu')['orientation']['z']"; break;
        case 'imu_orientation_w': code = "robot.get_sensor('imu')['orientation']['w']"; break;

        case 'env_temp': code = "robot.get_sensor('bme280')['temperature_celsius']"; break;
        case 'env_humidity': code = "robot.get_sensor('bme280')['humidity_percent']"; break;
        case 'env_pressure': code = "robot.get_sensor('bme280')['pressure_hpa']"; break;

        case 'gps_lat': code = "robot.get_sensor('gps')['latitude']"; break;
        case 'gps_lon': code = "robot.get_sensor('gps')['longitude']"; break;
        case 'gps_alt': code = "robot.get_sensor('gps')['altitude']"; break;

        case 'wifi_rssi': code = "robot.get_sensor('wifi')['rssi']"; break;
        case 'battery_voltage': code = "robot.get_sensor('battery')['voltage']"; break;
        default: code = "0";
    }

    return [code, pythonGenerator.ORDER_ATOMIC];
};

Blockly.Blocks['robot_display_text'] = {
    init: function () {
        this.appendValueInput("TEXT")
            .setCheck(null)
            .appendField("文字や変数を表示");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(230);
        this.setTooltip("ロボットの画面に文字や変数を表示します");
        this.setHelpUrl("");
    }
};

pythonGenerator.forBlock['robot_display_text'] = function (block, generator) {
    var value = generator.valueToCode(block, 'TEXT', pythonGenerator.ORDER_ATOMIC) || "''";
    var code = `robot.print_display(${value})\n`;
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

const BlockEditor = ({ onCodeChange, initialState, onStateChange }) => {
    const blocklyDiv = useRef(null);
    const workspaceRef = useRef(null);

    // Use refs for callbacks to ensure current versions are used in event listeners
    // without triggering re-effects
    const onCodeChangeRef = useRef(onCodeChange);
    const onStateChangeRef = useRef(onStateChange);

    useEffect(() => {
        onCodeChangeRef.current = onCodeChange;
        onStateChangeRef.current = onStateChange;
    }, [onCodeChange, onStateChange]);

    useEffect(() => {
        if (!blocklyDiv.current) return;

        // Prevent double injection in Strict Mode
        if (workspaceRef.current) return;

        // Inject Blockly
        workspaceRef.current = Blockly.inject(blocklyDiv.current, {
            toolbox: toolbox,
            scrollbars: true,
            trashcan: true,
        });

        // Load initial state if available
        // Only load once on mount
        if (initialState) {
            Blockly.serialization.workspaces.load(initialState, workspaceRef.current);
        }

        // Add change listener
        const onWorkspaceChange = () => {
            const code = pythonGenerator.workspaceToCode(workspaceRef.current);
            if (onCodeChangeRef.current) {
                onCodeChangeRef.current(code);
            }
            if (onStateChangeRef.current) {
                const state = Blockly.serialization.workspaces.save(workspaceRef.current);
                onStateChangeRef.current(state);
            }
        };
        workspaceRef.current.addChangeListener(onWorkspaceChange);

        // Initial resize
        Blockly.svgResize(workspaceRef.current);

        return () => {
            if (workspaceRef.current) {
                workspaceRef.current.dispose();
                workspaceRef.current = null;
            }
        };
    }, []); // Empty dependency array to ensure run only once

    return (
        <div style={{ width: '100%', height: '100%' }}>
            <div ref={blocklyDiv} style={{ width: '100%', height: '100%' }} />
        </div>
    );
};

export default BlockEditor;
