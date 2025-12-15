import React, { useEffect, useRef } from 'react';
import * as Blockly from 'blockly';
import { pythonGenerator } from 'blockly/python';
import * as Jp from 'blockly/msg/ja';


// Set the language
Blockly.setLocale(Jp);
Blockly.Msg['PROCEDURES_DEFNORETURN_PROCEDURE'] = 'マイ関数';
Blockly.Msg['PROCEDURES_DEFRETURN_PROCEDURE'] = 'マイ関数';

// Define custom blocks
Blockly.Blocks['robot_move'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("モーター制御");
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
            .appendField("センサーチェック")
            .appendField(new Blockly.FieldDropdown([
                ["コンパス: 方位(度)", "compass_heading"],
                //["コンパス: 方位(ラジアン)", "compass_heading_rad"],

                ["IMU: 加速度 X(m/s^2)", "imu_accel_x"],
                ["IMU: 加速度 Y(m/s^2)", "imu_accel_y"],
                ["IMU: 加速度 Z(m/s^2)", "imu_accel_z"],

                ["IMU: 角速度 X(rad/s)", "imu_gyro_x"],
                ["IMU: 角速度 Y(rad/s)", "imu_gyro_y"],
                ["IMU: 角速度 Z(rad/s)", "imu_gyro_z"],

                ["IMU: 姿勢角 X(rad)", "imu_orientation_x"],
                ["IMU: 姿勢角 Y(rad)", "imu_orientation_y"],
                ["IMU: 姿勢角 Z(rad)", "imu_orientation_z"],
                ["IMU: 姿勢角 W(rad)", "imu_orientation_w"],

                ["地磁気: X(mG)", "mag_x"],
                ["地磁気: Y(mG)", "mag_y"],
                ["地磁気: Z(mG)", "mag_z"],

                ["温湿度: 温度(℃)", "env_temp"],
                ["温湿度: 湿度(%)", "env_humidity"],
                ["温湿度: 気圧(hPa)", "env_pressure"],

                ["GPS: 緯度", "gps_lat"],
                ["GPS: 経度", "gps_lon"],
                ["GPS: 高度(m)", "gps_alt"],

                ["WiFi強度(dbm)", "wifi_rssi"],
                //["バッテリー(V)", "battery_voltage"]
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
        case 'compass_heading': code = "robot.get_sensor('compass')['heading']"; break;
        //case 'compass_heading_rad': code = "robot.get_sensor('compass')['heading_rad']"; break;

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


// Image Processing Blocks

Blockly.Blocks['image_capture'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("画像をキャプチャ");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(65);
        this.setTooltip("カメラ画像の現在のフレームを取得して保存します");
    }
};

pythonGenerator.forBlock['image_capture'] = function (block, generator) {
    return 'robot.capture_image()\n';
};

Blockly.Blocks['image_get_size_val'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("画像サイズ取得: ")
            .appendField(new Blockly.FieldDropdown([
                ["幅 (Width)", "0"],
                ["高さ (Height)", "1"]
            ]), "DIMENSION");
        this.setOutput(true, "Number");
        this.setColour(65);
        this.setTooltip("現在の画像の幅または高さを取得します");
    }
};

pythonGenerator.forBlock['image_get_size_val'] = function (block, generator) {
    var dim = block.getFieldValue('DIMENSION');
    return [`robot.get_image_size()[${dim}]`, pythonGenerator.ORDER_ATOMIC];
};

Blockly.Blocks['image_set_roi'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("処理領域(ROI)を設定");
        this.appendValueInput("X")
            .setCheck("Number")
            .appendField("X");
        this.appendValueInput("Y")
            .setCheck("Number")
            .appendField("Y");
        this.appendValueInput("W")
            .setCheck("Number")
            .appendField("幅");
        this.appendValueInput("H")
            .setCheck("Number")
            .appendField("高さ");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(65);
        this.setTooltip("画像処理を行う領域を切り抜きます");
    }
};

pythonGenerator.forBlock['image_set_roi'] = function (block, generator) {
    var x = generator.valueToCode(block, 'X', pythonGenerator.ORDER_ATOMIC) || '0';
    var y = generator.valueToCode(block, 'Y', pythonGenerator.ORDER_ATOMIC) || '0';
    var w = generator.valueToCode(block, 'W', pythonGenerator.ORDER_ATOMIC) || '0';
    var h = generator.valueToCode(block, 'H', pythonGenerator.ORDER_ATOMIC) || '0';
    return `robot.set_roi(${x}, ${y}, ${w}, ${h})\n`;
};

Blockly.Blocks['image_enhance_contrast'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("コントラスト強調 (CLAHE)");
        this.appendValueInput("LIMIT")
            .setCheck("Number")
            .appendField("制限値(Clip Limit)");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(65);
        this.setTooltip("画像のコントラストを強調します");
    }
};

pythonGenerator.forBlock['image_enhance_contrast'] = function (block, generator) {
    var limit = generator.valueToCode(block, 'LIMIT', pythonGenerator.ORDER_ATOMIC) || '2.0';
    return `robot.enhance_contrast(clip_limit=${limit})\n`;
};

Blockly.Blocks['image_morphology'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("モルフォロジー変換")
            .appendField(new Blockly.FieldDropdown([
                ["収縮 (Erode)", "erode"],
                ["膨張 (Dilate)", "dilate"],
                ["オープニング (Open)", "open"],
                ["クロージング (Close)", "close"]
            ]), "OP");
        this.appendValueInput("KERNEL")
            .setCheck("Number")
            .appendField("カーネルサイズ");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(65);
        this.setTooltip("ノイズ除去や領域結合を行います");
    }
};

pythonGenerator.forBlock['image_morphology'] = function (block, generator) {
    var op = block.getFieldValue('OP');
    var kernel = generator.valueToCode(block, 'KERNEL', pythonGenerator.ORDER_ATOMIC) || '5';
    return `robot.apply_morphology(operation='${op}', kernel_size=${kernel})\n`;
};

Blockly.Blocks['image_detect_color'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("色重心検出 (範囲指定)")
            .appendField(new Blockly.FieldDropdown([
                ["HSV", "HSV"],
                ["HSL", "HSL"]
            ]), "COLOR_SPACE");

        this.appendDummyInput().appendField("最小値 (Min)");
        this.appendValueInput("H_MIN").setCheck("Number").appendField("H");
        this.appendValueInput("S_MIN").setCheck("Number").appendField("S");
        this.appendValueInput("V_MIN").setCheck("Number").appendField("V/L");

        this.appendDummyInput().appendField("最大値 (Max)");
        this.appendValueInput("H_MAX").setCheck("Number").appendField("H");
        this.appendValueInput("S_MAX").setCheck("Number").appendField("S");
        this.appendValueInput("V_MAX").setCheck("Number").appendField("V/L");

        this.setOutput(true, null); // Returns dictionary
        this.setColour(65);
        this.setTooltip("指定された色範囲の重心を検出します。結果は辞書形式で返されます。");
    }
};

pythonGenerator.forBlock['image_detect_color'] = function (block, generator) {
    var space = block.getFieldValue('COLOR_SPACE');
    var h_min = generator.valueToCode(block, 'H_MIN', pythonGenerator.ORDER_ATOMIC) || '0';
    var s_min = generator.valueToCode(block, 'S_MIN', pythonGenerator.ORDER_ATOMIC) || '0';
    var v_min = generator.valueToCode(block, 'V_MIN', pythonGenerator.ORDER_ATOMIC) || '0';
    var h_max = generator.valueToCode(block, 'H_MAX', pythonGenerator.ORDER_ATOMIC) || '0';
    var s_max = generator.valueToCode(block, 'S_MAX', pythonGenerator.ORDER_ATOMIC) || '0';
    var v_max = generator.valueToCode(block, 'V_MAX', pythonGenerator.ORDER_ATOMIC) || '0';

    return [`robot.detect_color_centroid('${space}', [${h_min}, ${s_min}, ${v_min}], [${h_max}, ${s_max}, ${v_max}])`, pythonGenerator.ORDER_ATOMIC];
};

Blockly.Blocks['image_get_color_result'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("検出結果から取得")
            .appendField(new Blockly.FieldDropdown([
                ["重心 X", "x"],
                ["重心 Y", "y"],
                ["面積 (Area)", "area"],
                ["検出有無 (Exists)", "exists"]
            ]), "PARAM");
        this.appendValueInput("RESULT")
            .setCheck(null)
            .appendField("結果データ");
        this.setOutput(true, null);
        this.setColour(65);
        this.setTooltip("色検出結果の辞書から特定の値を取り出します");
    }
};

pythonGenerator.forBlock['image_get_color_result'] = function (block, generator) {
    var param = block.getFieldValue('PARAM');
    var result = generator.valueToCode(block, 'RESULT', pythonGenerator.ORDER_ATOMIC) || '{}';
    return [`${result}['${param}']`, pythonGenerator.ORDER_ATOMIC];
};

Blockly.Blocks['image_draw_marker'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("マーカーを描画")
            .appendField("色")
            .appendField(new Blockly.FieldDropdown([
                ["赤", "red"],
                ["緑", "green"],
                ["青", "blue"],
                ["黄", "yellow"],
                ["シアン", "cyan"],
                ["マゼンタ", "magenta"],
                ["白", "white"],
                ["黒", "black"]
            ]), "COLOR");
        this.appendValueInput("X").setCheck("Number").appendField("X");
        this.appendValueInput("Y").setCheck("Number").appendField("Y");
        this.appendValueInput("SIZE").setCheck("Number").appendField("サイズ");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(65);
        this.setTooltip("指定位置にマーカーを描画します");
    }
};

pythonGenerator.forBlock['image_draw_marker'] = function (block, generator) {
    var color = block.getFieldValue('COLOR');
    var x = generator.valueToCode(block, 'X', pythonGenerator.ORDER_ATOMIC) || '0';
    var y = generator.valueToCode(block, 'Y', pythonGenerator.ORDER_ATOMIC) || '0';
    var size = generator.valueToCode(block, 'SIZE', pythonGenerator.ORDER_ATOMIC) || '15';

    // OpenCV uses BGR
    var bgr = '(0, 255, 0)';
    switch (color) {
        case 'red': bgr = '(0, 0, 255)'; break;
        case 'green': bgr = '(0, 255, 0)'; break;
        case 'blue': bgr = '(255, 0, 0)'; break;
        case 'yellow': bgr = '(0, 255, 255)'; break;
        case 'cyan': bgr = '(255, 255, 0)'; break;
        case 'magenta': bgr = '(255, 0, 255)'; break;
        case 'white': bgr = '(255, 255, 255)'; break;
        case 'black': bgr = '(0, 0, 0)'; break;
    }

    return `robot.draw_marker(${x}, ${y}, color=${bgr}, size=${size})\n`;
};

Blockly.Blocks['image_draw_rect'] = {
    init: function () {
        this.appendDummyInput()
            .appendField("矩形を描画")
            .appendField("色")
            .appendField(new Blockly.FieldDropdown([
                ["赤", "red"],
                ["緑", "green"],
                ["青", "blue"],
                ["黄", "yellow"],
                ["シアン", "cyan"],
                ["マゼンタ", "magenta"],
                ["白", "white"],
                ["黒", "black"]
            ]), "COLOR");
        this.appendValueInput("X").setCheck("Number").appendField("X");
        this.appendValueInput("Y").setCheck("Number").appendField("Y");
        this.appendValueInput("W").setCheck("Number").appendField("幅");
        this.appendValueInput("H").setCheck("Number").appendField("高さ");
        this.appendValueInput("THICKNESS").setCheck("Number").appendField("線の太さ");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(65);
        this.setTooltip("指定領域に矩形（四角形）を描画します");
    }
};

pythonGenerator.forBlock['image_draw_rect'] = function (block, generator) {
    var color = block.getFieldValue('COLOR');
    var x = generator.valueToCode(block, 'X', pythonGenerator.ORDER_ATOMIC) || '0';
    var y = generator.valueToCode(block, 'Y', pythonGenerator.ORDER_ATOMIC) || '0';
    var w = generator.valueToCode(block, 'W', pythonGenerator.ORDER_ATOMIC) || '0';
    var h = generator.valueToCode(block, 'H', pythonGenerator.ORDER_ATOMIC) || '0';
    var t = generator.valueToCode(block, 'THICKNESS', pythonGenerator.ORDER_ATOMIC) || '2';

    // OpenCV uses BGR
    var bgr = '(0, 255, 0)';
    switch (color) {
        case 'red': bgr = '(0, 0, 255)'; break;
        case 'green': bgr = '(0, 255, 0)'; break;
        case 'blue': bgr = '(255, 0, 0)'; break;
        case 'yellow': bgr = '(0, 255, 255)'; break;
        case 'cyan': bgr = '(255, 255, 0)'; break;
        case 'magenta': bgr = '(255, 0, 255)'; break;
        case 'white': bgr = '(255, 255, 255)'; break;
        case 'black': bgr = '(0, 0, 0)'; break;
    }

    return `robot.draw_rect(${x}, ${y}, ${w}, ${h}, color=${bgr}, thickness=${t})\n`;
};

Blockly.Blocks['image_show'] = {
    init: function () {
        this.appendDummyInput().appendField("処理画像を表示");
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(65);
        this.setTooltip("現在の処理画像をWeb画面に送信して表示します");
    }
};

pythonGenerator.forBlock['image_show'] = function (block, generator) {
    return 'robot.show_image()\n';
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
            name: "画像処理",
            colour: "65",
            contents: [
                { kind: "block", type: "image_capture" },
                { kind: "block", type: "image_get_size_val" },
                { kind: "block", type: "image_set_roi" },
                { kind: "block", type: "image_enhance_contrast" },
                { kind: "block", type: "image_morphology" },
                { kind: "block", type: "image_detect_color" },
                { kind: "block", type: "image_get_color_result" },
                { kind: "block", type: "image_draw_marker" },
                { kind: "block", type: "image_draw_rect" },
                { kind: "block", type: "image_show" },
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
        {
            kind: "category",
            name: "関数",
            custom: "PROCEDURE",
            contents: [
                { kind: "block", type: "procedures_defnoreturn" },
                { kind: "block", type: "procedures_defreturn" },
                { kind: "block", type: "procedures_callnoreturn" },
                { kind: "block", type: "procedures_callreturn" },
            ],
            //default function name is "myFunction"
            //default function parameters are "arg1", "arg2", "arg3"
            //default function body is "pass"
            //default function return value is "None"
            //default function return type is "None"

            colour: "290"
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
