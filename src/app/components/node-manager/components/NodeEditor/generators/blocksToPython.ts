import { pythonGenerator } from "blockly/python";

import { getTemplate } from "./templateService";

import type * as Blockly from "blockly/core";
import type { PythonGenerator } from "blockly/python";

function indentCode(code: string, indent: string = "  "): string {
  const isAlreadyIndented = code
    .split("\n")
    .some((line) => line.startsWith("  "));

  if (isAlreadyIndented) {
    return code;
  }

  return code
    .split("\n")
    .map((line) => (line.trim() ? indent + line : ""))
    .join("\n");
}

export function registerCustomBlocksToPython() {
  pythonGenerator.forBlock["add_class"] = function (
    block: Blockly.Block,
    generator: PythonGenerator,
  ) {
    const className = block.getFieldValue("CLASS_NAME") || "class_name";
    const classBodyCode = generator.statementToCode(block, "CLASS_BODY") || "";
    const template = getTemplate("add_class");
    return template({ className, classBodyCode: indentCode(classBodyCode) });
  };

  pythonGenerator.forBlock["add_class_inheritance"] = function (
    block: Blockly.Block,
    generator: PythonGenerator,
  ) {
    const className = block.getFieldValue("CLASS_NAME") || "class_name";
    const extendsName = block.getFieldValue("EXTENDS_NAME") || "extends_name";
    const classBodyCode = generator.statementToCode(block, "CLASS_BODY") || "";
    const template = getTemplate("add_class_inheritance");
    return template({
      className,
      extendsName,
      classBodyCode: indentCode(classBodyCode),
    });
  };

  pythonGenerator.forBlock["add_import"] = function (block: Blockly.Block) {
    const importName = block.getFieldValue("IMPORT_NAME") || "import_name";
    const template = getTemplate("add_import");
    return template({ importName });
  };

  pythonGenerator.forBlock["add_import_as"] = function (block: Blockly.Block) {
    const importName = block.getFieldValue("IMPORT_NAME") || "import_name";
    const aliasName = block.getFieldValue("ALIAS_NAME") || "alias_name";
    const template = getTemplate("add_import_as");
    return template({ importName, aliasName });
  };

  pythonGenerator.forBlock["add_from_import"] = function (
    block: Blockly.Block,
  ) {
    const packageName = block.getFieldValue("PACKAGE_NAME") || "package_name";
    const methodName = block.getFieldValue("METHOD_NAME") || "method_name";
    const template = getTemplate("add_from_import");
    return template({ packageName, methodName });
  };

  pythonGenerator.forBlock["add_function"] = function (
    block: Blockly.Block,
    generator: PythonGenerator,
  ) {
    const functionName =
      block.getFieldValue("FUNCTION_NAME") || "function_name";
    const functionBodyCode =
      generator.statementToCode(block, "FUNCTION_BODY") || "";
    const template = getTemplate("add_function");
    return template({
      functionName,
      functionBodyCode: indentCode(functionBodyCode),
    });
  };

  pythonGenerator.forBlock["class_init"] = function (
    block: Blockly.Block,
    generator: PythonGenerator,
  ) {
    const className = block.getFieldValue("CLASS_NAME") || "class_name";
    const constructorBodyCode =
      generator.statementToCode(block, "CONSTRUCTOR_BODY") || "";
    const template = getTemplate("class_init");
    return template({
      className,
      constructorBodyCode: indentCode(constructorBodyCode),
    });
  };

  pythonGenerator.forBlock["node_init"] = function (
    block: Blockly.Block,
    generator: PythonGenerator,
  ) {
    const nodeName = block.getFieldValue("NODE_NAME") || "node_name";
    const constructorBodyCode =
      generator.statementToCode(block, "CONSTRUCTOR_BODY") || "";
    const template = getTemplate("node_init");
    return template({
      nodeName,
      constructorBodyCode: indentCode(constructorBodyCode),
    });
  };

  pythonGenerator.forBlock["start_node"] = function (block: Blockly.Block) {
    const className = block.getFieldValue("CLASS_NAME") || "class_name";
    const functionName =
      block.getFieldValue("FUNCTION_NAME") || "function_name";
    const template = getTemplate("start_node");
    return template({ className, functionName });
  };

  pythonGenerator.forBlock["init_statement"] = function (block: Blockly.Block) {
    const functionName =
      block.getFieldValue("FUNCTION_NAME") || "function_name";
    const template = getTemplate("init_statement");
    return template({ functionName });
  };

  pythonGenerator.forBlock["add_interface"] = function (block: Blockly.Block) {
    const interfaceName: string = block.getFieldValue("INTERFACE");
    const template = getTemplate("add_interface");
    return template({ interfaceName });
  };

  pythonGenerator.forBlock["add_pub"] = function (block: Blockly.Block) {
    const interfaceName: string = block.getFieldValue("INTERFACE");
    const topicName: string = block.getFieldValue("TOPIC_NAME");
    const publisherName: string = block
      .getFieldValue("PUB_NAME")
      .replace(/\s+/g, "_");
    const refreshRate = block.getFieldValue("REFRESH_RATE");
    const interfaceClass = interfaceName.split(" ").slice(-1);

    const template = getTemplate("add_pub");
    return template({
      interfaceName,
      publisherName,
      refreshRate,
      topicName,
      interfaceClass,
    });
  };

  pythonGenerator.forBlock["ros_communication"] = function (
    block: Blockly.Block,
  ) {
    const commType: string = block.getFieldValue("COMM_TYPE");
    const interfaceName: string = block.getFieldValue("INTERFACE");
    const topicName: string = block.getFieldValue("TARGET_NAME");
    const commName: string = (block.getFieldValue("COMM_NAME") || "comm")
      .replace(/\s+/g, "_")
      .trim();
    const interfaceClass = interfaceName.split(" ").slice(-1);
    const callbackName: string = (
      block.getFieldValue("CALLBACK_NAME") || `${commName}_callback`
    )
      .replace(/\s+/g, "_")
      .trim();

    const template = getTemplate("ros_communication");
    return template({
      commType,
      interfaceName,
      topicName,
      commName,
      interfaceClass,
      callbackName,
    });
  };

  pythonGenerator.forBlock["add_variable"] = function (block: Blockly.Block) {
    const variableName: string = block.getFieldValue("VARIABLE");
    const value: string = pythonGenerator.valueToCode(block, "VALUE", 0);
    const template = getTemplate("add_variable");
    return template({ variableName, value });
  };

  pythonGenerator.forBlock["add_numeric_value"] = function (
    block: Blockly.Block,
  ) {
    const value: string = block.getFieldValue("NUMERIC_VALUE");
    const template = getTemplate("add_numeric_value");
    return [template({ value }), 0];
  };

  pythonGenerator.forBlock["add_string_value"] = function (
    block: Blockly.Block,
  ) {
    const value: string = block.getFieldValue("STRING_VALUE");
    const template = getTemplate("add_string_value");
    return [template({ value }), 0];
  };

  pythonGenerator.forBlock["add_boolean_value"] = function (
    block: Blockly.Block,
  ) {
    const value: string = block.getFieldValue("BOOL_VALUE");
    const template = getTemplate("add_boolean_value");
    return [template({ value }), 0];
  };

  pythonGenerator.forBlock["add_literal_value"] = function (
    block: Blockly.Block,
  ) {
    const value: string = block.getFieldValue("LITERAL_VALUE");
    const template = getTemplate("add_literal_value");
    return [template({ value }), 0];
  };

  pythonGenerator.forBlock["add_literal"] = function (block: Blockly.Block) {
    const value: string = block.getFieldValue("LITERAL");
    const template = getTemplate("add_literal");
    return [template({ value }), 0];
  };

  pythonGenerator.forBlock["execute_literal"] = function (
    block: Blockly.Block,
  ) {
    const code = pythonGenerator.valueToCode(block, "CODE", 0);
    const template = getTemplate("execute_literal");
    return template({ code });
  };

  pythonGenerator.forBlock["execute_function_timer"] = function (
    block: Blockly.Block,
  ) {
    const functionName: string = block.getFieldValue("FUNCTION_NAME");
    const interval: number = block.getFieldValue("INTERVAL");
    const template = getTemplate("execute_function_timer");
    return template({ functionName, interval: interval.toFixed(1) });
  };

  pythonGenerator.forBlock["add_python_version"] = function (
    block: Blockly.Block,
  ) {
    const value: string = block.getFieldValue("PYTHON_VERSION");
    const template = getTemplate("add_python_version");
    return template({ value });
  };

  pythonGenerator.forBlock["add_logger"] = function (block: Blockly.Block) {
    const value: string = block.getFieldValue("LOG_LEVEL");
    const text: string = pythonGenerator.valueToCode(block, "LOG_TEXT", 0);
    const template = getTemplate("add_logger");
    return template({ value, text });
  };

  pythonGenerator.forBlock["init_message_interface"] = function (
    block: Blockly.Block,
  ) {
    const interfaceName: string = block.getFieldValue("INTERFACE");
    const messageVariable: string = (
      block.getFieldValue("MESSAGE_VARIABLE") || "msg"
    )
      .replace(/\s+/g, "_")
      .trim();
    const interfaceClass = interfaceName.split(" ").slice(-1);

    const template = getTemplate("init_message_interface");
    return template({ interfaceName, messageVariable, interfaceClass });
  };

  pythonGenerator.forBlock["add_message"] = function (block: Blockly.Block) {
    const property: string = block.getFieldValue("PROPERTY");
    const messageVariable: string = (
      block.getFieldValue("MESSAGE_VARIABLE") || "msg"
    )
      .replace(/\s+/g, "_")
      .trim();
    const value = pythonGenerator.valueToCode(block, "VALUE", 0);

    const template = getTemplate("add_message");
    return template({ property, messageVariable, value });
  };

  pythonGenerator.forBlock["publish_message"] = function (
    block: Blockly.Block,
  ) {
    const publisherName: string = (block.getFieldValue("PUBLISHER_NAME") || "")
      .replace(/\s+/g, "_")
      .trim();
    const messageVariable: string = (
      block.getFieldValue("MESSAGE_VARIABLE") || "msg"
    )
      .replace(/\s+/g, "_")
      .trim();

    const template = getTemplate("publish_message");
    return template({ publisherName, messageVariable });
  };

  pythonGenerator.forBlock["launch_executable"] = function (
    block: Blockly.Block,
  ) {
    const command: string = (
      block.getFieldValue("EXECUTABLE_OPTION") || ""
    ).trim();
    const sanitizedCommand = JSON.stringify(command);
    const helperName = pythonGenerator.provideFunction_(
      "launch_executable_helper",
      [
        "import subprocess",
        "",
        "def " + pythonGenerator.FUNCTION_NAME_PLACEHOLDER_ + "(command: str):",
        '  subprocess.Popen(["/bin/bash", "-c", command])',
      ],
    );

    const template = getTemplate("launch_executable");
    return template({ command, sanitizedCommand, helperName });
  };

  pythonGenerator.forBlock["subscribe_message"] = function (
    block: Blockly.Block,
  ) {
    const interfaceName: string = block.getFieldValue("MESSAGE_INTERFACE");
    const property: string = block.getFieldValue("PROPERTY");
    const variableName: string = block
      .getFieldValue("VARIABLE_NAME")
      .replace(/\s+/g, "_");
    const interfaceClass = interfaceName.split(" ").slice(-1);

    const template = getTemplate("subscribe_message");
    return template({ interfaceName, property, variableName, interfaceClass });
  };

  pythonGenerator.forBlock["add_callback_function"] = function (
    block: Blockly.Block,
    generator: PythonGenerator,
  ) {
    const interfaceName: string = block
      .getFieldValue("INTERFACE")
      .split(" ")
      .slice(-1)[0];
    const functionName =
      block.getFieldValue("FUNCTION_NAME") || "function_name";
    const functionBodyCode =
      generator.statementToCode(block, "FUNCTION_BODY") || "";

    const template = getTemplate("add_callback_function");
    return template({
      interfaceName,
      functionName,
      functionBodyCode: indentCode(functionBodyCode),
    });
  };

  pythonGenerator.forBlock["create_timer"] = function (block: Blockly.Block) {
    const tempName: string = block.getFieldValue("TEMP_NAME");
    const duration = block.getFieldValue("DURATION");
    const rawValue =
      pythonGenerator.valueToCode(block, "PUBLISHER_COUNTER", 0) || "";
    const value = rawValue.replace(/^self\./, "").trim();

    const template = getTemplate("create_timer");
    return template({ tempName, duration, value });
  };

  pythonGenerator.forBlock["counter_function"] = function (
    block: Blockly.Block,
  ) {
    const msgInterface = block.getFieldValue("COUNTER_INTERFACE");
    const duration: number = block.getFieldValue("COUNTER");
    const publisher: string = block.getFieldValue("PUBLISHER");
    const interfaceClass = msgInterface.split(" ").slice(-1);

    const template = getTemplate("counter_function");
    return template({ msgInterface, duration, publisher, interfaceClass });
  };

  pythonGenerator.forBlock["add_information"] = function (
    block: Blockly.Block,
  ) {
    const text: string = block.getFieldValue("LOG_INFO");
    const template = getTemplate("add_information");
    return template({ text: text.trim() });
  };

  pythonGenerator.forBlock["add_sub"] = function (block: Blockly.Block) {
    const subscriberName: string = block.getFieldValue("SUB_NAME");
    const interfaceName: string = block.getFieldValue("INTERFACE");
    const topicName: string = block.getFieldValue("TOPIC_NAME");
    const refreshRate = block.getFieldValue("REFRESH_RATE");
    const functionName = block.getFieldValue("FUNCTION_NAME");
    const interfaceClass = interfaceName.split(" ").slice(-1);

    const template = getTemplate("add_sub");
    return template({
      subscriberName,
      interfaceName,
      topicName,
      refreshRate,
      functionName,
      interfaceClass,
    });
  };
}

pythonGenerator.forBlock["add_velocity"] = function (block: Blockly.Block) {
  const propertyName: string = block.getFieldValue("PROPERTY");
  const velocityValue: number = block.getFieldValue("VELOCITY_VALUE");
  const property = propertyName.split(" ").slice(-1);

  const template = getTemplate("add_velocity");
  return template({
    propertyName,
    velocityValue: velocityValue.toFixed(1),
    property,
  });
};

pythonGenerator.forBlock["ros_node_template"] = function (
  block: Blockly.Block,
  generator: PythonGenerator,
) {
  const className: string = block.getFieldValue("CLASS_NAME") || "RosNode";
  const interfaceImport = block.getFieldValue("INTERFACE") || "";
  const nodeBody = generator.statementToCode(block, "NODE_BODY") || "";
  const normalizedBody = nodeBody.trim().length > 0 ? nodeBody : "pass\n";

  const template = getTemplate("ros_node_template");
  return template({
    className,
    interfaceImport,
    normalizedBody: indentCode(normalizedBody),
  });
};

pythonGenerator.forBlock["add_casting"] = function (block: Blockly.Block) {
  const value: string = pythonGenerator.valueToCode(block, "VALUE", 0);
  const dataType: string = block.getFieldValue("DATA_TYPE");

  const template = getTemplate("add_casting");
  return [template({ value, dataType }), 0];
};
