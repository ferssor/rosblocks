import * as Blockly from "blockly/core";
import { PythonGenerator, pythonGenerator } from "blockly/python";

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
    generator: PythonGenerator
  ) {
    const className = block.getFieldValue("CLASS_NAME") || "class_name";
    const classBodyCode = generator.statementToCode(block, "CLASS_BODY") || "";
    const code = `\nclass ${className}:\n${indentCode(classBodyCode)}\n`;
    return code;
  };

  pythonGenerator.forBlock["add_class_inheritance"] = function (
    block: Blockly.Block,
    generator: PythonGenerator
  ) {
    const className = block.getFieldValue("CLASS_NAME") || "class_name";
    const extendsName = block.getFieldValue("EXTENDS_NAME") || "extends_name";
    const classBodyCode = generator.statementToCode(block, "CLASS_BODY") || "";
    const code = `\nclass ${className}(${extendsName}):\n${indentCode(
      classBodyCode
    )}\n`;
    return code;
  };

  pythonGenerator.forBlock["add_import"] = function (block: Blockly.Block) {
    const importName = block.getFieldValue("IMPORT_NAME") || "import_name";
    const code = `import ${importName}\n`;
    return code;
  };

  pythonGenerator.forBlock["add_import_as"] = function (block: Blockly.Block) {
    const importName = block.getFieldValue("IMPORT_NAME") || "import_name";
    const aliasName = block.getFieldValue("ALIAS_NAME") || "alias_name";
    const code = `import ${importName} as ${aliasName}\n`;
    return code;
  };

  pythonGenerator.forBlock["add_from_import"] = function (
    block: Blockly.Block
  ) {
    const packageName = block.getFieldValue("PACKAGE_NAME") || "package_name";
    const methodName = block.getFieldValue("METHOD_NAME") || "method_name";
    const code = `from ${packageName} import ${methodName}\n`;
    return code;
  };

  pythonGenerator.forBlock["add_function"] = function (
    block: Blockly.Block,
    generator: PythonGenerator
  ) {
    const functionName =
      block.getFieldValue("FUNCTION_NAME") || "function_name";
    const functionBodyCode =
      generator.statementToCode(block, "FUNCTION_BODY") || "";
    const code = `def ${functionName}(self):\n${indentCode(
      functionBodyCode
    )}\n`;
    return code;
  };

  pythonGenerator.forBlock["class_init"] = function (
    block: Blockly.Block,
    generator: PythonGenerator
  ) {
    const className = block.getFieldValue("CLASS_NAME") || "class_name";
    const constructorBodyCode =
      generator.statementToCode(block, "CONSTRUCTOR_BODY") || "";
    const code = `def __init__(self):
  super().__init__('${className}')
    \n${indentCode(constructorBodyCode)}\n`;
    return code;
  };

  pythonGenerator.forBlock["node_init"] = function (
    block: Blockly.Block,
    generator: PythonGenerator
  ) {
    const nodeName = block.getFieldValue("NODE_NAME") || "node_name";
    const constructorBodyCode =
      generator.statementToCode(block, "CONSTRUCTOR_BODY") || "";
    const code = `def __init__(self):
  super().__init__('${nodeName}')
    \n${indentCode(constructorBodyCode)}\n`;
    return code;
  };

  pythonGenerator.forBlock["start_node"] = function (block: Blockly.Block) {
    const className = block.getFieldValue("CLASS_NAME") || "class_name";
    const functionName =
      block.getFieldValue("FUNCTION_NAME") || "function_name";
    const code = `def ${functionName}(args=None):
    rclpy.init(args=args)
    node = ${className}()     
    rclpy.spin(node)     
    rclpy.shutdown()\n
`;
    return code;
  };

  pythonGenerator.forBlock["init_statement"] = function (block: Blockly.Block) {
    const functionName =
      block.getFieldValue("FUNCTION_NAME") || "function_name";
    const code = `if __name__ == '__main__':
  ${functionName}()
`;
    return code;
  };

  pythonGenerator.forBlock["add_interface"] = function (block: Blockly.Block) {
    const interfaceName: string = block.getFieldValue("INTERFACE");
    const code = `${interfaceName}\n`;
    return code;
  };

  pythonGenerator.forBlock["add_pub"] = function (block: Blockly.Block) {
    const interfaceName: string = block.getFieldValue("INTERFACE");
    const publisherName: string = block
      .getFieldValue("PUB_NAME")
      .replace(/\s+/g, "_");
    const refreshRate = block.getFieldValue("REFRESH_RATE");

    const code =
      interfaceName && publisherName && refreshRate
        ? `self.${publisherName} = self.create_publisher(${interfaceName
            .split(" ")
            .slice(-1)}, "${publisherName}", ${refreshRate})\n`
        : "";
    return code;
  };

  pythonGenerator.forBlock["add_variable"] = function (block: Blockly.Block) {
    const variableName: string = block.getFieldValue("VARIABLE");
    const value: string = pythonGenerator.valueToCode(block, "VALUE", 0);
    const code = `self.${variableName} = ${value}\n`;
    return code;
  };

  pythonGenerator.forBlock["add_numeric_value"] = function (
    block: Blockly.Block
  ) {
    const value: string = block.getFieldValue("NUMERIC_VALUE");
    const code = `${value}`;
    return [code, 0];
  };

  pythonGenerator.forBlock["add_string_value"] = function (
    block: Blockly.Block
  ) {
    const value: string = block.getFieldValue("STRING_VALUE");
    const code = `"${value}"`;
    return [code, 0];
  };

  pythonGenerator.forBlock["add_boolean_value"] = function (
    block: Blockly.Block
  ) {
    const value: string = block.getFieldValue("BOOL_VALUE");
    const code = `${value}`;
    return [code, 0];
  };

  pythonGenerator.forBlock["add_python_version"] = function (
    block: Blockly.Block
  ) {
    const value: string = block.getFieldValue("PYTHON_VERSION");
    const code = `#!/usr/bin/env python${value}\n`;
    return code;
  };

  pythonGenerator.forBlock["add_logger"] = function (block: Blockly.Block) {
    const value: string = block.getFieldValue("LOG_LEVEL");
    const text: string = block.getFieldValue("LOG_TEXT");
    const code = `self.get_logger().${value}("${text.trim()}")\n`;
    return code;
  };

  pythonGenerator.forBlock["add_message"] = function (block: Blockly.Block) {
    const interfaceName: string = block.getFieldValue("INTERFACE");
    const functionName: string = block
      .getFieldValue("FUNCTION_NAME")
      .replace(/\s+/g, "_");

    const code =
      interfaceName && functionName
        ? `msg = ${interfaceName.split(" ").slice(-1)}()
msg.data = ${functionName}\n`
        : "";
    return code;
  };

  pythonGenerator.forBlock["publish_message"] = function (
    block: Blockly.Block
  ) {
    const publisherName: string = block
      .getFieldValue("PUBLISHER_NAME")
      .replace(/\s+/g, "_");

    const code = publisherName ? `self.${publisherName}.publish(msg)\n` : "";
    return code;
  };

  pythonGenerator.forBlock["subscribe_message"] = function (
    block: Blockly.Block
  ) {
    const variableName: string = block
      .getFieldValue("VARIABLE_NAME")
      .replace(/\s+/g, "_");
    const code = variableName ? `self.${variableName} += msg.data\n` : "";
    return code;
  };

  pythonGenerator.forBlock["add_callback_function"] = function (
    block: Blockly.Block,
    generator: PythonGenerator
  ) {
    const interfaceName: string = block
      .getFieldValue("INTERFACE")
      .split(" ")
      .slice(-1);
    const functionName =
      block.getFieldValue("FUNCTION_NAME") || "function_name";
    const functionBodyCode =
      generator.statementToCode(block, "FUNCTION_BODY") || "";
    const code = `def callback_${functionName}(self, msg:${interfaceName}):\n${indentCode(
      functionBodyCode
    )}\n`;
    return code;
  };

  pythonGenerator.forBlock["create_timer"] = function (block: Blockly.Block) {
    const tempName: string = block.getFieldValue("TEMP_NAME");
    const duration = block.getFieldValue("DURATION");
    const code =
      tempName && duration
        ? `self.${tempName.trim()} = self.create_timer(${duration}, self.publish_number)`
        : "";
    return code;
  };

  pythonGenerator.forBlock["counter_function"] = function (
    block: Blockly.Block
  ) {
    const msgInterface = block.getFieldValue("COUNTER_INTERFACE");
    const duration: number = block.getFieldValue("COUNTER");
    const publisher: string = block.getFieldValue("PUBLISHER");

    const code =
      msgInterface && duration && publisher
        ? `def publish_number(self):
    msg = ${msgInterface.split(" ").slice(-1)}
    msg.data = ${duration}
    self.${publisher.trim()}.publish(msg)
    `
        : "";
    return code;
  };

  pythonGenerator.forBlock["add_information"] = function (
    block: Blockly.Block
  ) {
    const text: string = block.getFieldValue("LOG_INFO");
    const code = text ? `self.get_logger().info("${text.trim()}")` : "";
    return code;
  };

  pythonGenerator.forBlock["add_sub"] = function (block: Blockly.Block) {
    const subscriberName: string = block.getFieldValue("SUB_NAME");
    const interfaceName: string = block.getFieldValue("INTERFACE");
    const publisherName: string = block.getFieldValue("PUB_NAME");
    const refreshRate = block.getFieldValue("REFRESH_RATE");
    const functionName = block.getFieldValue("FUNCTION_NAME");
    const code =
      interfaceName && publisherName && refreshRate
        ? `self.${subscriberName} = self.create_subscription(${interfaceName
            .split(" ")
            .slice(
              -1
            )}, "${publisherName}", self.${functionName}, ${refreshRate})`
        : "";

    return code;
  };
}
