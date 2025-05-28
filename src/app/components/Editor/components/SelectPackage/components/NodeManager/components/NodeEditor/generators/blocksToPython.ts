import * as Blockly from "blockly/core";
import { PythonGenerator, pythonGenerator } from "blockly/python";

export function registerCustomBlocksToPython() {
  pythonGenerator.forBlock["add_class"] = function (
    block: Blockly.Block,
    generator: PythonGenerator
  ) {
    const className = block.getFieldValue("CLASS_NAME") || "class_name";
    const classBodyCode = generator.statementToCode(block, "CLASS_BODY") || "";
    const code = `\nclass ${className}:
    ${classBodyCode.trim()}\n
`;
    return code;
  };

  pythonGenerator.forBlock["add_class_inheritance"] = function (
    block: Blockly.Block,
    generator: PythonGenerator
  ) {
    const className = block.getFieldValue("CLASS_NAME") || "class_name";
    const extendsName = block.getFieldValue("EXTENDS_NAME") || "extends_name";
    const classBodyCode = generator.statementToCode(block, "CLASS_BODY") || "";
    const code = `\nclass ${className}(${extendsName}):
    ${classBodyCode.trim()}\n
`;
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

  pythonGenerator.forBlock["add_pub"] = function (block: Blockly.Block) {
    const interfaceName: string = block.getFieldValue("INTERFACE");
    const publisherName: string = block
      .getFieldValue("PUB_NAME")
      .replace(/\s+/g, "_");
    const refreshRate = block.getFieldValue("REFRESH_RATE");

    const code =
      interfaceName && publisherName && refreshRate
        ? `self._${publisherName.concat(
            "_"
          )} = self.create_publisher(${interfaceName
            .split(" ")
            .slice(-1)}, "${publisherName}", ${refreshRate})`
        : "";
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
    const interfaceName: string = block.getFieldValue("INTERFACE");
    const publisherName: string = block.getFieldValue("PUB_NAME");
    const refreshRate = block.getFieldValue("REFRESH_RATE");
    const code =
      interfaceName && publisherName && refreshRate
        ? `self.number_subscriber_ = self.create_subscription(${interfaceName
            .split(" ")
            .slice(
              -1
            )}, "${publisherName}", self.callback_number, ${refreshRate})`
        : "";

    return code;
  };
}
