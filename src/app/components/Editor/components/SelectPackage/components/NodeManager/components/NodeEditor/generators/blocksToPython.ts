import * as Blockly from "blockly/core";
import { PythonGenerator, pythonGenerator } from "blockly/python";

export function registerCustomBlocksToPython() {
  pythonGenerator.forBlock["add_class"] = function (
    block: Blockly.Block,
    generator: PythonGenerator
  ) {
    // Get field values
    const className = block.getFieldValue("CLASS_NAME") || "class_name";

    // Generate code for CLASS_BODY using statementToCode
    const classBodyCode = generator.statementToCode(block, "CLASS_BODY") || "";

    // Generate the final code
    const code = `class ${className}():
    ${classBodyCode.trim()}
`;
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
