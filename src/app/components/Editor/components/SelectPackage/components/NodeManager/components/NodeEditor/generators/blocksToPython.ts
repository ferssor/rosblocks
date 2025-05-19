import * as Blockly from "blockly/core";
import { pythonGenerator } from "blockly/python";

export function registerCustomBlocksToPython() {
  pythonGenerator.forBlock["add_class"] = function (block: Blockly.Block) {
    const className = block.getFieldValue("CLASS_NAME") || "class_name";
    const nodeName: string = block.getFieldValue("NODE_NAME") || "node_name";
    const importLine = new Set<string>();
    let currentBlock = block.getInputTargetBlock("CLASS_BODY");
    let pubCode = "";
    let counterCode = "";
    let publisherCounterCode = "";
    let logCode = "";

    while (currentBlock) {
      if (currentBlock.type === "add_pub") {
        const interfaceValue = currentBlock.getFieldValue("INTERFACE");
        if (interfaceValue) {
          importLine.add(interfaceValue);
        }
        // Generate code for the current add_pub block
        pubCode +=
          pythonGenerator.forBlock["add_pub"](currentBlock, pythonGenerator) +
          "\n";
      } else if (currentBlock.type === "create_timer") {
        counterCode +=
          pythonGenerator.forBlock["create_timer"](
            currentBlock,
            pythonGenerator
          ) + "\n";

        const counterFunctionBlock =
          currentBlock.getInputTargetBlock("PUBLISHER_COUNTER");
        if (counterFunctionBlock) {
          publisherCounterCode +=
            pythonGenerator.forBlock["counter_function"](
              counterFunctionBlock,
              pythonGenerator
            ) + "\n";
        }
      } else if (currentBlock.type === "add_information") {
        logCode +=
          pythonGenerator.forBlock["add_information"](
            currentBlock,
            pythonGenerator
          ) + "\n";
      }

      // Move to the next block in the chain
      currentBlock = currentBlock.getNextBlock();
    }

    const msgImport = Array.from(importLine).join("\n");

    const code = `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
${msgImport}


class ${className}(Node):
    def __init__(self):
        super().__init__("${nodeName}")
        ${pubCode.trim()}
        ${counterCode.trim()}
        ${logCode.trim()}

${publisherCounterCode}
def main(args=None):
    rclpy.init(args=args)
    node = ${className}()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
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
    const interfaceName: string = block.getFieldValue("INTERFACE");
    const publisherName: string = block.getFieldValue("PUB_NAME");
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
}
