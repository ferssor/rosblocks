import * as Blockly from "blockly/core";
import { pythonGenerator } from "blockly/python";

export function registerCustomBlocksToPython() {
  pythonGenerator.forBlock["add_class"] = function (block: Blockly.Block) {
    const className = block.getFieldValue("CLASS_NAME") || "class_name";
    const nodeName: string = block.getFieldValue("NODE_NAME") || "node_name";
    const importLine = new Set<string>();
    let currentBlock = block.getInputTargetBlock("CLASS_BODY");
    let pubCode = "";

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
    const publisherName = block.getFieldValue("PUB_NAME");
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
}
