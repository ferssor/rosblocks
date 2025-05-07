import * as Blockly from "blockly/core";
import { pythonGenerator } from "blockly/python";

export function registerCustomBlocksToPython() {
  pythonGenerator.forBlock["add_class"] = function (block: Blockly.Block) {
    const className = block.getFieldValue("CLASS_NAME") || "class_name";
    const nodeName = block.getFieldValue("NODE_NAME") || "node_name";
    const code = `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class ${className}(Node):
    def __init__(self):
        super().__init__("${nodeName}")


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
}
