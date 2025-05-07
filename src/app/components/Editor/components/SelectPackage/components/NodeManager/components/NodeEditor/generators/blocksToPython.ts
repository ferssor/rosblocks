import * as Blockly from "blockly/core";
import { pythonGenerator } from "blockly/python";

export function registerCustomBlocksToPython() {
  pythonGenerator.forBlock["add_main"] = function (block: Blockly.Block) {
    const nodeName = block.getFieldValue("NODE_NAME") || "node_name";
    const code = `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyCustomNode(Node):
    def __init__(self):
        super().__init__('${nodeName}')


def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
`;
    return code;
  };

  pythonGenerator.forBlock["add_pub"] = function () {
    const code = `hello`;
    return code;
  };
}
