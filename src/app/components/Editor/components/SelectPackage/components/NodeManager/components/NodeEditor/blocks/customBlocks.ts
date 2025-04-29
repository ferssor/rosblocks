import * as Blockly from "blockly/core";

export function defineCustomBlocks() {
  // Define the add_main block
  Blockly.Blocks["add_main"] = {
    init: function () {
      this.appendDummyInput()
        .appendField("Add main function")
        .appendField(new Blockly.FieldTextInput("node_name"), "NODE_NAME");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(259);
      this.setTooltip("Creates a ROS2 main function template.");
      this.setHelpUrl("");
    },
  };

  // Define the add_pub block
  Blockly.Blocks["add_pub"] = {
    init: function () {
      this.appendValueInput("TEXT")
        .setCheck("String")
        .appendField("Add publisher");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour(259);
      this.setTooltip(
        "Creates a ROS2 publisher with the specified topic name."
      );
      this.setHelpUrl("");
    },
  };
}
