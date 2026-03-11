import * as Blockly from "blockly/core";

import {
  addBooleanValue,
  addCallbackFunction,
  addCasting,
  addClass,
  addClassInheritance,
  addFromImport,
  addFunction,
  addImport,
  addImportAs,
  addInformation,
  addInterface,
  addLiteral,
  addLiteralValue,
  addLogger,
  addMessage,
  addNumericValue,
  addPub,
  addPythonVersion,
  addStringValue,
  addSub,
  addVariable,
  addVelocity,
  classInit,
  counterFunction,
  createTimer,
  executeFunctionTimer,
  executeLiteral,
  initMessageInterface,
  initStatement,
  launchExecutable,
  nodeInit,
  publishMessage,
  rosCommunication,
  rosNodeTemplate,
  startNode,
  subscribeMessage,
} from "./definitions";

export async function defineCustomBlocks() {
  let interfaceOptions: string[][] = [["-", ""]];
  const executableOptions: string[][] = [
    ["Turtlesim (GUI)", "ros2 run turtlesim turtlesim_node"],
    ["Turtlesim Teleop", "ros2 run turtlesim turtle_teleop_key"],
  ];

  try {
    const result = await window.electronAPI.getInterfaces();
    const fetchedOptions = result.map((ROSInterface) => [
      ROSInterface.name,
      ROSInterface.location,
    ]);

    interfaceOptions = interfaceOptions.concat(fetchedOptions).sort();
  } catch (error) {
    console.error("Failed to fetch ROS2 interfaces:", error);
  }

  const blocks = [
    addClass,
    addClassInheritance,
    addImport,
    addImportAs,
    addFromImport,
    addFunction,
    classInit,
    nodeInit,
    startNode,
    initStatement,
    addInterface(interfaceOptions),
    addPub(interfaceOptions),
    rosCommunication(interfaceOptions),
    addVariable,
    addNumericValue,
    addStringValue,
    addBooleanValue,
    addCasting,
    addLiteralValue,
    addLiteral,
    executeLiteral,
    executeFunctionTimer,
    addPythonVersion,
    addLogger,
    initMessageInterface(interfaceOptions),
    addMessage(interfaceOptions),
    publishMessage,
    launchExecutable(executableOptions),
    subscribeMessage(interfaceOptions),
    addCallbackFunction(interfaceOptions),
    createTimer,
    counterFunction(interfaceOptions),
    addInformation,
    addVelocity,
    addSub(interfaceOptions),
    rosNodeTemplate(interfaceOptions),
  ];

  // Register the blocks using createBlockDefinitionsFromJsonArray
  Blockly.defineBlocksWithJsonArray(blocks);

  const toggleRosCommunicationCallbackVisibility = (
    blockInstance: Blockly.Block,
  ) => {
    const isSubscriber =
      blockInstance.getFieldValue("COMM_TYPE") === "subscriber";
    const callbackField = blockInstance.getField("CALLBACK_NAME");
    const callbackLabel = blockInstance.getField("CALLBACK_LABEL");
    callbackField?.setVisible(isSubscriber);
    callbackLabel?.setVisible(isSubscriber);
  };

  const rosCommunicationBlock = Blockly.Blocks["ros_communication"];
  const originalRosCommunicationInit = rosCommunicationBlock?.init;
  if (rosCommunicationBlock && originalRosCommunicationInit) {
    rosCommunicationBlock.init = function (...args: unknown[]) {
      originalRosCommunicationInit.apply(this, args);
      toggleRosCommunicationCallbackVisibility(this);
    };
  }

  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  Blockly.Blocks["ros_communication"].onchange = function (event: any) {
    if (!this.workspace || this.isInFlyout) return;
    if (
      !event ||
      (event.type === Blockly.Events.BLOCK_CHANGE &&
        event.blockId === this.id &&
        event.name === "COMM_TYPE")
    ) {
      toggleRosCommunicationCallbackVisibility(this);
    }
  };

  // After Blockly.defineBlocksWithJsonArray(blocks);

  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  Blockly.Blocks["subscribe_message"].onchange = async function (event: any) {
    if (!this.workspace || this.isInFlyout) return;
    if (
      event &&
      event.type === Blockly.Events.BLOCK_CHANGE &&
      event.blockId === this.id &&
      event.name === "MESSAGE_INTERFACE"
    ) {
      const interfaceLocation = this.getFieldValue("MESSAGE_INTERFACE");
      const propertyField = this.getField("PROPERTY");
      if (!propertyField) return;

      // Fetch properties for the selected interface (replace with your real fetch)
      let options = [["Selecione a interface", ""]];
      if (interfaceLocation) {
        // Example: replace with your async fetch
        const properties: MessageProperty[] =
          await window.electronAPI.getMessageProperties(interfaceLocation);
        options =
          properties && properties.length > 0
            ? properties.map((prop: MessageProperty) => [
                prop.name,
                prop.property,
              ])
            : [["Sem propriedades", ""]];
      }
      propertyField.menuGenerator_ = options;
      propertyField.setValue(options[0][1]);
      // eslint-disable-next-line @typescript-eslint/no-unused-expressions
      propertyField.forceRerender && propertyField.forceRerender();
    }
  };

  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  Blockly.Blocks["add_message"].onchange = async function (event: any) {
    if (!this.workspace || this.isInFlyout) return;
    if (
      event &&
      event.type === Blockly.Events.BLOCK_CHANGE &&
      event.blockId === this.id &&
      event.name === "INTERFACE"
    ) {
      const interfaceLocation = this.getFieldValue("INTERFACE");
      const propertyField = this.getField("PROPERTY");
      if (!propertyField) return;

      // Fetch properties for the selected interface (replace with your real fetch)
      let options = [["Selecione a interface", ""]];
      if (interfaceLocation) {
        // Example: replace with your async fetch
        const properties: MessageProperty[] =
          await window.electronAPI.getMessageProperties(interfaceLocation);
        options =
          properties && properties.length > 0
            ? properties.map((prop: MessageProperty) => [
                prop.name,
                prop.property,
              ])
            : [["Sem propriedades", ""]];
      }
      propertyField.menuGenerator_ = options;
      propertyField.setValue(options[0][1]);
      // eslint-disable-next-line @typescript-eslint/no-unused-expressions
      propertyField.forceRerender && propertyField.forceRerender();
    }
  };
}
