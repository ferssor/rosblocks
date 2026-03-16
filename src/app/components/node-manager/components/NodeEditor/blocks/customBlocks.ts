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

type StringDropdownOption = [string, string];
type BlocklyChangeEvent = Blockly.Events.Abstract | undefined;

type ROSInterfaceOption = {
  name: string;
  location: string;
};

type MessageProperty = {
  name: string;
  property: string;
};

const NONE_OPTION: StringDropdownOption = ["None", ""];
const DEFAULT_INTERFACE_OPTION: StringDropdownOption = [
  "Selecione a interface",
  "",
];
const EMPTY_PROPERTIES_OPTION: StringDropdownOption = ["Sem propriedades", ""];

export async function defineCustomBlocks() {
  let interfaceOptions: StringDropdownOption[] = [["-", ""]];
  const executableOptions: StringDropdownOption[] = [
    ["Turtlesim (GUI)", "ros2 run turtlesim turtlesim_node"],
    ["Turtlesim Teleop", "ros2 run turtlesim turtle_teleop_key"],
  ];

  try {
    const result = await window.electronAPI.getInterfaces();
    const fetchedOptions: StringDropdownOption[] = result.map(
      (rosInterface: ROSInterfaceOption) => [
        rosInterface.name,
        rosInterface.location,
      ],
    );

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
    createTimer(),
    counterFunction(interfaceOptions),
    addInformation,
    addVelocity,
    addSub(interfaceOptions),
    rosNodeTemplate(),
  ];

  // Register the blocks using createBlockDefinitionsFromJsonArray
  Blockly.defineBlocksWithJsonArray(blocks);

  const isBlockChangeEvent = (
    event: Blockly.Events.Abstract | undefined,
  ): event is Blockly.Events.BlockChange => {
    return event instanceof Blockly.Events.BlockChange;
  };

  const getAvailableFunctionOptions = (
    workspace: Blockly.Workspace | null,
  ): StringDropdownOption[] => {
    if (!workspace) {
      return [NONE_OPTION];
    }

    const functionNames = workspace
      .getAllBlocks(false)
      .filter(
        (blockInstance) =>
          !blockInstance.isInFlyout &&
          !blockInstance.isDisposed() &&
          blockInstance.isEnabled() &&
          (blockInstance.type === "add_function" ||
            blockInstance.type === "add_callback_function"),
      )
      .map((blockInstance) => blockInstance.getFieldValue("FUNCTION_NAME"))
      .filter((functionName): functionName is string => Boolean(functionName));

    const uniqueFunctionNames = [...new Set(functionNames)];
    const options: StringDropdownOption[] = uniqueFunctionNames.map(
      (functionName) => [functionName, functionName],
    );

    return options.length > 0 ? options : [NONE_OPTION];
  };

  const setDropdownOptions = (
    field: Blockly.FieldDropdown,
    options: Blockly.MenuGenerator,
  ) => {
    (
      field as unknown as {
        menuGenerator_: Blockly.MenuGenerator;
      }
    ).menuGenerator_ = options;
  };

  const syncCreateTimerFunctionField = (blockInstance: Blockly.Block) => {
    const functionField = blockInstance.getField(
      "FUNCTION_NAME",
    ) as Blockly.FieldDropdown | null;
    if (!functionField) return;

    const currentValue = functionField.getValue();
    const options = getAvailableFunctionOptions(blockInstance.workspace);

    setDropdownOptions(functionField, () =>
      getAvailableFunctionOptions(blockInstance.workspace),
    );

    const hasCurrentValue = options.some(([, value]) => value === currentValue);
    if (hasCurrentValue) return;

    functionField.setValue(options[0][1]);
  };

  const shouldSyncCreateTimer = (event?: BlocklyChangeEvent) => {
    return (
      !event ||
      event.type === Blockly.Events.BLOCK_CHANGE ||
      event.type === Blockly.Events.BLOCK_CREATE ||
      event.type === Blockly.Events.BLOCK_DELETE ||
      event.type === Blockly.Events.BLOCK_MOVE
    );
  };

  const getDropdownField = (blockInstance: Blockly.Block, fieldName: string) => {
    return blockInstance.getField(fieldName) as Blockly.FieldDropdown | null;
  };

  const getMessagePropertyOptions = async (interfaceLocation: string) => {
    if (!interfaceLocation) {
      return [DEFAULT_INTERFACE_OPTION];
    }

    const properties: MessageProperty[] =
      await window.electronAPI.getMessageProperties(interfaceLocation);

    if (properties.length === 0) {
      return [EMPTY_PROPERTIES_OPTION];
    }

    return properties.map(
      (prop): StringDropdownOption => [prop.name, prop.property],
    );
  };

  const refreshPropertyDropdown = async (
    blockInstance: Blockly.Block,
    interfaceFieldName: string,
  ) => {
    const propertyField = getDropdownField(blockInstance, "PROPERTY");
    if (!propertyField) return;

    const interfaceLocation = blockInstance.getFieldValue(interfaceFieldName);
    const options = await getMessagePropertyOptions(interfaceLocation);

    setDropdownOptions(propertyField, options);
    propertyField.setValue(options[0][1]);
    propertyField.forceRerender();
  };

  const createTimerBlock = Blockly.Blocks["create_timer"];
  const originalCreateTimerInit = createTimerBlock?.init;
  if (createTimerBlock && originalCreateTimerInit) {
    createTimerBlock.init = function (this: Blockly.Block, ...args: unknown[]) {
      originalCreateTimerInit.apply(this, args);
      syncCreateTimerFunctionField(this);
    };
  }

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
    rosCommunicationBlock.init = function (
      this: Blockly.Block,
      ...args: unknown[]
    ) {
      originalRosCommunicationInit.apply(this, args);
      toggleRosCommunicationCallbackVisibility(this);
    };
  }

  Blockly.Blocks["ros_communication"].onchange = function (
    this: Blockly.Block,
    event?: BlocklyChangeEvent,
  ) {
    if (!this.workspace || this.isInFlyout) return;
    if (
      !event ||
      (isBlockChangeEvent(event) &&
        event.blockId === this.id &&
        event.name === "COMM_TYPE")
    ) {
      toggleRosCommunicationCallbackVisibility(this);
    }
  };

  Blockly.Blocks["create_timer"].onchange = function (
    this: Blockly.Block,
    event?: BlocklyChangeEvent,
  ) {
    if (!this.workspace || this.isInFlyout) return;

    if (shouldSyncCreateTimer(event)) {
      syncCreateTimerFunctionField(this);
    }
  };

  // After Blockly.defineBlocksWithJsonArray(blocks);

  Blockly.Blocks["subscribe_message"].onchange = async function (
    this: Blockly.Block,
    event?: BlocklyChangeEvent,
  ) {
    if (!this.workspace || this.isInFlyout) return;
    if (
      isBlockChangeEvent(event) &&
      event.blockId === this.id &&
      event.name === "MESSAGE_INTERFACE"
    ) {
      await refreshPropertyDropdown(this, "MESSAGE_INTERFACE");
    }
  };

  Blockly.Blocks["add_message"].onchange = async function (
    this: Blockly.Block,
    event?: BlocklyChangeEvent,
  ) {
    if (!this.workspace || this.isInFlyout) return;
    if (
      isBlockChangeEvent(event) &&
      event.blockId === this.id &&
      event.name === "INTERFACE"
    ) {
      await refreshPropertyDropdown(this, "INTERFACE");
    }
  };

  // Handler para importações dinâmicas no ros_node_template
  Blockly.Blocks["ros_node_template"].onchange = function (
    this: Blockly.Block,
    event?: BlocklyChangeEvent,
  ) {
    if (!this.workspace || this.isInFlyout) return;

    // Detecta mudanças ou movimentos que podem exigir atualização dos imports
    if (
      event &&
      (event.type === Blockly.Events.BLOCK_MOVE ||
        event.type === Blockly.Events.BLOCK_CHANGE)
    ) {
      // TODO: Implementar lógica para:
      // 1. Varrer blocos conectados em 'NODE_BODY'
      // 2. Coletar interfaces usadas (ex: std_msgs/msg/String)
      // 3. Verificar/Adicionar blocos 'add_from_import' em 'IMPORTS'
    }
  };
}
