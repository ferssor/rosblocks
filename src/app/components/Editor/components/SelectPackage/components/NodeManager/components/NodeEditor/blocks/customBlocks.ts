import * as Blockly from "blockly/core";

export async function defineCustomBlocks() {
  let interfaceOptions: string[][] = [["-", ""]];

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
    {
      type: "add_class",
      message0: "Iniciar classe %1",
      args0: [
        {
          type: "field_input",
          name: "CLASS_NAME",
          text: "Defina o nome da classe",
        },
      ],
      message1: "%1",
      args1: [
        {
          type: "input_statement",
          name: "CLASS_BODY",
        },
      ],
      colour: "#40a02b",
      previousStatement: null,
      nextStatement: null,
      tooltip: "Define uma classe em Python.",
      helpUrl: "",
    },
    {
      type: "add_class_inheritance",
      message0: "Iniciar classe %1 herdando da classe %2",
      args0: [
        {
          type: "field_input",
          name: "CLASS_NAME",
          text: "Defina o nome da classe",
        },
        {
          type: "field_input",
          name: "EXTENDS_NAME",
          text: "Defina o nome da classe a ser herdada",
        },
      ],
      message1: "%1",
      args1: [
        {
          type: "input_statement",
          name: "CLASS_BODY",
        },
      ],
      colour: "#40a02b",
      previousStatement: null,
      nextStatement: null,
      tooltip: "Define uma classe em Python.",
      helpUrl: "",
    },
    {
      type: "add_import",
      message0: "Importar %1",
      args0: [
        {
          type: "field_input",
          name: "IMPORT_NAME",
          text: "Defina o nome do pacote",
        },
      ],
      colour: "#fe640b",
      previousStatement: null,
      nextStatement: null,
      tooltip: "Importa um pacote Python",
      helpUrl: "",
    },
    {
      type: "add_import_as",
      message0: "Importar %1 como %2",
      args0: [
        {
          type: "field_input",
          name: "IMPORT_NAME",
          text: "Defina o nome do pacote",
        },
        {
          type: "field_input",
          name: "ALIAS_NAME",
          text: "Defina o nome do apelido",
        },
      ],
      colour: "#fe640b",
      previousStatement: null,
      nextStatement: null,
      tooltip: "Importa um pacote Python",
      helpUrl: "",
    },
    {
      type: "add_from_import",
      message0: "A partir de %1 importe %2",
      args0: [
        {
          type: "field_input",
          name: "PACKAGE_NAME",
          text: "Defina o nome do pacote",
        },
        {
          type: "field_input",
          name: "METHOD_NAME",
          text: "Defina o nome do método",
        },
      ],
      colour: "#fe640b",
      previousStatement: null,
      nextStatement: null,
      tooltip: "Importa um pacote Python",
      helpUrl: "",
    },
    {
      type: "add_function",
      message0: "Função %1",
      args0: [
        {
          type: "field_input",
          name: "FUNCTION_NAME",
          text: "Defina o nome da função",
        },
      ],
      message1: "%1",
      args1: [
        {
          type: "input_statement",
          name: "FUNCTION_BODY",
        },
      ],
      colour: "#8839ef",
      previousStatement: null,
      nextStatement: null,
      tooltip: "Adicione uma função",
      helpUrl: "",
    },
    {
      type: "class_init",
      message0: "Inicializa o construtor da classe %1",
      args0: [
        {
          type: "field_input",
          name: "CLASS_NAME",
          text: "Defina o nome da classe",
        },
      ],
      message1: "%1",
      args1: [
        {
          type: "input_statement",
          name: "CONSTRUCTOR_BODY",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#40a02b",
      tooltip: "Adicione o construtor da classe",
      helpUrl: "",
    },
    {
      type: "node_init",
      message0: "Configure o nó %1",
      args0: [
        {
          type: "field_input",
          name: "NODE_NAME",
          text: "Defina o nome do nó",
        },
      ],
      message1: "%1",
      args1: [
        {
          type: "input_statement",
          name: "CONSTRUCTOR_BODY",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#1e66f5",
      tooltip: "Adicione o nó",
      helpUrl: "",
    },
    {
      type: "start_node",
      message0: "Inicialize o nó da classe %1 com a função %2",
      args0: [
        {
          type: "field_input",
          name: "CLASS_NAME",
          text: "Defina o nome da classe",
        },
        {
          type: "field_input",
          name: "FUNCTION_NAME",
          text: "Defina o nome da função",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#1e66f5",
      tooltip: "Adicione o nó",
      helpUrl: "",
    },
    {
      type: "init_statement",
      message0: "Inicializa a função %1 no script",
      args0: [
        {
          type: "field_input",
          name: "FUNCTION_NAME",
          text: "Defina o nome da função",
        },
      ],
      previousStatement: null,
      colour: "#40a02b",
      tooltip: "Adicione o inicializador do script",
      helpUrl: "",
    },
    {
      type: "add_interface",
      message0: "Importe a interface %1",
      args0: [
        {
          type: "field_dropdown",
          name: "INTERFACE",
          options: interfaceOptions,
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#1e66f5",
      tooltip: "Adicione uma interface ROS2",
      helpUrl: "",
    },
    {
      type: "add_pub",
      message0:
        "Adicionar publicador %1 no tópico %2 com interface %3 e taxa de atualização %4 Hz",
      args0: [
        {
          type: "field_input",
          name: "PUB_NAME",
          text: "Defina o nome do publicador",
        },
        {
          type: "field_input",
          name: "TOPIC_NAME",
          text: "Defina o nome do tópico",
        },
        {
          type: "field_dropdown",
          name: "INTERFACE",
          options: interfaceOptions,
        },
        {
          type: "field_number",
          name: "REFRESH_RATE",
          value: 10,
          min: 1,
          precision: 1,
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#1e66f5",
      tooltip:
        "Define um publisher ROS com nome, interface e taxa de atualização.",
      helpUrl: "",
    },
    {
      type: "add_variable",
      message0: "Adicione à variável %1 o valor %2",
      args0: [
        {
          type: "field_input",
          name: "VARIABLE",
          text: "Defina o nome da variável",
        },
        {
          type: "input_value",
          name: "VALUE",
          text: "Defina o valor da variável",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#e64553",
      tooltip: "Adicione uma variável ao script",
      helpUrl: "",
    },
    {
      type: "add_numeric_value",
      message0: "%1",
      args0: [
        {
          type: "field_number",
          name: "NUMERIC_VALUE",
          text: "Defina o valor da variável",
        },
      ],
      colour: "#df8e1d",
      output: null,
      tooltip: "Adicione uma variável ao script",
      helpUrl: "",
    },
    {
      type: "add_string_value",
      message0: `" %1 "`,
      args0: [
        {
          type: "field_input",
          name: "STRING_VALUE",
        },
      ],
      colour: "#df8e1d",
      output: null,
      tooltip: "Adicione uma variável ao script",
      helpUrl: "",
    },
    {
      type: "add_boolean_value",
      message0: `%1`,
      args0: [
        {
          type: "field_dropdown",
          options: [
            ["Verdadeiro", "True"],
            ["Falso", "False"],
          ],
          name: "BOOL_VALUE",
          checked: true,
        },
      ],
      colour: "#df8e1d",
      output: null,
      tooltip: "Adicione uma variável ao script",
      helpUrl: "",
    },
    {
      type: "add_literal_value",
      message0: `%1`,
      args0: [
        {
          type: "field_input",
          name: "LITERAL_VALUE",
          text: "Defina o nome da váriavel",
        },
      ],
      colour: "#df8e1d",
      output: null,
      tooltip: "Descreve uma variável literal",
      helpUrl: "",
    },
    {
      type: "add_counter",
      message0: `Adicione o contador a função %1 com intervalo de %2`,
      args0: [
        {
          type: "field_input",
          name: "FUNCTION_NAME",
          text: "Defina o nome da função",
        },
        {
          type: "field_number",
          name: "INTERVAL",
          value: 1,
          min: 1,
          precision: 1,
        },
      ],
      colour: "#1e66f5",
      output: null,
      tooltip: "Adiciona um contador com intervalo que chama uma função",
      helpUrl: "",
    },
    {
      type: "add_python_version",
      message0: "Defina a versão %1 do Python",
      args0: [
        {
          type: "field_dropdown",
          options: [
            ["3", "3"],
            ["3.6", "3.6"],
            ["3.7", "3.7"],
            ["3.8", "3.8"],
            ["3.9", "3.9"],
            ["3.10", "3.10"],
          ],
          name: "PYTHON_VERSION",
          checked: true,
        },
      ],
      colour: "#fe640b",
      nextStatement: null,
      tooltip: "Adicione uma variável ao script",
      helpUrl: "",
    },
    {
      type: "add_logger",
      message0: "Registre %1: %2",
      args0: [
        {
          type: "field_dropdown",
          options: [
            ["a informação", "info"],
            ["o erro", "error"],
            ["o aviso", "warning"],
            ["o debug", "debug"],
          ],
          name: "LOG_LEVEL",
          checked: true,
        },
        {
          type: "input_value",
          name: "LOG_TEXT",
          text: "Defina o texto que será exibido",
        },
      ],
      colour: "#1e66f5",
      previousStatement: null,
      nextStatement: null,
      tooltip: "Adicione um registro ao nó",
      helpUrl: "",
    },
    {
      type: "add_message",
      message0:
        "Adicione a mensagem com a interface %1 e os dados da variável %2",
      args0: [
        {
          type: "field_dropdown",
          name: "INTERFACE",
          options: interfaceOptions,
        },
        {
          type: "field_input",
          name: "VARIABLE_NAME",
          text: "Defina o nome da variável",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#1e66f5",
      tooltip: "Adicione uma mensagem ROS2",
      helpUrl: "",
    },
    {
      type: "publish_message",
      message0: "Publique a mensagem no %1",
      args0: [
        {
          type: "field_input",
          name: "PUBLISHER_NAME",
          text: "Defina o nome do publicador",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#1e66f5",
      tooltip: "Publica a mensagem no publicador",
      helpUrl: "",
    },
    {
      type: "subscribe_message",
      message0: `Atribua o valor da mensagem com a interface %1
         e a propriedade %2 na variável %3`,
      args0: [
        {
          type: "field_dropdown",
          name: "MESSAGE_INTERFACE",
          options: interfaceOptions,
        },
        {
          type: "field_dropdown",
          name: "PROPERTY",
          options: [["Selecione interface primeiro", ""]],
        },
        {
          type: "field_input",
          name: "VARIABLE_NAME",
          text: "Defina o nome da variável",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#1e66f5",
      tooltip: "Armazena os dados da mensagem no assinante",
      helpUrl: "",
    },
    {
      type: "add_callback_function",
      message0: `Adicione a função de retorno %1 com a interface %2`,
      args0: [
        {
          type: "field_input",
          name: "FUNCTION_NAME",
          text: "Defina o nome da função",
        },
        {
          type: "field_dropdown",
          name: "INTERFACE",
          options: interfaceOptions,
        },
      ],
      message1: "%1",
      args1: [
        {
          type: "input_statement",
          name: "FUNCTION_BODY",
        },
      ],
      colour: "#1e66f5",
      previousStatement: null,
      nextStatement: null,
      tooltip: "Adicione uma função de retorno de chamada",
      helpUrl: "",
    },
    {
      type: "create_timer",
      message0:
        "Criar um temporizador %1 com intervalo %2 e publique na função %3",
      args0: [
        {
          type: "field_input",
          name: "TEMP_NAME",
          text: "Defina o nome do temporizador",
        },
        {
          type: "field_number",
          name: "DURATION",
          value: 1.0,
          min: 1.0,
          precision: 1.0,
        },
        {
          type: "input_value",
          name: "PUBLISHER_COUNTER",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#1e66f5",
      tooltip: "Adiciona uma função contadora e publicadora",
      helpUrl: "",
    },
    {
      type: "counter_function",
      message0:
        "Adicione a função contadora com a interface %1 e inicie o contador com o valor %2 e publique em %3",
      args0: [
        {
          type: "field_dropdown",
          name: "COUNTER_INTERFACE",
          options: interfaceOptions,
        },
        {
          type: "field_number",
          name: "COUNTER",
          value: 0,
          min: 0,
          precision: 1,
        },
        {
          type: "field_input",
          name: "PUBLISHER",
          text: "Defina o nome do publicador",
        },
      ],
      output: null,
      colour: "#1e66f5",
      tooltip: "Adiciona uma função que publica o contador",
      helpUrl: "",
    },
    {
      type: "add_information",
      message0: "Adicione o texto de log %1",
      args0: [
        {
          type: "field_input",
          name: "LOG_INFO",
          text: "Defina o texto que será exibido",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#1e66f5",
      tooltip: "Adiciona um texto de log",
      helpUrl: "",
    },
    {
      type: "add_velocity",
      message0: "Adicione uma mensagem de velocidade %1 com o valor %2",
      args0: [
        {
          type: "field_dropdown",
          name: "PROPERTY",
          options: [
            ["linear.x", "linear.x"],
            ["linear.y", "linear.y"],
            ["linear.z", "linear.z"],
            ["angular.x", "angular.x"],
            ["angular.y", "angular.y"],
            ["angular.z", "angular.z"],
          ],
        },
        {
          type: "field_number",
          name: "VELOCITY_VALUE",
          value: 0,
          min: 0,
          precision: 1,
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#1e66f5",
      tooltip: "Adiciona um valor de velocidade",
      helpUrl: "",
    },
    {
      type: "add_sub",
      message0: `Adicionar assinante %1 
      no tópico %2 
      com interface %3, 
      taxa de atualização %4 Hz 
      e armazene os dados na %5`,
      args0: [
        {
          type: "field_input",
          name: "SUB_NAME",
          text: "Defina o nome do assinante",
        },
        {
          type: "field_input",
          name: "TOPIC_NAME",
          text: "Defina o nome do tópico",
        },
        {
          type: "field_dropdown",
          name: "INTERFACE",
          options: interfaceOptions,
        },
        {
          type: "field_number",
          name: "REFRESH_RATE",
          value: 10,
          min: 1,
          precision: 1,
        },
        {
          type: "field_input",
          name: "FUNCTION_NAME",
          text: "Defina o nome da função",
        },
      ],
      previousStatement: null,
      nextStatement: null,
      colour: "#1e66f5",
      tooltip:
        "Define um subscriber ROS2 com nome, interface e taxa de atualização.",
      helpUrl: "",
    },
    {
      type: "init_node_template",
      message0: `Adicione o(s) pacote(s): %1`,
      args0: [
        {
          type: "input_statement",
          name: "IMPORT_PACKAGES",
        },
      ],
      message1: `Defina a classe %1`,
      args1: [
        {
          type: "field_input",
          name: "CLASS_NAME",
          text: "Digite o nome da classe",
        },
      ],
      message2: "%1",
      args2: [
        {
          type: "input_statement",
          name: "TEMPLATE_BODY",
        },
      ],
      message3: "inicialize o nó da classe %1",
      args3: [
        {
          type: "field_input",
          name: "NODE_CLASS_NAME",
          text: "Digite o nome da classe",
        },
      ],
      colour: "#5c5f77",
      tooltip:
        "Define um modelo de nó ROS2 com importações, classe e inicialização do nó.",
      helpUrl: "",
    },
  ];

  // Register the blocks using createBlockDefinitionsFromJsonArray
  Blockly.defineBlocksWithJsonArray(blocks);
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
}
