import type { BlocklyOptions } from "blockly";

export const toolbox: BlocklyOptions["toolbox"] = {
  kind: "categoryToolbox",
  contents: [
    {
      kind: "category",
      name: "Lógica e Controle",
      colour: "210",
      // Subcategorias
      contents: [
        {
          kind: "category",
          name: "Condicionais",
          contents: [
            { kind: "block", type: "controls_if" },
            { kind: "block", type: "logic_compare" },
            { kind: "block", type: "logic_operation" },
            { kind: "block", type: "logic_negate" },
            { kind: "block", type: "logic_boolean" },
          ],
        },
        {
          kind: "category",
          name: "Repetição",
          contents: [
            { kind: "block", type: "controls_repeat_ext" },
            { kind: "block", type: "controls_whileUntil" },
            { kind: "block", type: "controls_for" },
            { kind: "block", type: "controls_flow_statements" },
          ],
        },
      ],
    },
    {
      kind: "category",
      name: "Dados",
      colour: "230",
      contents: [
        {
          kind: "category",
          name: "Matemática",
          contents: [
            { kind: "block", type: "math_number" },
            { kind: "block", type: "math_arithmetic" },
            { kind: "block", type: "math_single" },
          ],
        },
        {
          kind: "category",
          name: "Texto",
          contents: [
            { kind: "block", type: "text" },
            { kind: "block", type: "text_join" },
            { kind: "block", type: "text_print" },
          ],
        },
        {
          kind: "category",
          name: "Variáveis",
          contents: [{ kind: "block", type: "add_variable" }],
        },
      ],
    },
    {
      kind: "category",
      name: "ROS",
      colour: "20",
      contents: [
        {
          kind: "category",
          name: "Estrutura",
          contents: [{ kind: "block", type: "ros_node_template" }],
        },
        {
          kind: "category",
          name: "Definição do Nó",
          contents: [{ kind: "block", type: "node_init" }],
        },
        {
          kind: "category",
          name: "Informação",
          contents: [{ kind: "block", type: "add_logger" }],
        },
        {
          kind: "category",
          name: "Comunicação",
          contents: [
            { kind: "block", type: "ros_communication" },
            { kind: "block", type: "add_pub" },
            { kind: "block", type: "add_sub" },
            { kind: "block", type: "add_interface" },
            { kind: "block", type: "init_message_interface" },
            { kind: "block", type: "add_message" },
            { kind: "block", type: "publish_message" },
            { kind: "block", type: "subscribe_message" },
          ],
        },
        {
          kind: "category",
          name: "Lógica e Funções",
          contents: [
            { kind: "block", type: "add_function" },
            { kind: "block", type: "add_callback_function" },
            { kind: "block", type: "create_timer" },
            { kind: "block", type: "execute_function_timer" },
            { kind: "block", type: "counter_function" },
            { kind: "block", type: "add_variable" },
            { kind: "block", type: "add_casting" },
            { kind: "block", type: "add_logger" },
            { kind: "block", type: "add_information" },
            { kind: "block", type: "add_velocity" },
            { kind: "block", type: "add_literal" },
            { kind: "block", type: "execute_literal" },
          ],
        },
      ],
    },
  ],
};
