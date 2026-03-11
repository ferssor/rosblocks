export const addPythonVersion = {
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
};
