export const addVelocity = {
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
};
