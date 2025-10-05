(function Template() {
  // --- helpers ---
  function toPascalCase(str) {
    return str
      .replace(/(?:^\w|[A-Z]|\b\w)/g, function (fl) {
        return fl.toUpperCase();
      })
      .replace(/\W+/g, "");
  }
  function toCamelCase(str) {
    var p = toPascalCase(str);
    return p.charAt(0).toLowerCase() + p.slice(1);
  }
  function toKebabCase(str) {
    return toCamelCase(str).replace(/([A-Z])/g, function (m) {
      return "-" + m.toLowerCase();
    });
  }

  return {
    userInputs: [
      {
        title: "What is the Component Name",
        argumentName: "name",
        defaultValue: "SampleComponent",
      },
    ],

    template: [
      {
        type: "folder",
        name: function (inputs) {
          return toKebabCase(inputs.name);
        },
        children: [
          // index.ts
          {
            type: "file",
            name: function () {
              return "index.ts";
            },
            content: function (inputs) {
              return (
                "export type { " +
                toPascalCase(inputs.name) +
                "Props } from './types';\n" +
                "export { default as " +
                toPascalCase(inputs.name) +
                " } from './" +
                toKebabCase(inputs.name) +
                "';"
              );
            },
          },

          // types.ts
          {
            type: "file",
            name: function () {
              return "types.ts";
            },
            content: function (inputs) {
              return (
                "export interface BaseProps {\n" +
                "  className?: string;\n" +
                "  style?: React.CSSProperties;\n" +
                "  id?: string;\n" +
                "}\n\n" +
                "export interface " +
                toPascalCase(inputs.name) +
                "Props extends BaseProps {\n" +
                "  children?: React.ReactNode;\n" +
                "}\n"
              );
            },
          },

          // component .tsx (usa t('title'))
          {
            type: "file",
            name: function (inputs) {
              return toKebabCase(inputs.name) + ".tsx";
            },
            content: function (inputs) {
              return (
                "import React, { memo } from 'react';\n" +
                "import { " +
                toPascalCase(inputs.name) +
                "Props } from './types';\n" +
                "import use" +
                toPascalCase(inputs.name) +
                "Hook from './" +
                toKebabCase(inputs.name) +
                ".hook';\n" +
                "import styles from './" +
                toKebabCase(inputs.name) +
                ".styles.module.css';\n\n" +
                "function " +
                toPascalCase(inputs.name) +
                "(props: " +
                toPascalCase(inputs.name) +
                "Props) {\n" +
                "  const { className = '', style, children } = props;\n" +
                "  const { t, state, handlers } = use" +
                toPascalCase(inputs.name) +
                "Hook();\n\n" +
                "  return (\n" +
                "    <div\n" +
                "      id={props.id}\n" +
                "      className={(styles.container + ' ' + (className || '')).trim()}\n" +
                "      style={style}\n" +
                "      onClick={handlers.onClick}\n" +
                "    >\n" +
                "      <h2 className={styles.title}>{t('title')}</h2>\n" +
                "      {state.showContent && children}\n" +
                "    </div>\n" +
                "  );\n" +
                "}\n\n" +
                "export default memo(" +
                toPascalCase(inputs.name) +
                ");\n"
              );
            },
          },

          // hook padronizado (fixa o namespace do componente)
          {
            type: "file",
            name: function (inputs) {
              return toKebabCase(inputs.name) + ".hook.ts";
            },
            content: function (inputs) {
              return (
                "import { useTranslation } from 'react-i18next';\n" +
                "import { useState, useCallback } from 'react';\n" +
                "import './i18n';\n\n" +
                "function use" +
                toPascalCase(inputs.name) +
                "Hook() {\n" +
                "  const { t } = useTranslation('" +
                toKebabCase(inputs.name) +
                "');\n" +
                "  const [showContent, setShowContent] = useState(true);\n" +
                "  const handleClick = useCallback(function () { setShowContent(function (p) { return !p; }); }, []);\n" +
                "  return { t: t, state: { showContent: showContent }, handlers: { onClick: handleClick } };\n" +
                "}\n\n" +
                "export default use" +
                toPascalCase(inputs.name) +
                "Hook;\n"
              );
            },
          },

          // styles (CSS Modules)
          {
            type: "file",
            name: function (inputs) {
              return toKebabCase(inputs.name) + ".styles.module.css";
            },
            content: function () {
              return (
                ".container {\n" +
                "  width: 100%;\n" +
                "  display: flex;\n" +
                "  flex-direction: column;\n" +
                "  gap: 0.5rem;\n" +
                "  padding: 0.5rem;\n" +
                "  border: 1px solid var(--color-border, #ddd);\n" +
                "  border-radius: 8px;\n" +
                "}\n" +
                ".title { font-size: 1.2rem; font-weight: 600; color: var(--color-text-primary, #222); }\n"
              );
            },
          },

          // i18n (formato solicitado)
          {
            type: "folder",
            name: function () {
              return "i18n";
            },
            children: [
              {
                type: "file",
                name: function () {
                  return "en_US.json";
                },
                content: function (inputs) {
                  return (
                    '{\n  "title": "' +
                    toPascalCase(inputs.name) +
                    ' Component"\n}\n'
                  );
                },
              },
              {
                type: "file",
                name: function () {
                  return "pt_BR.json";
                },
                content: function (inputs) {
                  return (
                    '{\n  "title": "Componente ' +
                    toPascalCase(inputs.name) +
                    '"\n}\n'
                  );
                },
              },
              {
                type: "file",
                name: function () {
                  return "index.ts";
                },
                content: function (inputs) {
                  var ns = toKebabCase(inputs.name);
                  return (
                    "import en_US from './en_US.json';\n" +
                    "import pt_BR from './pt_BR.json';\n\n" +
                    "const translations = {\n" +
                    "  'en_US': { " +
                    ns +
                    ": en_US },\n" +
                    "  'pt_BR': { " +
                    ns +
                    ": pt_BR },\n" +
                    "};\n\n" +
                    "export default translations;\n"
                  );
                },
              },
            ],
          },
        ],
      },
    ],
  };
});
