/** @type {import('tailwindcss').Config} */
export default {
  content: ["./index.html", "./src/**/*.{js,jsx}"],
  theme: {
    extend: {
      colors: {
        console: {
          ink: "#18202f",
          panel: "#f7f8fb",
          rail: "#111827",
          line: "#d9dde8"
        },
        signal: {
          ok: "#178f62",
          warn: "#b7791f",
          danger: "#c2410c",
          info: "#2563eb"
        }
      },
      boxShadow: {
        soft: "0 12px 32px rgba(17, 24, 39, 0.08)"
      }
    }
  },
  plugins: []
};
