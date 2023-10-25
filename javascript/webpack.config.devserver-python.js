const path = require("path");
const HtmlWebpackPlugin = require("html-webpack-plugin");
var DeclarationBundlerPlugin = require('declaration-bundler-webpack-plugin');

module.exports = {
  entry: {
    live: "./src/live.ts",
    NimbleStandaloneReact: "./src/NimbleStandaloneReact.ts",
    NimbleStandalone: "./src/NimbleStandalone.ts",
    NimbleRemote: "./src/NimbleRemote.ts"
  },
  module: {
    rules: [
      {
        test: /\.(js|ts)$/,
        exclude: /node_modules/,
        use: ["babel-loader"],
      },
      {
        test: /\.s[ac]ss$/i,
        use: [
          // Creates `style` nodes from JS strings
          "style-loader",
          // Translates CSS into CommonJS
          "css-loader",
          // Compiles Sass to CSS
          "sass-loader",
        ],
      },
      {
        test: /\.txt$/i,
        use: "raw-loader",
      },
    ],
  },
  resolve: {
    extensions: ["*", ".ts", ".js"],
  },
  output: {
    path: path.join(__dirname, "dist"),
    publicPath: "/",
    filename: "[name].js"
  },
  plugins: [
    new HtmlWebpackPlugin({
      template: path.join(__dirname, "src", "index.html")
    })
  ],
  devServer: {
    contentBase: path.join(__dirname, "dist"),
    // compress: true,
    port: 9000,
  },
};
