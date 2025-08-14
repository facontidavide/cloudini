# foxglove-cloudini-converter

A Foxglove extension to convert `Cloudini` compressed point clouds to a standard `PointCloud2` message. Originally developed by [Ascento AG](https://www.ascento.ai/).

## Usage

In this folder:

1. start copying the file "cloudini_wasm.js" that you previously compiled as explained [here](../README.md) into "cloudini_foxglove/src"
2. Run `npm install`
2. Build the extension with `npm run package`. This should create a file called "release/cloudini.foxglove-cloudini-converter-*.foxe"
3. Open the extension manager in Foxglove: "Settings -> Extensions".
4. Drag and drop the `*.foxe` file into the extension manager.

