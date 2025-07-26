import { ExtensionContext, Immutable, MessageEvent } from "@foxglove/extension";
import { convertPointCloudWasm, loadCloudiniWasm } from "./PointCloudConverter";

import { CompressedPointCloud } from "./Schemas";

export function activate(extensionContext: ExtensionContext): void {
  // Preload WASM module
  loadCloudiniWasm().catch(console.error);

  extensionContext.registerMessageConverter<CompressedPointCloud>({
    type: "schema",
    fromSchemaName: "point_cloud_interfaces/msg/CompressedPointCloud2",
    toSchemaName: "sensor_msgs/msg/PointCloud2",
    converter: (inputMessage: CompressedPointCloud, _messageEvent: Immutable<MessageEvent<CompressedPointCloud>>) => {
      return convertPointCloudWasm(inputMessage);
    },
  });
}
