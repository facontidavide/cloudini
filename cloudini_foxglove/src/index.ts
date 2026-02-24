import { ExtensionContext, Immutable, MessageEvent } from "@foxglove/extension";
import { convertPointCloudWasm, loadCloudiniWasm } from "./PointCloudConverter";

import { CloudiniCompressedPointCloud } from "./Schemas";

export function activate(extensionContext: ExtensionContext): void {
  // Preload WASM module
  loadCloudiniWasm().catch(console.error);

  extensionContext.registerMessageConverter<CloudiniCompressedPointCloud>({
    type: "schema",
    fromSchemaName: "perception_types_ipc.proto.CloudiniCompressedPointCloud",
    toSchemaName: "foxglove.PointCloud",
    converter: (inputMessage: CloudiniCompressedPointCloud, _messageEvent: Immutable<MessageEvent<CloudiniCompressedPointCloud>>) => {
      return convertPointCloudWasm(inputMessage);
    },
  });
}
