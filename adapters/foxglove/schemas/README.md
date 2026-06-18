# Foxglove well-known JSON schemas

`McapWriter` registers each channel with a Foxglove well-known schema *name*
(e.g. `foxglove.SceneUpdate`). Lichtblick/Foxglove recognize these by name, so
rendering works even when the schema *text* is empty.

For fully self-describing logs (any consumer, not just Foxglove), drop the
official jsonschema files here so `FoxgloveDebugRecorder` registers the real
schema text alongside the name:

- `SceneUpdate.json`
- `LocationFix.json`
- `FrameTransform.json`
- `Log.json`

Source: <https://github.com/foxglove/foxglove-sdk> under `schemas/jsonschema/`.
(One-time fetch; not required for Lichtblick rendering.)
