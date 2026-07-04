#pragma once
namespace navtracker::foxglove {
/**
 * Well-known Foxglove schema-name constants used when writing MCAP debug
 * recordings. The `foxglove.*` names are recognized by Lichtblick/Foxglove
 * Studio for automatic rendering; `navtracker.Diag` is a custom flat-scalar
 * diagnostic schema plotted by field path. `kRootFrame` is the ENU frame id
 * every transform and pose is expressed against.
 */
// Foxglove well-known schema names (recognized by Lichtblick for auto-render).
inline constexpr const char* kSceneUpdateSchema  = "foxglove.SceneUpdate";
inline constexpr const char* kLocationFixSchema  = "foxglove.LocationFix";
inline constexpr const char* kFrameTransformSchema = "foxglove.FrameTransform";
inline constexpr const char* kLogSchema          = "foxglove.Log";
// Custom diagnostic schema name (flat scalar object, plotted by field path).
inline constexpr const char* kDiagSchema         = "navtracker.Diag";
inline constexpr const char* kRootFrame          = "enu";
}  // namespace navtracker::foxglove
