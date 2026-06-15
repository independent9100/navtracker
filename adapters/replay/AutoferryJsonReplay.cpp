#include "adapters/replay/AutoferryJsonReplay.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "core/types/Ids.hpp"
#include "core/types/Measurement.hpp"

namespace navtracker::replay {
namespace {

// --- Minimal recursive-descent JSON parser -------------------------------
//
// Scoped to exactly the subset the AutoFerry files use: objects, arrays,
// numbers (incl. scientific notation), strings, and the literals
// true/false/null. We hand-roll this rather than add a third-party JSON
// dependency — the project deliberately depends only on Eigen and gtest,
// and the build sandbox cannot reach a package index. The parser is
// allocation-light (values share a tagged union) and tolerant: on any
// malformed input it leaves `ok=false` and the caller falls back to an
// empty Scenario.

struct JValue {
  enum class Type { Null, Bool, Number, String, Array, Object } type{Type::Null};
  double number{0.0};
  bool boolean{false};
  std::string str;
  std::vector<JValue> arr;
  std::map<std::string, JValue> obj;

  bool isArray() const { return type == Type::Array; }
  bool isNumber() const { return type == Type::Number; }
  const JValue* find(const std::string& k) const {
    auto it = obj.find(k);
    return it == obj.end() ? nullptr : &it->second;
  }
};

class Parser {
 public:
  explicit Parser(const std::string& s) : s_(s) {}

  bool parse(JValue& out) {
    skipWs();
    if (!parseValue(out)) return false;
    skipWs();
    return true;  // trailing content tolerated
  }

 private:
  const std::string& s_;
  std::size_t i_{0};

  void skipWs() {
    while (i_ < s_.size() &&
           (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\n' || s_[i_] == '\r'))
      ++i_;
  }

  bool parseValue(JValue& v) {
    skipWs();
    if (i_ >= s_.size()) return false;
    const char c = s_[i_];
    switch (c) {
      case '{': return parseObject(v);
      case '[': return parseArray(v);
      case '"': {
        v.type = JValue::Type::String;
        return parseString(v.str);
      }
      case 't': case 'f': return parseBool(v);
      case 'n': return parseNull(v);
      default:  return parseNumber(v);
    }
  }

  bool parseObject(JValue& v) {
    v.type = JValue::Type::Object;
    ++i_;  // '{'
    skipWs();
    if (i_ < s_.size() && s_[i_] == '}') { ++i_; return true; }
    while (true) {
      skipWs();
      if (i_ >= s_.size() || s_[i_] != '"') return false;
      std::string key;
      if (!parseString(key)) return false;
      skipWs();
      if (i_ >= s_.size() || s_[i_] != ':') return false;
      ++i_;
      JValue child;
      if (!parseValue(child)) return false;
      v.obj.emplace(std::move(key), std::move(child));
      skipWs();
      if (i_ >= s_.size()) return false;
      if (s_[i_] == ',') { ++i_; continue; }
      if (s_[i_] == '}') { ++i_; return true; }
      return false;
    }
  }

  bool parseArray(JValue& v) {
    v.type = JValue::Type::Array;
    ++i_;  // '['
    skipWs();
    if (i_ < s_.size() && s_[i_] == ']') { ++i_; return true; }
    while (true) {
      JValue child;
      if (!parseValue(child)) return false;
      v.arr.push_back(std::move(child));
      skipWs();
      if (i_ >= s_.size()) return false;
      if (s_[i_] == ',') { ++i_; continue; }
      if (s_[i_] == ']') { ++i_; return true; }
      return false;
    }
  }

  bool parseString(std::string& out) {
    if (i_ >= s_.size() || s_[i_] != '"') return false;
    ++i_;  // opening quote
    std::string r;
    while (i_ < s_.size()) {
      const char c = s_[i_++];
      if (c == '"') { out = std::move(r); return true; }
      if (c == '\\') {
        if (i_ >= s_.size()) return false;
        const char e = s_[i_++];
        switch (e) {
          case 'n': r.push_back('\n'); break;
          case 't': r.push_back('\t'); break;
          case 'r': r.push_back('\r'); break;
          case 'b': r.push_back('\b'); break;
          case 'f': r.push_back('\f'); break;
          case '/': r.push_back('/'); break;
          case '\\': r.push_back('\\'); break;
          case '"': r.push_back('"'); break;
          case 'u': i_ += 4; r.push_back('?'); break;  // not used by dataset
          default: r.push_back(e); break;
        }
      } else {
        r.push_back(c);
      }
    }
    return false;  // unterminated
  }

  bool parseNumber(JValue& v) {
    const std::size_t start = i_;
    while (i_ < s_.size()) {
      const char c = s_[i_];
      if (std::isdigit(static_cast<unsigned char>(c)) || c == '+' || c == '-' ||
          c == '.' || c == 'e' || c == 'E') {
        ++i_;
      } else {
        break;
      }
    }
    if (i_ == start) return false;
    v.type = JValue::Type::Number;
    v.number = std::strtod(s_.substr(start, i_ - start).c_str(), nullptr);
    return true;
  }

  bool parseBool(JValue& v) {
    if (s_.compare(i_, 4, "true") == 0) { i_ += 4; v.type = JValue::Type::Bool; v.boolean = true; return true; }
    if (s_.compare(i_, 5, "false") == 0) { i_ += 5; v.type = JValue::Type::Bool; v.boolean = false; return true; }
    return false;
  }

  bool parseNull(JValue& v) {
    if (s_.compare(i_, 4, "null") == 0) { i_ += 4; v.type = JValue::Type::Null; return true; }
    return false;
  }
};

bool readFile(const std::string& path, std::string& out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::ostringstream ss;
  ss << f.rdbuf();
  out = ss.str();
  return !out.empty();
}

// Extract the (north, east) points carried by a Lidar/Radar `measurement`
// node. Accepts the 2×M nested form `[[n..],[e..]]` and the flat M==1 form
// `[n, e]`. Returns nothing for an empty array or unexpected shapes.
std::vector<std::pair<double, double>> activePoints(const JValue& meas) {
  std::vector<std::pair<double, double>> out;
  if (!meas.isArray() || meas.arr.empty()) return out;
  if (meas.arr[0].isArray()) {
    // 2×M: row 0 norths, row 1 easts.
    if (meas.arr.size() < 2) return out;
    const auto& norths = meas.arr[0].arr;
    const auto& easts = meas.arr[1].arr;
    const std::size_t m = std::min(norths.size(), easts.size());
    for (std::size_t k = 0; k < m; ++k) {
      if (norths[k].isNumber() && easts[k].isNumber())
        out.emplace_back(norths[k].number, easts[k].number);
    }
  } else if (meas.arr.size() >= 2 && meas.arr[0].isNumber() &&
             meas.arr[1].isNumber()) {
    out.emplace_back(meas.arr[0].number, meas.arr[1].number);
  }
  return out;
}

// Extract bearings (radians, NED atan2(e,n)) from an IR/EO `measurement`
// node, which is a flat 1×M list or a bare scalar.
std::vector<double> bearingValues(const JValue& meas) {
  std::vector<double> out;
  if (meas.isNumber()) {
    out.push_back(meas.number);
  } else if (meas.isArray()) {
    for (const auto& b : meas.arr)
      if (b.isNumber()) out.push_back(b.number);
  }
  return out;
}

}  // namespace

Scenario loadAutoferryScenario(const std::string& dir,
                               const std::string& label,
                               const AutoferryLoadOptions& opts) {
  Scenario empty;
  const std::string det_path = dir + "/" + label + "_detections.json";
  const std::string gt_path = dir + "/" + label + "_groundTruth.json";

  std::string det_raw, gt_raw;
  if (!readFile(det_path, det_raw) || !readFile(gt_path, gt_raw)) return empty;

  JValue det_root, gt_root;
  if (!Parser(det_raw).parse(det_root) || !det_root.isArray()) return empty;
  if (!Parser(gt_raw).parse(gt_root) || !gt_root.isArray()) return empty;

  Scenario s;

  // --- Measurements (detections) ---
  for (const JValue& d : det_root.arr) {
    const JValue* sid = d.find("sensorID");
    const JValue* os = d.find("ownshipPosition");
    const JValue* tm = d.find("time");
    const JValue* meas = d.find("measurement");
    if (!sid || !sid->isNumber() || !tm || !tm->isNumber() || !meas) continue;
    if (!os || !os->isArray() || os->arr.size() < 2) continue;
    const double os_n = os->arr[0].number;  // ownship north
    const double os_e = os->arr[1].number;  // ownship east
    const Timestamp t = Timestamp::fromSeconds(tm->number);
    const int s_id = static_cast<int>(std::lround(sid->number));

    if (s_id == 1 || s_id == 2) {
      // Lidar / Radar → absolute ENU position (E, N) = ownship + (n, e).
      const bool is_lidar = (s_id == 1);
      const double std_m = is_lidar ? opts.lidar_pos_std_m : opts.radar_pos_std_m;
      for (const auto& [mn, me] : activePoints(*meas)) {
        Measurement m;
        m.time = t;
        m.sensor = is_lidar ? SensorKind::Lidar : SensorKind::ArpaTtm;
        m.source_id = is_lidar ? "autoferry_lidar" : "autoferry_radar";
        m.model = MeasurementModel::Position2D;
        m.value = Eigen::Vector2d(os_e + me, os_n + mn);  // (E, N)
        m.covariance = Eigen::Matrix2d::Identity() * (std_m * std_m);
        // Ownship carries the sensor: needed downstream to range-
        // condition the per-sensor miss P_D (lidar coverage ~140 m).
        m.sensor_position_enu = Eigen::Vector2d(os_e, os_n);
        s.measurements.push_back(std::move(m));
      }
    } else if ((s_id == 3 || s_id == 4) && opts.include_bearings) {
      // IR / EO → bearing about the ownship position. Dataset bearing is
      // NED atan2(e,n) from north; our Bearing2D is ENU atan2(dy,dx) from
      // east, so θ_enu = π/2 − θ_ned. sensor_position_enu = (E, N).
      const bool is_ir = (s_id == 3);
      for (const double b_ned : bearingValues(*meas)) {
        Measurement m;
        m.time = t;
        m.sensor = SensorKind::EoIr;
        m.source_id = is_ir ? "autoferry_ir" : "autoferry_eo";
        m.model = MeasurementModel::Bearing2D;
        Eigen::VectorXd v(1);
        v(0) = M_PI / 2.0 - b_ned;
        m.value = v;
        Eigen::MatrixXd r(1, 1);
        r(0, 0) = opts.bearing_std_rad * opts.bearing_std_rad;
        m.covariance = r;
        m.sensor_position_enu = Eigen::Vector2d(os_e, os_n);
        s.measurements.push_back(std::move(m));
      }
    }
  }

  // Detections arrive time-ordered in the file, but a defensive stable
  // sort guarantees the engine sees monotonic timestamps (time-driven
  // invariant) and keeps replay deterministic regardless of file order.
  std::stable_sort(s.measurements.begin(), s.measurements.end(),
                   [](const Measurement& a, const Measurement& b) {
                     return a.time < b.time;
                   });

  // --- Ground truth (per detection-index scan of per-target objects) ---
  //
  // The dataset stamps every target with its OWN time inside a scan
  // (skews ~0.1 s). Passed through verbatim, those per-target times
  // fragment one 2-target scan into two 1-target evaluation steps in
  // BenchRunner::groupTruth (which buckets on exact time), wrecking
  // every cardinality/identity metric. One scan = one evaluation
  // instant: all of a scan's samples are stamped with the scan's latest
  // target time (≤0.1 s labelling skew ≈ <1 m at harbour speeds —
  // negligible against sensor noise).
  for (const JValue& scan : gt_root.arr) {
    if (!scan.isArray()) continue;
    const std::size_t scan_begin = s.truth.size();
    double scan_t = -std::numeric_limits<double>::infinity();
    for (const JValue& tgt : scan.arr) {
      const JValue* id = tgt.find("targetID");
      const JValue* pos = tgt.find("position");
      const JValue* tm = tgt.find("time");
      if (!id || !pos || !tm || !pos->isArray() || pos->arr.size() < 2) continue;
      TruthSample ts;
      ts.truth_id = static_cast<std::uint64_t>(std::lround(id->number));
      ts.position = Eigen::Vector2d(pos->arr[1].number, pos->arr[0].number);  // (E,N)
      scan_t = std::max(scan_t, tm->number);
      s.truth.push_back(ts);
    }
    for (std::size_t i = scan_begin; i < s.truth.size(); ++i)
      s.truth[i].time = Timestamp::fromSeconds(scan_t);
  }

  // Scans should already be file-ordered by time; guarantee the sorted-
  // by-time precondition of BenchRunner::groupTruth either way. The
  // dataset also repeats the previous truth scan verbatim when no new
  // ground-truth fix arrived between detection events — sort by
  // (time, id) and drop those exact duplicates so one evaluation step
  // sees each target once.
  std::sort(s.truth.begin(), s.truth.end(),
            [](const TruthSample& a, const TruthSample& b) {
              if (a.time < b.time) return true;
              if (b.time < a.time) return false;
              return a.truth_id < b.truth_id;
            });
  s.truth.erase(std::unique(s.truth.begin(), s.truth.end(),
                            [](const TruthSample& a, const TruthSample& b) {
                              return a.time == b.time &&
                                     a.truth_id == b.truth_id;
                            }),
                s.truth.end());

  // Optional: inject synthetic AIS-style anchor measurements per
  // (target, truth scan) from the deduplicated ground truth. One
  // Measurement per TruthSample, σ = truth_anchor_std_m (1-sigma per
  // axis, isotropic) — analogous to RTK-GNSS in the paper's
  // calibration setup. Re-sort the measurement vector to preserve
  // BenchRunner's monotonic-timestamp precondition.
  if (opts.inject_truth_anchor) {
    const double v =
        opts.truth_anchor_std_m * opts.truth_anchor_std_m;
    for (const TruthSample& ts : s.truth) {
      Measurement m;
      m.time = ts.time;
      m.sensor = SensorKind::Ais;
      m.source_id = "autoferry_truth_anchor";
      m.model = MeasurementModel::Position2D;
      m.value = ts.position;
      m.covariance = (Eigen::Matrix2d() << v, 0.0, 0.0, v).finished();
      m.hints.mmsi = static_cast<std::uint32_t>(ts.truth_id);
      s.measurements.push_back(std::move(m));
    }
    std::stable_sort(s.measurements.begin(), s.measurements.end(),
                     [](const Measurement& a, const Measurement& b) {
                       return a.time < b.time;
                     });
  }

  // The dataset carries positions only; derive per-target velocity by
  // finite differences so SOG/COG metrics compare against real target
  // kinematics instead of the zero vector. Central difference at
  // interior samples, one-sided at the ends, zero for singletons.
  {
    std::map<std::uint64_t, std::vector<std::size_t>> by_target;
    for (std::size_t i = 0; i < s.truth.size(); ++i)
      by_target[s.truth[i].truth_id].push_back(i);
    for (const auto& [tid, idx] : by_target) {
      const std::size_t n = idx.size();
      for (std::size_t k = 0; k < n; ++k) {
        const std::size_t lo = idx[k > 0 ? k - 1 : k];
        const std::size_t hi = idx[k + 1 < n ? k + 1 : k];
        if (lo == hi) continue;  // singleton: velocity stays zero
        const double dt =
            s.truth[hi].time.secondsSince(s.truth[lo].time);
        if (dt <= 0.0) continue;
        s.truth[idx[k]].velocity =
            (s.truth[hi].position - s.truth[lo].position) / dt;
      }
    }
  }

  return s;
}

}  // namespace navtracker::replay
