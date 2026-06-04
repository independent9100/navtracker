#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include <gtest/gtest.h>

#include "adapters/own_ship/OwnShipNmeaAdapter.hpp"
#include "adapters/own_ship/OwnShipProvider.hpp"
#include "core/bias/HeadingBiasEstimator.hpp"
#include "core/types/Timestamp.hpp"

using namespace navtracker;

namespace {

std::string makeNmea(const std::string& payload) {
  std::uint8_t cs = 0;
  for (char c : payload) cs ^= static_cast<std::uint8_t>(c);
  char buf[8];
  std::snprintf(buf, sizeof(buf), "*%02X", cs);
  return "$" + payload + buf;
}

Timestamp at(double s) { return Timestamp::fromSeconds(s); }

}  // namespace

TEST(NmeaBiasDispatch, HdgDispatchesMagneticAfterGyro) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  HeadingBiasEstimator est({});
  adapter.setHeadingBiasEstimator(&est);
  adapter.ingest(makeNmea("IIHDT,100.0,T"), at(1.0));
  adapter.ingest(makeNmea("IIHDG,97.0,0.0,E,3.0,E"), at(1.1));
  EXPECT_EQ(est.acceptedMagnetic(), 1u);
  EXPECT_EQ(adapter.dispatchedMagnetic(), 1u);
}

TEST(NmeaBiasDispatch, HdgWithoutGyroSkipsAndCounts) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  HeadingBiasEstimator est({});
  adapter.setHeadingBiasEstimator(&est);
  adapter.ingest(makeNmea("IIHDG,97.0,0.0,E,3.0,E"), at(1.0));
  EXPECT_EQ(est.acceptedMagnetic(), 0u);
  EXPECT_EQ(adapter.skippedGyroStale(), 1u);
}

TEST(NmeaBiasDispatch, HdgWithoutVariationAndNoCacheSkips) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  HeadingBiasEstimator est({});
  adapter.setHeadingBiasEstimator(&est);
  adapter.ingest(makeNmea("IIHDT,100.0,T"), at(1.0));
  adapter.ingest(makeNmea("IIHDG,97.0,,,,"), at(1.1));
  EXPECT_EQ(est.acceptedMagnetic(), 0u);
  EXPECT_EQ(adapter.skippedMagNoVariation(), 1u);
}

TEST(NmeaBiasDispatch, RmcVariationCachedAndUsedByLaterHdg) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  HeadingBiasEstimator est({});
  adapter.setHeadingBiasEstimator(&est);
  adapter.ingest(makeNmea("IIHDT,100.0,T"), at(1.0));
  adapter.ingest(
      makeNmea("GPRMC,000000,A,0000.00,N,00000.00,E,5.0,0.0,010100,3.0,E,A"),
      at(1.5));
  adapter.ingest(makeNmea("IIHDG,97.0,,,,"), at(2.0));
  EXPECT_EQ(est.acceptedMagnetic(), 1u);
  EXPECT_EQ(adapter.skippedMagNoVariation(), 0u);
}

TEST(NmeaBiasDispatch, RmcDispatchesCogWhenGyroFresh) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  HeadingBiasEstimator est({});
  adapter.setHeadingBiasEstimator(&est);
  adapter.ingest(makeNmea("IIHDT,100.0,T"), at(1.0));
  adapter.ingest(
      makeNmea("GPRMC,000000,A,0000.00,N,00000.00,E,9.72,100.0,010100,0.0,E,A"),
      at(1.5));
  EXPECT_EQ(est.acceptedGpsCog(), 1u);
  EXPECT_EQ(adapter.dispatchedGpsCog(), 1u);
}

TEST(NmeaBiasDispatch, RmcLowSogRejectedByEstimatorButStillDispatched) {
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider);
  HeadingBiasEstimator est({});
  adapter.setHeadingBiasEstimator(&est);
  adapter.ingest(makeNmea("IIHDT,100.0,T"), at(1.0));
  adapter.ingest(
      makeNmea("GPRMC,000000,A,0000.00,N,00000.00,E,1.0,100.0,010100,0.0,E,A"),
      at(1.5));
  EXPECT_EQ(adapter.dispatchedGpsCog(), 1u);
  EXPECT_EQ(est.acceptedGpsCog(), 0u);
  EXPECT_EQ(est.rejectedCogBySog(), 1u);
}

TEST(NmeaBiasDispatch, GpsHdtRoutedAndDispatchedWhenConfigured) {
  OwnShipNmeaAdapterConfig cfg;
  cfg.gps_heading_talkers = {"GP"};
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider, cfg);
  HeadingBiasEstimator est({});
  adapter.setHeadingBiasEstimator(&est);
  adapter.ingest(makeNmea("IIHDT,100.0,T"), at(1.0));
  adapter.ingest(makeNmea("GPHDT,99.0,T"), at(1.1));
  EXPECT_EQ(est.acceptedGpsHeading(), 1u);
  EXPECT_EQ(adapter.dispatchedGpsHeading(), 1u);
}

TEST(NmeaBiasDispatch, GpsHdtWithoutGyroSkipsStale) {
  OwnShipNmeaAdapterConfig cfg;
  cfg.gps_heading_talkers = {"GP"};
  OwnShipProvider provider;
  OwnShipNmeaAdapter adapter(provider, cfg);
  HeadingBiasEstimator est({});
  adapter.setHeadingBiasEstimator(&est);
  adapter.ingest(makeNmea("GPHDT,99.0,T"), at(1.0));
  EXPECT_EQ(est.acceptedGpsHeading(), 0u);
  EXPECT_EQ(adapter.skippedGyroStale(), 1u);
}
