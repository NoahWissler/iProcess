#pragma once
#include <optional>
#include <deque>

struct SLP_data {
	int Gear;
	int RPM;
	int Lap;
	float LapCurrentLapTime;
	float LongAccel;
	float Speed;
	float LapDist;
};

// Stores everything needed to finish the loss calculation once the next brake/lift zone arrives.
struct PendingShift {
	SLP_data atShift;
	float    avgLongAccel_preShift;  // rolling-average LongAccel over pre-shift window
	int      RPM_preShift;           // RPM at start of pre-shift window
	float    LapTime_preShift;       // LapCurrentLapTime at start of pre-shift window
	int      Lap_preShift;
};

// Reads RPM_opt for a given gear from slp_config.txt (e.g. "gear2=7500").
// Returns -1 if the key is not found.
int    load_RPM_opt(int gear);

// lastLapTime is passed in so this stays a pure calculation (no irsdk I/O).
double RPM_climbrate_preshift(SLP_data atshift, int RPM_preshift, float LapTime_preshift, int Lap_preshift, float lastLapTime);

// Returns time (s) the driver should have held the previous gear.
// Clamped to >= 0: an over-rev shift cannot yield a negative hold time.
double t_hold(int RPM_opt, double RPM_climbrate, int RPM_atshift);

// Δv = (avgAccel_pre - Accel_atShift) * t_hold
double speed_advantage(float LongAccel_atshift, float avgLongAccel_preshift, double t_hold);

// Lap time lost over the zone from the shift point to the next brake/lift point.
// Uses Lap numbers (not LapDist) to detect cross-lap-boundary cases.
// Returns 0 if Speed_atshift is invalid.
double laptime_loss(float LapDist_atshift, float LapDist_atbrakelift, float TrackLength, int Lap_atshift, int Lap_atbrakelift, float Speed_atshift, double speed_advantage);