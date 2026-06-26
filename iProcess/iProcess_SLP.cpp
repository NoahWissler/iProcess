// IProcess Shift Loss Processing
#include "iProcess_SLP.h"
#include <cstdio>
#include <cstring>

// ---------------------------------------------------------------------------
// Config loader
// ---------------------------------------------------------------------------

// Reads slp_config.txt from the working directory.
// Expected format (one entry per line):  gear<N>=<RPM>
// Example:
//   gear2=7200
//   gear3=7500
//   gear4=7500
// Returns -1 if the key is not present.
int load_RPM_opt(int gear) {
	FILE* f = nullptr;
	if (fopen_s(&f, "slp_config.txt", "r") != 0 || !f)
		return -1;

	char line[64];
	char key[32];
	snprintf(key, sizeof(key), "gear%d=", gear);

	int result = -1;
	while (fgets(line, sizeof(line), f)) {
		if (strncmp(line, key, strlen(key)) == 0) {
			result = atoi(line + strlen(key));
			break;
		}
	}
	fclose(f);
	return result;
}

// ---------------------------------------------------------------------------
// Pure calculation functions
// ---------------------------------------------------------------------------

// RPM climb rate (RPM/s) measured over the pre-shift window.
// lastLapTime must be supplied by the caller when atshift and the pre-shift
// sample are on different laps — keeps this function free of irsdk I/O.
double RPM_climbrate_preshift(SLP_data atshift, int RPM_preshift, float LapTime_preshift, int Lap_preshift, float lastLapTime) {
	double dt;
	if (atshift.Lap == Lap_preshift) {
		dt = atshift.LapCurrentLapTime - LapTime_preshift;
	}
	else {
		// Pre-shift sample was on the previous lap; stitch lap times together.
		dt = (atshift.LapCurrentLapTime + lastLapTime) - LapTime_preshift;
	}
	if (dt <= 0.0)
		return 0.0;
	return double(atshift.RPM - RPM_preshift) / dt;
}

// Time (s) the driver should have stayed in the lower gear.
// Clamped to >= 0: if the driver already over-revved past RPM_opt there is
// no recoverable hold time — the loss calculation should still run but
// with a zero advantage rather than a phantom gain.
double t_hold(int RPM_opt, double RPM_climbrate, int RPM_atshift) {
	if (RPM_climbrate <= 0.0)
		return 0.0;
	double t = double(RPM_opt - RPM_atshift) / RPM_climbrate;
	return t > 0.0 ? t : 0.0;
}

// Speed advantage (m/s) gained by holding the lower gear for t_hold extra seconds.
// Uses the difference between the rolling-average pre-shift acceleration and the
// instantaneous acceleration at the shift point.
double speed_advantage(float LongAccel_atshift, float avgLongAccel_preshift, double t_hold) {
	return (avgLongAccel_preshift - LongAccel_atshift) * t_hold;
}

// Lap time lost between the shift point and the next brake/lift zone.
// Branches on Lap number (not LapDist) to correctly handle the case where
// the brake zone is on the following lap.
// Guard against divide-by-zero on invalid speed values.
double laptime_loss(float LapDist_atshift, float LapDist_atbrakelift, float TrackLength, int Lap_atshift, int Lap_atbrakelift, float Speed_atshift, double speed_advantage) {
	double v1 = Speed_atshift;
	double v2 = Speed_atshift + speed_advantage;
	if (v1 <= 0.0 || v2 <= 0.0)
		return 0.0;

	double dist;
	if (Lap_atshift == Lap_atbrakelift) {
		dist = LapDist_atbrakelift - LapDist_atshift;
	}
	else {
		// Brake zone is on the next lap — wrap around via TrackLength.
		dist = (LapDist_atbrakelift + TrackLength) - LapDist_atshift;
	}
	if (dist <= 0.0)
		return 0.0;

	return dist * ((1.0 / v1) - (1.0 / v2));
}