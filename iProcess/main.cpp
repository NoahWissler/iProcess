#include <iostream>
#include "irsdk_defines.h"
#include "irsdk_client.h"
#include "irsdk_diskclient.h"
#include "yaml_parser.h"
#include "iProcess_SLP.h"
#include <chrono>
#include <thread>
#include <string>
#include <optional>
#include <deque>

// Number of samples held in the rolling pre-shift window used to smooth LongAccel.
static constexpr int PRE_SHIFT_WINDOW = 5;

// LongAccel threshold (m/s^2, negative = braking) that marks entry into a brake/lift zone.
static constexpr float BRAKE_ACCEL_THRESHOLD = -1.5f;

int main() {
	irsdkCVar g_Gear("Gear");
	irsdkCVar g_RPM("RPM");
	irsdkCVar g_Lap("Lap");
	irsdkCVar g_LapCurrentLapTime("LapCurrentLapTime");
	irsdkCVar g_LongAccel("LongAccel");
	irsdkCVar g_Speed("Speed");
	irsdkCVar g_LapDist("LapDist");
	irsdkCVar g_LastLapTime("LapLastLapTime");

	irsdkClient::instance().waitForData();
	const char* sessionStr = irsdk_getSessionInfoStr();
	const char* value = NULL;
	int len = 0;
	float TrackLength = 0.0f;
	if (parseYaml(sessionStr, "TrackLength:", &value, &len)) {
		char buffer[64] = { 0 };
		memcpy(buffer, value, len);
		TrackLength = (float)atof(buffer);
	}

	// Rolling window of recent samples — used for the pre-shift LongAccel average.
	std::deque<SLP_data> window;

	// Stashed shift event waiting for the next brake/lift zone.
	std::optional<PendingShift> pendingShift;

	// Cached last-lap time, refreshed whenever the lap counter increments.
	float lastLapTime = 0.0f;
	int   prevLap     = -1;

	while (true) {
		irsdkClient::instance().waitForData();

		SLP_data data = {
			g_Gear.getInt(),
			g_RPM.getInt(),
			g_Lap.getInt(),
			g_LapCurrentLapTime.getFloat(),   // was incorrectly g_Lap.getFloat()
			g_LongAccel.getFloat(),
			g_Speed.getFloat(),
			g_LapDist.getFloat()
		};

		// Refresh lastLapTime on lap boundary so cross-lap RPM climb rate is accurate.
		if (data.Lap != prevLap) {
			lastLapTime = g_LastLapTime.getFloat();
			prevLap = data.Lap;
		}

		// Maintain rolling window (drop oldest when full).
		window.push_back(data);
		if ((int)window.size() > PRE_SHIFT_WINDOW)
			window.pop_front();

		if (window.size() > 1) {
			const SLP_data& prev = window[window.size() - 2];
			const SLP_data& curr = window.back();

			// ── Upshift detected ─────────────────────────────────────────────────
			if (curr.Gear > 1 && curr.Gear > prev.Gear) {
				// Build rolling average of LongAccel over the pre-shift window.
				float accelSum = 0.0f;
				for (const auto& s : window)
					accelSum += s.LongAccel;
				float avgAccel = accelSum / (float)window.size();

				// The oldest sample in the window is the start of the pre-shift period.
				const SLP_data& oldest = window.front();

				PendingShift ps;
				ps.atShift              = curr;
				ps.avgLongAccel_preShift = avgAccel;
				ps.RPM_preShift         = oldest.RPM;
				ps.LapTime_preShift     = oldest.LapCurrentLapTime;
				ps.Lap_preShift         = oldest.Lap;
				pendingShift            = ps;
			}

			// ── Brake / lift zone detected ────────────────────────────────────────
			if (pendingShift.has_value() && curr.LongAccel < BRAKE_ACCEL_THRESHOLD) {
				const PendingShift& ps = *pendingShift;

				int RPM_opt = load_RPM_opt(ps.atShift.Gear - 1);   // gear before the shift
				if (RPM_opt > 0) {
					double climb = RPM_climbrate_preshift(
						ps.atShift,
						ps.RPM_preShift,
						ps.LapTime_preShift,
						ps.Lap_preShift,
						lastLapTime);

					double hold = t_hold(RPM_opt, climb, ps.atShift.RPM);

					double dv = speed_advantage(
						ps.atShift.LongAccel,
						ps.avgLongAccel_preShift,
						hold);

					double loss = laptime_loss(
						ps.atShift.LapDist,
						curr.LapDist,
						TrackLength,
						ps.atShift.Lap,
						curr.Lap,
						ps.atShift.Speed,
						dv);

					std::cout << "Gear " << (ps.atShift.Gear - 1) << "->" << ps.atShift.Gear
					          << "  early by " << hold * 1000.0 << " ms"
					          << "  lap time loss: " << loss * 1000.0 << " ms\n";
				}

				pendingShift.reset();
			}
		}
	}
	return 0;
}