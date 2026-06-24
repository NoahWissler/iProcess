#include <iostream>
#include "irsdk_defines.h"
#include "irsdk_client.h"
#include "irsdk_diskclient.h"
#include "yaml_parser.h"
#include "iProcess_SLP.h"
#include <chrono>
#include <thread>
#include <string>

int main() {
	irsdkCVar g_Gear("Gear");
	irsdkCVar g_RPM("RPM");
	irsdkCVar g_Lap("Lap");
	irsdkCVar g_LapCurrentLapTime("LapCurrentLapTime");
	irsdkCVar g_LongAccel("LongAccel");
	irsdkCVar g_Speed("Speed");
	irsdkCVar g_LapDist("LapDist");

	irsdkClient::instance().waitForData();
	const char* sessionStr = irsdk_getSessionInfoStr();
	const char* value = NULL;
	int len = 0;
	double TrackLength;
	if (parseYaml(sessionStr, "TrackLength:", &value, &len)) {
		char buffer[64] = { 0 };
		memcpy(buffer, value, len);
		TrackLength = atof(buffer);
	}

	std::vector<SLP_data> data_points;
	while (true) {
		irsdkClient::instance().waitForData();
		SLP_data data = { g_Gear.getInt(), g_RPM.getInt(), g_Lap.getInt(), g_Lap.getFloat(), g_LongAccel.getFloat(), g_Speed.getFloat(), g_LapDist.getFloat() };
		data_points.push_back(data);
		if (data_points.size() > 1) {
			if (data_points[data_points.size() - 1].Gear > 1 && data_points[data_points.size() - 1].Gear > data_points[data_points.size() - 2].Gear) {
				
			}
		}
	}
	return 0;
}