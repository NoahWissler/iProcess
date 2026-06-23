// IProcess Shift Loss Processing
#include "iProcess_SLP.h"
#include "irsdk_client.h"

double RPM_climbrate_preshift(SLP_data atshift, SLP_data preshift) {
	double RPM_climbrate_preshift{};
	if (atshift.Lap == preshift.Lap) {
		RPM_climbrate_preshift = (atshift.RPM - preshift.RPM) / (atshift.LapCurrentLapTime - preshift.LapCurrentLapTime);
	}
	else {
		irsdkClient::instance().waitForData();
		irsdkCVar g_lastLapTime("LapLastLapTime");
		float lastLapTime = g_lastLapTime.getFloat();
		RPM_climbrate_preshift = double(atshift.RPM - preshift.RPM) / ((atshift.LapCurrentLapTime + lastLapTime)- preshift.LapCurrentLapTime);
	}
		return RPM_climbrate_preshift;
}

double t_hold(int RPM_opt, double RPM_climbrate_preshift, int RPM_atshift) {
	return double(RPM_opt - RPM_atshift) / RPM_climbrate_preshift;
}

double speed_advantage(float LongAccel_atshift, float LongAccel_preshift, double t_hold) {
	return (LongAccel_preshift - LongAccel_atshift) * t_hold;
}

double laptime_loss(float LapDist_atshift, float LapDist_atbrakelift, float TrackLength,int Lap_atshift, int Lap_atbrakelift, float Speed_atshift, double speed_advantage) {
	if (LapDist_atshift == LapDist_atbrakelift) {
		return (LapDist_atbrakelift - LapDist_atshift)* ((1 / Speed_atshift) - (1 / (Speed_atshift + speed_advantage)));
	}
	else {
		return ((LapDist_atbrakelift + TrackLength) - LapDist_atshift) * ((1 / Speed_atshift) - (1 / (Speed_atshift + speed_advantage)));
	}
}

void run() {

}