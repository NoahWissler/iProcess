#pragma once

struct SLP_data {
	int Gear;
	int RPM;
	int Lap;
	float LapCurrentLapTime;
	float LongAccel;
	float Speed;
	float LapDist;
};

double RPM_climbrate_preshift(SLP_data atshift, SLP_data preshift);

double t_hold(int RPM_opt, double RPM_climbrate_preshift, int RPM_atshift); // amount of time driver should've stayed in previous gear

double speed_advantage(float LongAccel_atshift, float LongAccel_preshift, double t_hold);

double laptime_loss(float LapDist_atshift, float LapDist_atbrakelift, float TrackLength, int Lap_atshift, int Lap_atbrakelift, float Speed_atshift, double speed_advantage);

void run();