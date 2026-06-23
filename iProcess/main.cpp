#include <iostream>
#include "irsdk_defines.h"
#include "irsdk_client.h"
#include "irsdk_diskclient.h"
#include "yaml_parser.h"
#include "iProcess_SLP.h"
#include <chrono>
#include <thread>

int main() {
	//temporary; only testing
	irsdkCVar g_CurrAltitude("Alt");
	irsdkCVar g_CurrVelocityX("VelocityX");
	irsdkCVar g_CurrLapTime("LapCurrentLapTime");
	while (true) {
		irsdkClient::instance().waitForData();
		std::cout << g_CurrLapTime.getFloat() << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	return 0;
}