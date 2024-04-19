/* controller.cpp
 * author: nikobk
 * created: apr 8 2024
*/
#include "../include/controller.hpp"
#include <unistd.h>
#include <fstream>
#include "../include/socketserver.hpp"
#include "../include/debug.hpp"

int main() {
	loadConfig("config.xml");
	log("Stopping drone controller application...");
	return 0;
}

void loadConfig(const char* path) {
	log("Loading config [TBA]");
	// TODO: Parse this from actual XML or something.
	DroneConfig cfg;
	cfg.SvrAddr = "127.0.0.1"; // NOTE: Not actually being used now.
	cfg.Port = 8080;

	// Start socketserver.
	startServer(cfg.Port);
}
