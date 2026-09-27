// zeliboba - headless entry point.
//
// The console frontend lives in src/debug/cli.cpp; the SDL3 frontend
// (src/ui) is a separate executable that shares the same core library.
#include "debug/cli.h"

int main(int argc, char** argv) { return zlb::cli_main(argc, argv); }
