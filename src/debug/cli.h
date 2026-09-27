// zeliboba - headless frontend: the interactive debugger console.
#pragma once

namespace zlb {

/// Entry point shared by `zeliboba` (console) and used as a fallback by the SDL3
/// frontend when it is started with --headless.
int cli_main(int argc, char** argv);

}  // namespace zlb
