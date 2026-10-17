#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <cstdlib>
#include "mex.h"
#include "../../cpp/extensions/Environment.h"
void mexFunction(int outputs, mxArray** result, int, const mxArray**) {
  const char* crt = std::getenv("G1_WINDOWS_ENV_PROBE");
  char native[256]{};
  GetEnvironmentVariableA("G1_WINDOWS_ENV_PROBE",native,sizeof(native));
  if (outputs > 0) result[0] = mxCreateString(crt ? crt : "");
  if (outputs > 1) result[1] = mxCreateString(native);
  if (outputs > 2) result[2] = mxCreateString(g1_platform::environmentVariable("G1_WINDOWS_ENV_PROBE").c_str());
}
