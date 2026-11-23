// E1 Zenoh link: hooks called from CF-Drone (inserted by tools/prepare_src.py).
#pragma once
#include <Arduino.h>

void setupZenohLink();                                          // setup(): start the Zenoh task on core 0
void zenohLinkApply();                                          // loop(): apply received commands
void zenohLinkSnapshot();                                       // loop(): copy telemetry for the Zenoh task
void zenohLinkLog(const char *text);                            // print(): mirror console to e1/<id>/log
bool zenohLinkCommand(String &command, String &arg0, String &arg1);  // cli: "zn ..." commands
