#pragma once
#include <cstddef>

namespace net_tcp {

bool init(int port);
// Accept a new client if one is waiting. Returns true if connected.
bool accept_client();
// Returns true once (edge-triggered) after a new client connects.
bool client_just_connected();
// True if there is currently an active connection.
bool is_connected();
// True if data is available to read (non-blocking peek).
bool data_available();
// Read one byte. Returns -1 if none available.
int  read_byte();
bool send(const char* data, size_t len);
int  readline(char* buf, int max_len);
void shutdown();

}  // namespace net_tcp
