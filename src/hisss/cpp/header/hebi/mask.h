#pragma once

extern "C" {
// Evaluates safe moves from state JSON and writes to the boolean array.
void get_safe_moves_cpp(const char* state_json, bool* safe_moves_out);
}
