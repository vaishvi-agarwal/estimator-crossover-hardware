// serial_commands.h — split one typed line into a command word and its
// arguments. Plain C++ (no Arduino), tested in tests/test_host.cpp.
//
//   "ge 0.02 0.1 50"      -> name "ge",   numbers {0.02, 0.1, 50}
//   "mode mix 0.3"        -> name "mode", word "mix", numbers {0.3}
//   "inject on"           -> name "inject", word "on"
#pragma once
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

struct Command {
  static const int MAX_NUMBERS = 6;
  char name[16] = {0};  // first word, lower case
  char word[16] = {0};  // first argument that is not a number ("" if none)
  double numbers[MAX_NUMBERS] = {0};
  int count = 0;        // how many numbers were found
  bool bad_number = false;  // an argument looked like a number but was not
};

// Parse a line. Returns false for an empty line.
inline bool parseCommand(const char* line, Command& cmd) {
  cmd = Command();
  char buf[128];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  int nTokens = 0;
  for (char* tok = strtok(buf, " \t\r\n"); tok; tok = strtok(nullptr, " \t\r\n")) {
    for (char* p = tok; *p; p++) *p = (char)tolower((unsigned char)*p);
    if (nTokens++ == 0) {
      strncpy(cmd.name, tok, sizeof(cmd.name) - 1);
      continue;
    }
    char* end = nullptr;
    const double v = strtod(tok, &end);
    if (end != tok && *end == 0) {
      if (cmd.count < Command::MAX_NUMBERS) cmd.numbers[cmd.count++] = v;
    } else if (cmd.word[0] == 0 && !(isdigit((unsigned char)tok[0]) || tok[0] == '-' || tok[0] == '.')) {
      strncpy(cmd.word, tok, sizeof(cmd.word) - 1);
    } else {
      cmd.bad_number = true;
    }
  }
  return nTokens > 0;
}

// Collects characters into lines. feed() returns true when a full line is ready.
class LineBuffer {
 public:
  bool feed(char c) {
    if (c == '\n' || c == '\r') {
      if (len_ == 0) return false;
      buf_[len_] = 0;
      len_ = 0;
      return true;
    }
    if (len_ < sizeof(buf_) - 1) buf_[len_++] = c;
    return false;
  }
  const char* line() const { return buf_; }

 private:
  char buf_[128] = {0};
  size_t len_ = 0;
};
