// command_parser.h — split a serial command line into words and check numbers.
//
// Example: "sel_test 50 60 4 2 500" -> name "sel_test", 5 arguments.
// Plain C++ with no Arduino code, so it is unit-tested on the PC.
#pragma once
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

struct Command {
  static const int MAX_ARGS = 8;
  static const int MAX_LEN = 24;
  char name[MAX_LEN] = {0};
  int argc = 0;
  char args[MAX_ARGS][MAX_LEN] = {{0}};
};

// Split `line` on spaces/tabs. Returns false for an empty line, a word that is
// too long, or too many words. Never writes past the end of any buffer.
inline bool parseCommand(const char* line, Command& out) {
  out = Command();
  int word = -1;  // -1 = the command name, 0.. = arguments
  const char* p = line;
  while (*p) {
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (!*p) break;
    const char* start = p;
    while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') p++;
    const int len = (int)(p - start);
    if (len >= Command::MAX_LEN) return false;
    if (word >= Command::MAX_ARGS) return false;
    char* dst = (word < 0) ? out.name : out.args[word];
    memcpy(dst, start, len);
    dst[len] = '\0';
    word++;
  }
  if (word < 0) return false;  // nothing but spaces
  out.argc = word;
  return true;
}

// Read a whole-number argument and check it is in [lo, hi].
// "12abc" and "" are rejected (the entire word must be a number).
inline bool parseLong(const char* s, long lo, long hi, long& out) {
  if (!s || !*s) return false;
  char* end = nullptr;
  const long v = strtol(s, &end, 10);
  if (*end != '\0') return false;
  if (v < lo || v > hi) return false;
  out = v;
  return true;
}

// Same for decimal numbers such as "62.5".
inline bool parseFloat(const char* s, float lo, float hi, float& out) {
  if (!s || !*s) return false;
  char* end = nullptr;
  const float v = strtof(s, &end);
  if (*end != '\0') return false;
  if (!(v >= lo && v <= hi)) return false;  // also rejects NaN
  out = v;
  return true;
}
