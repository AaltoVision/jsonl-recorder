#include "assert.hpp"

#include <cstdlib>

namespace accelerated {
void assert_fail(const char *assertion, const char *fn, unsigned int line, const char *func) {
  // TODO: Logging
  std::abort();
}
}
