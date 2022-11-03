// RUN: %clangxx -fsanitize=exception-escape %s -O3 -o %t
// RUN:     %run %t 2>&1 | FileCheck %s --implicit-check-not="error:" --check-prefixes=CHECK-ALL,CHECK-ONE
// RUN: not %run %t 2 2>&1 | FileCheck %s --implicit-check-not="error:" --check-prefixes=CHECK-ALL,CHECK-TWO
// RUN: not %run %t 2 3 2>&1 | FileCheck %s --implicit-check-not="error:" --check-prefixes=CHECK-ALL,CHECK-THREE
// RUN:     %run %t 2 3 4 2>&1 | FileCheck %s --implicit-check-not="error:" --check-prefixes=CHECK-ALL,CHECK-FOUR
// RUN: not %run %t 2 3 4 5 2>&1 | FileCheck %s --implicit-check-not="error:" --check-prefixes=CHECK-ALL,CHECK-FIVE
// RUN: not %run %t 2 3 4 5 6 2>&1 | FileCheck %s --implicit-check-not="error:" --check-prefixes=CHECK-ALL,CHECK-SIX
// RUN: not %run %t 2 3 4 5 6 6 2>&1 | FileCheck %s --implicit-check-not="error:" --check-prefixes=CHECK-ALL,CHECK-SEVEN

#include <stdio.h>
#include <stdlib.h>

void thrower() { throw 0; }

void maybe_throws() {}

void nonthrower() noexcept {}

// pure functions are generally side-effect free,
// so we need to be smart to defeat optimizer
// from completely dropping the call to the function...

void *__attribute__((pure)) footgun(int x) {
  if (x == 2)
#line 100
    thrower();

  if (x == 3)
#line 200
    thrower();

  nonthrower();

  if (x == 4) {
    try {
#line 300
      thrower();
    } catch (...) {
    }
  }

  if (x == 5) {
    try {
#line 400
      thrower();
    } catch (...) {
#line 500
      throw;
    }
  }

  if (x == 6) {
    try {
#line 600
      thrower();
    } catch (...) {
#line 700
      maybe_throws();
#line 800
      throw;
    }
  }

  if (x == 7) {
    try {
#line 900
      thrower();
    } catch (...) {
#line 1000
      thrower();
#line 1100
      throw;
    }
  }

  fprintf(stderr, "SUCCESS\n"); // Should be here, not in `main()`.
  return malloc(1);
}

// CHECK-ALL: TEST

// CHECK-ONE: SUCCESS
// CHECK-TWO: exception-escape.cpp:100:5: runtime error: exception escapes out of function that should not throw exception
// CHECK-THREE: exception-escape.cpp:200:5: runtime error: exception escapes out of function that should not throw exception
// CHECK-FOUR: SUCCESS
// CHECK-FIVE: exception-escape.cpp:400:7: runtime error: exception escapes out of function that should not throw exception
// CHECK-SIX: exception-escape.cpp:600:7: runtime error: exception escapes out of function that should not throw exception
// CHECK-SEVEN: exception-escape.cpp:1000:7: runtime error: exception escapes out of function that should not throw exception

int main(int argc, char **argv) {
  bool status = 0;
  void *mem = nullptr;

  fprintf(stderr, "TEST\n");

  try {
    mem = footgun(argc);
    status = !(mem != 0);
  } catch (...) {
  }

  fprintf(stderr, "escaping: %p\n", mem);
  return status;
}
