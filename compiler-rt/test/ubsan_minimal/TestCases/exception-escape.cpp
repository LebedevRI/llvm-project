// RUN: %clangxx -fsanitize=exception-escape %s -O3 -o %t
// RUN: %run %t 2>&1 | FileCheck %s --implicit-check-not="exception-escape" --check-prefixes=CHECK-ALL,CHECK-ONE
// RUN: not --crash %run %t a 2>&1 | FileCheck %s --implicit-check-not="exception-escape" --check-prefixes=CHECK-ALL,CHECK-TWO
// RUN: not --crash %run %t a b 2>&1 | FileCheck %s --implicit-check-not="exception-escape" --check-prefixes=CHECK-ALL,CHECK-THREE

#include <stdio.h>
#include <stdlib.h>

void thrower() { throw 0; }

void nonthrower() noexcept {}

// pure functions are generally side-effect free,
// so we need to be smart to defeat optimizer
// from completely dropping the call to the function...

void *__attribute__((pure)) footgun(int x) {
  if (x == 2)
    thrower();
  if (x == 3)
    thrower();
  nonthrower();
  fprintf(stderr, "SUCCESS\n");
  return malloc(1);
}

// CHECK-ALL: TEST

// CHECK-ONE: SUCCESS
// CHECK-TWO: exception-escape
// CHECK-THREE: exception-escape

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
  free(mem);
  return status;
}
