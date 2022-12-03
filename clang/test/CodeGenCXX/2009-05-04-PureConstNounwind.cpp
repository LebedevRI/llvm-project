// RUN: %clang_cc1 -triple i386-unknown-unknown -fexceptions -emit-llvm %s -o - | FileCheck %s
int c(void) __attribute__((const));
int p(void) __attribute__((pure));
int NoT(void) __attribute__((nothrow));
int t(void);

// CHECK: define{{.*}} i32 @_Z1fv() [[TF:#[0-9]+]] {
int f(void) {
  // CHECK: call noundef i32 @_Z1cv() [[RN_CALL:#[0-9]+]]
  // CHECK: call noundef i32 @_Z1pv() [[RO_CALL:#[0-9]+]]
  // CHECK: call noundef i32 @_Z3NoTv() [[NUW_CALL:#[0-9]+]]
  return c() + p() + NoT() +  t();
}

// CHECK: declare noundef i32 @_Z1cv() [[RN:#[0-9]+]]
// CHECK: declare noundef i32 @_Z1pv() [[RO:#[0-9]+]]
// CHECK: declare noundef i32 @_Z3NoTv() [[NUW:#[0-9]+]]
// CHECK: declare noundef i32 @_Z1tv() [[TF2:#[0-9]+]]

// CHECK: attributes [[TF]] = { {{.*}} }
// CHECK: attributes [[RN]] = { willreturn memory(none){{.*}} }
// CHECK: attributes [[RO]] = { willreturn memory(read){{.*}} }
// CHECK: attributes [[NUW]] = { nounwind {{.*}} }
// CHECK: attributes [[TF2]] = { {{.*}} }
// CHECK: attributes [[RN_CALL]] = { willreturn memory(none) }
// CHECK: attributes [[RO_CALL]] = { willreturn memory(read) }
// CHECK: attributes [[NUW_CALL]] = { nounwind }
