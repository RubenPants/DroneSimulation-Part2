/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class autopilot_library_AutopilotLibrary */

#ifndef _Included_autopilot_library_AutopilotLibrary
#define _Included_autopilot_library_AutopilotLibrary
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     autopilot_library_AutopilotLibrary
 * Method:    setup
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_autopilot_1library_AutopilotLibrary_setup
  (JNIEnv *, jclass);

/*
 * Class:     autopilot_library_AutopilotLibrary
 * Method:    cleanup
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_autopilot_1library_AutopilotLibrary_cleanup
  (JNIEnv *, jclass);

/*
 * Class:     autopilot_library_AutopilotLibrary
 * Method:    locateUnitCubes
 * Signature: ([BIIZZ)[[I
 */
JNIEXPORT jobjectArray JNICALL Java_autopilot_1library_AutopilotLibrary_locateUnitCubes
  (JNIEnv *, jclass, jbyteArray, jint, jint, jboolean, jboolean);

#ifdef __cplusplus
}
#endif
#endif
