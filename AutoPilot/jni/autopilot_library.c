/**
 * C implementation of autopilot_library (JNI).
 *  This class just forwards calls to the relevant classes.
 *
 * @author Team Safier
 * @version 1.0
 */

#include "autopilot_library.h"

#include "vision/vision.h"
#include "planning/planning.h"

JNIEXPORT jobjectArray JNICALL Java_autopilot_1library_AutopilotLibrary_locateUnitCubes
(JNIEnv *env, jclass class, jbyteArray bytes, jint width, jint height, jboolean useHistory, jboolean hasGround) {
    return locateUnitCubes(env, class, bytes, width, height, useHistory, hasGround);
}