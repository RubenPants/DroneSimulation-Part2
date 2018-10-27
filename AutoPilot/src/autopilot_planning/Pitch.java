package autopilot_planning;

import autopilot_utilities.Point3D;
import autopilot_utilities.Vector3f;
import interfaces.AutopilotInputs;

public interface Pitch {
		
	/**
	 * Return the angle of pitch the drone should have at the given moment
	 * 
	 * @param inputs
	 * @param target
	 * 
	 * @return   The pitch (radians)
	 */
	static float getPitch(AutopilotInputs inputs, Point3D target) {
		if (target != null) {
			float pitch;
			Vector3f requestedVector = new Vector3f(
					target.getX() - inputs.getX(), 
					target.getY() - inputs.getY(),
					target.getZ() - inputs.getZ());
			
			pitch = (float) Math.atan(requestedVector.y / 
					(Math.sqrt(requestedVector.z * requestedVector.z + requestedVector.x * requestedVector.x)));
			float shortPitch = (float) Math.atan(requestedVector.y / 100.0);

			if (shortPitch > Math.PI / 36) shortPitch = (float) Math.PI / 36;
			else if (shortPitch < -Math.PI / 36) shortPitch = (float) -Math.PI / 36;
			if (pitch > 0 && pitch < shortPitch) return shortPitch;
			if (pitch < 0 && pitch > shortPitch) return shortPitch;
			
			else return pitch;
		} else
			return (float) Math.PI/36;
	}

}
