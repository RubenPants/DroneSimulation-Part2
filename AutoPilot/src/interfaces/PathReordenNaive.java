package interfaces;

import autopilot_utilities.Path3D;
import autopilot_utilities.Point3D;
import autopilot_utilities.Vector3f;

public class PathReordenNaive {
	
	private float rho = 400;
	
	public void reordenPath(Path3D path, Vector3f positionDrone, float droneHeading, float dronePitch) {
		float minDist = Float.POSITIVE_INFINITY;
		int bestInt = 0;
		float distance;
		Point3D target;
		float reqPitch, reqHeading;
		float deltaPitch, deltaHeading;
		
		for (int i = 0; i < path.getNbCoordinates(); i++) {
			target = path.getCoordinate(i);
			Vector3f requestedVector = new Vector3f(target.getX() - positionDrone.x, 
					target.getY() - positionDrone.y, 
					target.getZ() - positionDrone.z);
			
			reqPitch = (float)Math.atan(requestedVector.y/(Math.sqrt(requestedVector.z*requestedVector.z + requestedVector.x*requestedVector.x)));
			deltaPitch = Math.abs(dronePitch - reqPitch);
			reqHeading = (float)Math.atan2(-requestedVector.x, -requestedVector.z);

			deltaHeading = (reqHeading - droneHeading);
			if (deltaHeading > Math.PI)	deltaHeading -= 2 * Math.PI;
			else if (deltaHeading < -Math.PI) deltaHeading += 2 * Math.PI;
			
			deltaHeading = Math.abs(deltaHeading);
			deltaPitch = Math.abs(deltaPitch);
			
			if (possibleFirstTarget(positionDrone, path.getCoordinate(i), reqPitch, deltaHeading, deltaPitch)) {
				distance = distanceToHeuristic(path.getCoordinate(i), positionDrone, deltaHeading, deltaPitch);
				if (distance < minDist) {
					minDist = distance;
					bestInt = i;
				}
			}
		}
		
		path.moveToStart(bestInt);
	}
	
	private boolean possibleFirstTarget(Vector3f positionDrone, Point3D target, float reqPitch, float deltaHeading, float deltaPitch) {
		float deltaX = distanceTo(target, positionDrone);
		float pitchFrac;
		float headingFrac;
		
		// Check pitch and heading
		if (reqPitch > Math.PI/18 || reqPitch < -Math.PI/15) return false;
		
		if (deltaX <= 100) {
			if (deltaPitch < Math.PI/36)	// 5 degrees
				pitchFrac = (float) (deltaPitch / (Math.PI/36));
			else
				return false;
			
			if (deltaHeading < Math.PI/45)	// 4 degrees
				headingFrac = (float) (deltaHeading / (Math.PI/45));
			else
				return false;
		}
		else if (deltaX <= 200) {
			if (deltaPitch < Math.atan(0.2-10/deltaX))
				pitchFrac = (float) (deltaPitch / (Math.atan(0.2-10/deltaX)));
			else
				return false;
			
			if (deltaHeading < Math.atan(43/100 - 36/deltaX))
				headingFrac = (float) (deltaHeading / (Math.atan(43/100 - 36/deltaX)));
			else
				return false;
		}
		else if (deltaX <= 300) {
			if (deltaPitch < Math.atan(0.3-30/deltaX))
				pitchFrac = (float) (deltaPitch / (Math.atan(0.3-30/deltaX)));
			else
				return false;
			
			if (deltaHeading < Math.atan(57/20 - 520/deltaX))
				headingFrac = (float) (deltaHeading / (Math.atan(57/20 - 520/deltaX)));
			else
				return false;
		}
		else if (deltaX <= 500) {
			pitchFrac = (float) (deltaPitch/(Math.PI/15));	// 12 degrees
			headingFrac = (float) (deltaHeading/(Math.PI/8));	// 22.5 degrees
		}
		else {
			if (deltaHeading > (Math.PI/45) && deltaX < 1000) {
				return false;
			} else {
				pitchFrac = 0;
				headingFrac = 0;				
			}
		}

		return (pitchFrac + headingFrac) <= 1;
	}

	private float distanceTo(Point3D point, Vector3f position) {
		return (float) Math.sqrt(Math.pow(point.getX() - position.x, 2) + Math.pow(point.getZ() - position.z, 2));
	}

	private float distanceToHeuristic(Point3D point, Vector3f position, float deltaHeading, float deltaPitch) {
		float distance = (float) Math.sqrt(Math.pow(point.getX() - position.x, 2) + Math.pow(point.getZ() - position.z, 2));
		float turn = deltaHeading*rho;
		float levelDistance = (float) (distance*Math.atan(deltaPitch));
		return distance + turn + levelDistance;
	}
}
