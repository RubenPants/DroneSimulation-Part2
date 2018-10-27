package interfaces;

import java.util.ArrayList;
import java.util.List;

import autopilot_planning.DubinsPath2D;
import autopilot_utilities.Path3D;
import autopilot_utilities.Point3D;
import autopilot_utilities.Vector3f;

public class PathReorden2 {

	boolean printsEnabled = false;

	int breadth = 4;
	double rho = 450;
	Point3D landingPoint = new Point3D(0, 60, -1500);

	public void reordenPath(Path3D path, Point3D positionDrone, float heading, float pitch) {
		// Variables
		int firstIndex = -1;
		int secondIndex = -1;
		int thirdIndex = -1;
		int fourthIndex = -1;
		int fifthIndex = -1;

		float minDubinsLength = Float.POSITIVE_INFINITY;

		Point3D coordinate;
		Point3D secondCoordinate;
		Vector3f cubeVector;
		float endHeading;
		float tempPitch;
		Vector3f startConfiguration;
		Vector3f endConfiguration;
		DubinsPath2D dubinsPath;

		// Consider the best point
		for (int i = 0; i < path.getCoordinates().size(); i++) {
			coordinate = path.getCoordinates().get(i);

			if (possibleFirstTarget(positionDrone, coordinate, heading, pitch)) {
				// Get closes second cube!
				float bestDistSecond;
				int tempSecondIndex;

				if (i != 0) {
					secondCoordinate = path.getCoordinate(0);
					bestDistSecond = distanceTo(coordinate, secondCoordinate);
					tempSecondIndex = 0;
				} else {
					secondCoordinate = path.getCoordinate(1);
					bestDistSecond = distanceTo(coordinate, secondCoordinate);
					tempSecondIndex = 1;
				}

				for (int j = 0; j < path.getCoordinates().size(); j++) {
					if (i != j) {
						if (distanceTo(coordinate, path.getCoordinate(j)) < bestDistSecond) {
							tempPitch = (float) Math.atan((path.getCoordinate(j).getY() - coordinate.getY())
									/ distanceTo(coordinate, path.getCoordinate(j)));
							if (tempPitch < Math.PI / 18 && tempPitch > -Math.PI / 15) {
								secondCoordinate = path.getCoordinate(j);
								bestDistSecond = distanceTo(coordinate, secondCoordinate);
								tempSecondIndex = j;
							}
						}
					}
				}

				if (tempSecondIndex > 0) {
					cubeVector = new Vector3f(secondCoordinate.getX() - coordinate.getX(), 0,
							secondCoordinate.getZ() - coordinate.getZ());
					endHeading = (float) Math.atan2(-cubeVector.x, -cubeVector.z);
					startConfiguration = new Vector3f(-positionDrone.getZ(), -positionDrone.getX(), heading);
					endConfiguration = new Vector3f(-coordinate.getZ(), -coordinate.getX(), endHeading);

					dubinsPath = new DubinsPath2D(startConfiguration, endConfiguration, rho);

					if (dubinsPath.getLength() < minDubinsLength) {
						minDubinsLength = (float) dubinsPath.getLength();
						firstIndex = i;
						secondIndex = tempSecondIndex;
					}
				}
			}
		}

		// No Point will fit first point --> start with closest cube with a distance of
		// at least 1000 from the drone
		if (firstIndex < 0) {
			firstIndex = 0;
			boolean biggerThen1000 = false;

			if (distanceTo(positionDrone, path.getCoordinate(firstIndex)) >= 1000)
				biggerThen1000 = true;

			for (int i = 1; i < path.getNbCoordinates(); i++) {
				if (distanceTo(path.getCoordinate(i), positionDrone) >= 1000
						&& (!biggerThen1000 || distanceTo(path.getCoordinate(i),
								positionDrone) < distanceTo(path.getCoordinate(firstIndex), positionDrone))) {
					biggerThen1000 = true;
					firstIndex = i;
				} else if (distanceTo(path.getCoordinate(i), positionDrone) < 1000 && distanceTo(path.getCoordinate(i),
						positionDrone) > distanceTo(path.getCoordinate(firstIndex), positionDrone)) {
					firstIndex = i;
				}
			}

		}

		List<Integer> indicesUsed = new ArrayList<>();
		indicesUsed.add(firstIndex);
		if (secondIndex > 0)
			indicesUsed.add(secondIndex);

		List<Integer> resultList = reordenPathRecursive(path, indicesUsed, breadth);

		float totalDistance1 = (float) (resultList.get(0) + minDubinsLength);
		if (printsEnabled)
			System.out.println("total distance: " + totalDistance1);

		if (resultList.size() > 1)
			firstIndex = resultList.get(1);
		else
			firstIndex = 0;

		if (resultList.size() > 2) {
			secondIndex = resultList.get(2);
			if (resultList.size() > 3) {
				thirdIndex = resultList.get(3);
				if (resultList.size() > 4) {
					fourthIndex = resultList.get(4);
					if (resultList.size() > 5) {
						fifthIndex = resultList.get(5);
					}
				}
			}
		}

		System.out.println("PATH REORDEND");
		System.out.print("[ ");
		if (firstIndex > 0)
			System.out.print("[" + path.getCoordinate(firstIndex).getX() + ", " + path.getCoordinate(firstIndex).getY()
					+ ", " + path.getCoordinate(firstIndex).getZ() + "], ");
		if (secondIndex > 0)
			System.out.print("[" + path.getCoordinate(secondIndex).getX() + ", "
					+ path.getCoordinate(secondIndex).getY() + ", " + path.getCoordinate(secondIndex).getZ() + "], ");
		if (thirdIndex > 0)
			System.out.print("[" + path.getCoordinate(thirdIndex).getX() + ", " + path.getCoordinate(thirdIndex).getY()
					+ ", " + path.getCoordinate(thirdIndex).getZ() + "], ");
		if (fourthIndex > 0)
			System.out.print("[" + path.getCoordinate(fourthIndex).getX() + ", "
					+ path.getCoordinate(fourthIndex).getY() + ", " + path.getCoordinate(fourthIndex).getZ() + "], ");
		if (fifthIndex > 0)
			System.out.print("[" + path.getCoordinate(fifthIndex).getX() + ", " + path.getCoordinate(fifthIndex).getY()
					+ ", " + path.getCoordinate(fifthIndex).getZ() + "], ");
		System.out.println(" ]");

		// Re-orden the path
		path.moveBestFive(firstIndex, secondIndex, thirdIndex, fourthIndex, fifthIndex);
	}

	private boolean possibleFirstTarget(Point3D positionDrone, Point3D target, float droneHeading, float dronePitch) {
		float reqPitch;
		float deltaPitch;
		float deltaHeading;
		float deltaX = distanceTo(positionDrone, target);
		float pitchFrac;
		float headingFrac;

		Vector3f requestedVector = new Vector3f(target.getX() - positionDrone.getX(),
				target.getY() - positionDrone.getY(), target.getZ() - positionDrone.getZ());

		reqPitch = (float) Math.atan(requestedVector.y
				/ (Math.sqrt(requestedVector.z * requestedVector.z + requestedVector.x * requestedVector.x)));
		deltaPitch = Math.abs(dronePitch - reqPitch);
		float reqHeading = (float) Math.atan2(-requestedVector.x, -requestedVector.z);

		deltaHeading = (reqHeading - droneHeading);
		if (deltaHeading > Math.PI)
			deltaHeading -= 2 * Math.PI;
		else if (deltaHeading < -Math.PI)
			deltaHeading += 2 * Math.PI;

		deltaHeading = Math.abs(deltaHeading);
		deltaPitch = Math.abs(deltaPitch);

		// Check pitch and heading
		if (reqPitch > Math.PI / 18 || reqPitch < -Math.PI / 15)
			return false;

		if (deltaX <= 100) {
			if (deltaPitch < Math.PI / 30)
				pitchFrac = (float) (deltaPitch / (Math.PI / 36));
			else
				return false;

			if (deltaHeading < Math.PI / 45)
				headingFrac = (float) (deltaHeading / (Math.PI / 45));
			else
				return false;
		} else if (deltaX <= 200) {
			if (deltaPitch < Math.atan(0.2 - 10 / deltaX))
				pitchFrac = (float) (deltaPitch / (Math.atan(0.2 - 10 / deltaX)));
			else
				return false;

			if (deltaHeading < Math.atan(43 / 100 - 36 / deltaX))
				headingFrac = (float) (deltaHeading / (Math.atan(43 / 100 - 36 / deltaX)));
			else
				return false;
		} else if (deltaX <= 300) {
			if (deltaPitch < Math.atan(0.3 - 30 / deltaX))
				pitchFrac = (float) (deltaPitch / (Math.atan(0.3 - 30 / deltaX)));
			else
				return false;

			if (deltaHeading < Math.atan(57 / 20 - 520 / deltaX))
				headingFrac = (float) (deltaHeading / (Math.atan(57 / 20 - 520 / deltaX)));
			else
				return false;
		} else if (deltaX <= 500) {
			pitchFrac = (float) (deltaPitch / (Math.PI / 15));
			headingFrac = (float) (deltaHeading / (Math.PI / 4)); // 45 degrees
		} else {
			if (deltaHeading > (Math.PI / 45) && deltaX < 1000) {
				return false;
			} else {
				pitchFrac = 0;
				headingFrac = 0;
			}
		}

		return (pitchFrac + headingFrac) <= 1;
	}

	public void reordenPath(Path3D path, Vector3f positionDrone, float heading, float pitch) {
		reordenPath(path, new Point3D(positionDrone.x, positionDrone.y, positionDrone.z), heading, pitch);
	}

	public List<Integer> reordenPathRecursive(Path3D path, List<Integer> indicesUsed, int breadth) {
		float minDist = Float.POSITIVE_INFINITY;
		List<Integer> minList = new ArrayList<>();
		List<Integer> returnList = new ArrayList<>();

		// Get the remaindingIndices
		List<Integer> remaindingIndices = new ArrayList<>();
		for (int i = 0; i < path.getNbCoordinates(); i++) {
			if (!indicesUsed.contains(i))
				remaindingIndices.add(i);
		}

		// Start recursive method
		Point3D thisPoint = path.getCoordinate(indicesUsed.get(indicesUsed.size() - 1));

		if (remaindingIndices.size() == 0 || breadth == 0) {
			if (remaindingIndices.size() == 0)
				returnList.add((int) distanceTo(thisPoint, landingPoint));
			else
				returnList.add(0);

			returnList.addAll(indicesUsed);
			return returnList;
		} else { // Go recursive

			getClosestPointsInOrder(path, remaindingIndices, thisPoint);

			for (int i = 0; i < Math.min(remaindingIndices.size(), breadth); i++) {
				// Recursive
				int usedIndex = remaindingIndices.get(i);
				indicesUsed.add(usedIndex);
				float distance = distanceTo(thisPoint, path.getCoordinate(usedIndex));

				if (Math.abs(thisPoint.getY() - path.getCoordinate(usedIndex).getY()) < distance
						* Math.tan(Math.PI / 15)) {
					returnList = reordenPathRecursive(path, indicesUsed, breadth - 1);

					if (returnList.get(0) + distance < minDist) {
						minDist = returnList.get(0) + distance;
						minList.clear();
						for (int ii = 1; ii < returnList.size(); ii++) {
							minList.add(returnList.get(ii));
						}
					}

					// Re-do recursive calculations
					indicesUsed.remove(indicesUsed.size() - 1);
				}
			}
		}

		List<Integer> resultList = new ArrayList<>();
		resultList.add((int) minDist);
		resultList.addAll(minList);
		return resultList;
	}

	public void getClosestPointsInOrder(Path3D path, List<Integer> remaindingIndices, Point3D point) {
		for (int indexOuterLoop = 1; indexOuterLoop < remaindingIndices.size(); indexOuterLoop++) {
			boolean isAdded = false;
			float distanceOuterInteger = distanceTo(point, path.getCoordinate(remaindingIndices.get(indexOuterLoop)));
			for (int indexInnerLoop = 0; indexInnerLoop < indexOuterLoop && !isAdded; indexInnerLoop++) {
				if (distanceOuterInteger < distanceTo(point,
						path.getCoordinate(remaindingIndices.get(indexInnerLoop)))) {
					int index = remaindingIndices.remove(indexOuterLoop);
					remaindingIndices.add(indexInnerLoop, index);
					isAdded = true;
				}
			}
		}
	}

	private float distanceTo(Point3D point, Point3D position) {
		return (float) Math
				.sqrt(Math.pow(point.getX() - position.getX(), 2) + Math.pow(point.getZ() - position.getZ(), 2));
	}
}
