package autopilot_planning;

import java.util.List;

import autopilot_utilities.Path3D;
import autopilot_utilities.Point3D;
import autopilot_utilities.Vector3f;
import interfaces.PathReorden2;

public class DroneTrajectoryDubins extends DroneTrajectory {
	
	// Print booleans
	private boolean ENABLE_LOGGING = false;
	private boolean printPoints = true;
	private boolean printPID = true;
	private boolean printDubins = true;
	
	// Member variables
	private int stage;
	private DubinsPath2D dubinsPath = null;
	private float endHeading;
	
	private List<Point3D> targetCoordinates;
	private Point3D firstCircleCenter;
	private Point3D secondCircleCenter;
	private Point3D firstCircleKeyPoint;
	private Point3D secondCircleKeyPoint;
	private Point3D target;
	private Point3D secondTarget;
	private Point3D thirdTarget;

	// PID using position
	private float positionError = 0;
	private float prevPositionError = 0;

	private Point3D dronePosStart = new Point3D(0,0,0);

	private float maxFault = 3f;
	private float maxFaultResetAll = 30f;
	private boolean turnAround = false;
	
	private boolean dubinsInitialized = false;
	private boolean reordenPath = true;
	private Point3D prevDronePos = new Point3D(0,0,0);
	private Point3D prevDronePosPath = new Point3D(0,0,0);
	private float distanceTraveled = 0;
	
	
	@Override
	protected float[] calculateTrajectoryInternal(Path3D path, Vector3f dronePosition, Vector3f rotations, double velocity, double rho, boolean landingTrailSet, float deltaTimeElapsed) {
		float[] motion = new float[2];
		if (targetCoordinates != null && targetCoordinates.size() != path.getNbCoordinates())
			dubinsInitialized = false;

		targetCoordinates = path.getCoordinates();
		
		if (!dubinsInitialized || (dronePosition.y > 30 && dronePosition.y < 31) || target == null) {
			if (dronePosition.y < 31) reordenPath = true;
			
			if (ENABLE_LOGGING) {
				System.out.print("Old Path: [");
				for (Point3D coor: path.getCoordinates())
					System.out.print("["+coor.getX()+", "+coor.getY()+", "+coor.getZ()+"]");
				System.out.println("]");
			}
			
			initDubins(path, dronePosition, rotations, rho, landingTrailSet);
			
			if (ENABLE_LOGGING) {
				System.out.print("New Path: [");
				for (Point3D coor: path.getCoordinates())
					System.out.print("["+coor.getX()+", "+coor.getY()+", "+coor.getZ()+"]");
				System.out.println("]");
				
				System.out.println("Initialized!");
			}
		}
		else if (Math.abs(positionError) > maxFaultResetAll) {
			if (ENABLE_LOGGING) System.out.println("ERROR: MAX FAULT SURPASSED - "+Math.abs(positionError));
			crashResetAll();
			
			if (ENABLE_LOGGING) {
				System.out.print("Old Path: [");
				for (Point3D coor: path.getCoordinates())
					System.out.print("["+coor.getX()+", "+coor.getY()+", "+coor.getZ()+"]");
				System.out.println("]");
			}
				
			initDubins(path, dronePosition, rotations, rho, landingTrailSet);
			
			if (ENABLE_LOGGING) {
				System.out.print("New Path: [");
				for (Point3D coor: path.getCoordinates())
					System.out.print("["+coor.getX()+", "+coor.getY()+", "+coor.getZ()+"]");
				System.out.println("]");
			}
		}
		
		if (dubinsPath == null) {
			if (landingTrailSet) dubinsInitialized = false;
			motion = handleOnlyNextCube(dronePosition, rotations, true);
		} else {
			switch (targetCoordinates.size()) {
			case 0:
				motion[0] = (float)rotations.y;
				motion[1] = (float)rotations.x;
				break;
			case 1:
				if (secondTarget != null) {
					motion[0] = getPitch(dronePosition, target);
					motion[1] = handleDubinsDefault(dronePosition, rotations, landingTrailSet);
					
					if (Math.abs(getPitch(dronePosition, target)) > Math.PI/36 && 
							distanceTo(target, dronePosition) > 50) 
						dubinsPath = null;
				} else {
					motion = handleOnlyNextCube(dronePosition, rotations, true);
				}
				break;
			default:
				motion[0] = getPitch(dronePosition, target);
				motion[1] = handleDubinsDefault(dronePosition, rotations, landingTrailSet);
				if (dubinsPath != null && dubinsPath.type.params[0] + dubinsPath.type.params[2] > 2*Math.PI) {
					motion[1] = (float) rotations.x;
					dubinsInitialized = false;
				}
				
				if (getPitch(dronePosition, target) < -Math.PI/36 && 
						distanceTo(target, dronePosition) > 50 && 
						stage < 2) 
					dubinsPath = null;
				break;
			}
		}
		
		if (ENABLE_LOGGING && printPoints) System.out.println("targetCoordinates.size(): "+targetCoordinates.size());
		if (ENABLE_LOGGING && printDubins && dubinsPath == null) System.out.println("DUBINS - null");
		if (ENABLE_LOGGING) System.out.println("Dubins initialized: "+dubinsInitialized);
		
		return motion;
	}
	
	
	private void initDubins(Path3D path, Vector3f dronePosition, Vector3f rotations, double rho, boolean landingTrailSet) {
		// Re-orden path if needed
		if (!landingTrailSet && reordenPath) {
			if (distanceTo(prevDronePosPath, dronePosition) > 300)
				distanceTraveled = Float.POSITIVE_INFINITY;
			
			if (path.getCoordinates().size() > 1 && distanceTraveled > 200) {
				//new PathReorden2().reordenPath(path, dronePosition, (float)rotations.x, (float) rotations.y);
				targetCoordinates = path.getCoordinates();
				prevDronePosPath = new Point3D(dronePosition.x, dronePosition.y, dronePosition.z);
				distanceTraveled = 0;
			} else {
				distanceTraveled += distanceTo(prevDronePos, dronePosition);
				prevDronePos = new Point3D(dronePosition.x, dronePosition.y, dronePosition.z);
			}
			
			reordenPath = false;
		}
		
		switch(path.getNbCoordinates()) {
		case 0:
			target = new Point3D(0, 0, 0);
			break;
		case 1:
			target = path.getCoordinate(0);
			if (landingTrailSet)
				secondTarget = null;
			else
				secondTarget = calculateSecondLandingCube(target);
			thirdTarget = null;
			break;
		case 2:
			target = path.getCoordinate(0);
			secondTarget = path.getCoordinate(1);
			if (landingTrailSet)
				thirdTarget = null;
			else
				thirdTarget = new Point3D(0.0, 60.0, -2000.0);
			break;
		default:
			target = path.getCoordinate(0);
			secondTarget = path.getCoordinate(1);
			thirdTarget = path.getCoordinate(2);
			break;
		}
		
		// Reposition the second target
		if (thirdTarget != null && distanceTo(target, secondTarget) > 1000) {
			Vector3f differenceVector = new Vector3f(
					secondTarget.getX() - thirdTarget.getX(),
					secondTarget.getY() - thirdTarget.getY(),
					secondTarget.getZ() - thirdTarget.getZ());
			double differenceHeading = Math.atan2(-differenceVector.x, -differenceVector.z);
			secondTarget = new Point3D(
					secondTarget.getX() - 500 * Math.sin(differenceHeading), 
					secondTarget.getY(), 
					secondTarget.getZ() - 500 * Math.cos(differenceHeading));
		} 

		// Create the Dubins-path if enough target cubes
		if (secondTarget != null) {
			Vector3f cubeVector = new Vector3f(secondTarget.getX() - target.getX(), 0, secondTarget.getZ() - target.getZ());
			endHeading = (float) Math.atan2(-cubeVector.x, -cubeVector.z);
			Vector3f startConfiguration = new Vector3f(-dronePosition.z, -dronePosition.x, rotations.x);
			Vector3f endConfiguration = new Vector3f(-target.getZ(), -target.getX(), endHeading);

			dubinsPath = new DubinsPath2D(startConfiguration, endConfiguration, rho);
			
			calculateCircleCenters(dronePosition, rotations, dubinsPath.type.identifier.toString());
			calculateCircleKeyPoints(dronePosition, rotations, dubinsPath.type.toString());
		}
		
		if (!landingTrailSet && target != null && dubinsPath != null) {
			Vector3f targetPointVector = new Vector3f(
					target.getX() - dronePosition.x, 
					0, 
					target.getZ() - dronePosition.z);
			float reqHeading = (float) Math.atan2(-targetPointVector.x, -targetPointVector.z);
			double headingDifference = Math.min(Math.abs(rotations.x - reqHeading),
					Math.PI * 2 - Math.abs(rotations.x - reqHeading));
			
			
			// Go naive if target is to close to drone AND in front of drone!
			if ((distanceTo(target, dronePosition) < 800 && Math.abs(headingDifference) < Math.PI/4) || 
					(distanceTo(target, dronePosition) < 1000 && (dubinsPath.type.params[0] + dubinsPath.type.params[2]) > (3.0/2.0)*Math.PI && Math.abs(headingDifference) < Math.PI/4)){
				dubinsPath = null;
			}
		}
		
		dubinsInitialized = true;
	}
	
	private Point3D calculateSecondLandingCube(Point3D target) {
		if (target.getZ() < -2500) {
			return new Point3D(0.0, 60.0, -2000.0);
		} else if (target.getZ() < -500) {
			if (target.getX() > 0)
				return new Point3D(1000.0, 60.0, target.getZ()-500);
			else
				return new Point3D(-1000.0, 60.0, target.getZ()-500);
		} else {
			if (target.getX() > 0)
				return new Point3D(1000.0, 60.0, -1500);
			else
				return new Point3D(-1000.0, 60.0, -1500);
		}
	}
	
	private float handleDubinsDefault(Vector3f dronePosition, Vector3f rotations, boolean landingTrailSet) {
		switch (stage) {
		case 0:	// First circle
			if (isReachedWide(firstCircleKeyPoint, dronePosition, rotations) ||
					dubinsPath.type.params[0] < Math.PI/18) {
				stage++;
				dronePosStart = new Point3D(dronePosition.x, dronePosition.y, dronePosition.z);
			}
			break;
		case 1:	// Straight line
			if (isReached(secondCircleKeyPoint, dronePosition, rotations)) stage++;
			else if (landingTrailSet && isReachedWide(secondCircleKeyPoint, dronePosition, rotations) && dubinsPath.type.params[2] < Math.PI/36) stage++;
			break;
		case 2:	// Last circle
			if (isReached(target, dronePosition, rotations)) stage++;
			else if (landingTrailSet && isReachedWide(target, dronePosition, rotations)) stage++;
			break;
		default:
			if (target != targetCoordinates.get(0) || distanceTo(target, dronePosition) > 100 || landingTrailSet)
				resetAll(dronePosition);

			Vector3f targetPointVector = new Vector3f(
					target.getX() - dronePosition.x, 
					0, 
					target.getZ() - dronePosition.z);
			float reqHeading = (float) Math.atan2(-targetPointVector.x, -targetPointVector.z);
			double headingDifference = Math.min(Math.abs(rotations.x - reqHeading),
					Math.PI * 2 - Math.abs(rotations.x - reqHeading));
			
			// The drone has missed the objective
			if (headingDifference > Math.PI/2 && distanceTo(target, dronePosition) < 500) {
				if (ENABLE_LOGGING) System.out.println("ERROR: TARGET IMPOSSIBLE TO REACH - 311");
				crashResetAll();
			}
		}
		
		if (ENABLE_LOGGING && printDubins) {
			System.out.println("DUBINS: " + dubinsPath.type.identifier.toString() + ", stage: "+stage);
			System.out.println("--- length " + dubinsPath.getLength());
			System.out.println("--- params " + dubinsPath.type.params[0]);
			System.out.println("--- params " + dubinsPath.type.params[1]);
			System.out.println("--- params " + dubinsPath.type.params[2]);
		}

		if (ENABLE_LOGGING && printPoints) {
			System.out.println("POINTS");
			System.out.println("---firstCenter: ["+firstCircleCenter.getX()+",0,"+firstCircleCenter.getZ()+"]");
			System.out.println("---secondCenter: ["+secondCircleCenter.getX()+",0,"+secondCircleCenter.getZ()+"]");
			System.out.println("---firstCircleKeyPoint: ["+firstCircleKeyPoint.getX()+",0,"+firstCircleKeyPoint.getZ()+"]");
			System.out.println("---secondCircleKeyPoint: ["+secondCircleKeyPoint.getX()+",0,"+secondCircleKeyPoint.getZ()+"]");
			if (target != null)
				System.out.println("---target: ["+target.getX()+","+target.getY()+","+target.getZ()+"]");
			if (secondTarget != null)
				System.out.println("---secondTarget: ["+secondTarget.getX()+","+secondTarget.getY()+","+secondTarget.getZ()+"]");
			if (thirdTarget != null)
				System.out.println("---thirdTarget: ["+thirdTarget.getX()+","+thirdTarget.getY()+","+thirdTarget.getZ()+"]");			
		}
		
		return getPidPosition(dronePosition, rotations);
	}
	
	
	
	
	
	
	
	
	
	
	// Error methods
	private float[] handleOnlyNextCube(Vector3f dronePosition, Vector3f rotations, boolean iterationEnabled) {
		if (distanceTo(target, dronePosition) > 1000)
			dubinsInitialized = false;
			
		if (ENABLE_LOGGING) System.out.println("GO DIRECTLY TO NEXT");
		float[] motion = new float[2];

		// Check if drone is heading towards its target (delta heading < PI/4), otherwise keep updating dronePosStart
		Vector3f requestedVector = new Vector3f(
				target.getX() - dronePosition.x, 
				0, 
				target.getZ() - dronePosition.z);
		float deltaHeading = (float) (rotations.x - Math.atan2(-requestedVector.x, -requestedVector.z));
		if (deltaHeading > Math.PI) deltaHeading -= 2*Math.PI;
		else if (deltaHeading < -Math.PI) deltaHeading += 2*Math.PI;
		
		motion[0] = getPitch(dronePosition, target);
		if (turnAround) {
			if (ENABLE_LOGGING) System.out.println("ERROR: TURN AROUND");
			crashResetAll();
			if (distanceTo(target, dronePosition) > 600) {
				if (rotations.z > Math.PI/90) motion[1] = (float) (rotations.x + Math.PI/16);	// roll of 33.75 degrees
				else motion[1] = (float) (rotations.x - Math.PI/16);
			} else {
				motion[1] = (float) rotations.x;
			}
		} else if (distanceTo(target, dronePosition) < 500 && Math.abs(deltaHeading) > Math.PI/2) {
			if (ENABLE_LOGGING) System.out.println("ERROR: TARGET IMPOSSIBLE TO REACH - 390");
			crashResetAll();
			motion[1] = (float) rotations.x;
		} else if (deltaHeading > Math.PI/4) {
			motion[1] = (float) (rotations.x - Math.PI/16);	// 33.75 degrees
		} else if (deltaHeading < -Math.PI/4) {
			motion[1] = (float) (rotations.x + Math.PI/16);	// 33.75 degrees
		} else {
			motion[1] = getPidPositionOneCube(dronePosition, rotations);
		}
		
		if (Math.abs(positionError) > maxFault && iterationEnabled) {
			if (ENABLE_LOGGING) {
				System.out.println("dronePosStart updated!");
				System.out.println("Fault: "+Math.abs(positionError));
			}
			dronePosStart = new Point3D(dronePosition.x, dronePosition.y, dronePosition.z);
			if (ENABLE_LOGGING) System.out.println("Iteration");
			handleOnlyNextCube(dronePosition, rotations, false);
		}
		
		if (ENABLE_LOGGING && printPoints) {
			System.out.println("POINTS");
			if (dronePosition != null)
				System.out.println("---dronePosition: ["+dronePosition.x+",0,"+dronePosition.z+"]");
			if (dronePosStart != null)
				System.out.println("---dronePosStart: ["+dronePosStart.getX()+",0,"+dronePosStart.getZ()+"]");
			if (target != null)
				System.out.println("---target: ["+target.getX()+","+target.getY()+","+target.getZ()+"]");
			if (secondTarget != null)
				System.out.println("---secondTarget: ["+secondTarget.getX()+","+secondTarget.getY()+","+secondTarget.getZ()+"]");
			if (thirdTarget != null)
				System.out.println("---thirdTarget: ["+thirdTarget.getX()+","+thirdTarget.getY()+","+thirdTarget.getZ()+"]");			
		}
		
		if (isReached(target, dronePosition, rotations)) 
			resetAll(dronePosition);
		
		return motion;
	}
	
	
	// INITIALISATION METHODS //
	
	private void calculateCircleCenters(Vector3f dronePosition, Vector3f rotations, String dubinsString) {
		String directionFirst = dubinsString.substring(0, 1);
		String directionSecond = dubinsString.substring(2, 3);
		float sigma;
		
		// Calculate first center
		if (directionFirst.equals("R"))
			sigma = (float) (rotations.x - Math.PI / 2);
		else // "L"
			sigma = (float) (rotations.x + Math.PI / 2);
		
		firstCircleCenter = new Point3D(
				dronePosition.x - 400 * Math.sin(sigma), 
				0,
				dronePosition.z - 400 * Math.cos(sigma));
		
		// Calculate second center
		if (directionSecond.equals("R"))
			sigma = (float) (endHeading - Math.PI / 2);
		else // "L"
			sigma = (float) (endHeading + Math.PI / 2);
		
		secondCircleCenter = new Point3D(
				target.getX() - 400 * Math.sin(sigma), 
				0,
				target.getZ() - 400 * Math.cos(sigma));
	}
	
	private void calculateCircleKeyPoints(Vector3f dronePosition, Vector3f rotations, String dubinsString) {
		// Get first key point
		double deltaAngle = dubinsPath.type.params[0];
		if (dubinsString.substring(0, 1).equals("L")) deltaAngle *= -1;
		Vector3f targetCubeVector = new Vector3f(
				dronePosition.x - firstCircleCenter.getX(), 
				0,
				dronePosition.z - firstCircleCenter.getZ());
		float centerTargetAngle = (float) Math.atan2(-targetCubeVector.x, -targetCubeVector.z);
		float totalAngle;
		if (dubinsPath.type.identifier.toString().substring(0, 1).equals("R"))
			totalAngle = (float) (centerTargetAngle - deltaAngle);
		else 
			totalAngle = (float) (centerTargetAngle + deltaAngle);
		firstCircleKeyPoint = new Point3D(
				firstCircleCenter.getX() - 400 * Math.sin(totalAngle), 
				0,
				firstCircleCenter.getZ() - 400 * Math.cos(totalAngle));
		
		// Get second key point
		deltaAngle = dubinsPath.type.params[2];
		if (dubinsString.substring(2, 3).equals("L")) deltaAngle *= -1;
		targetCubeVector = new Vector3f(
				target.getX() - secondCircleCenter.getX(), 
				0,
				target.getZ() - secondCircleCenter.getZ());
		centerTargetAngle = (float) Math.atan2(-targetCubeVector.x, -targetCubeVector.z);
		if (dubinsPath.type.identifier.toString().substring(2, 3).equals("L"))
			totalAngle = (float) (centerTargetAngle - deltaAngle);
		else 
			totalAngle = (float) (centerTargetAngle + deltaAngle);
		secondCircleKeyPoint = new Point3D(
				secondCircleCenter.getX() - 400 * Math.sin(totalAngle), 
				0,
				secondCircleCenter.getZ() - 400 * Math.cos(totalAngle));
	}
	
	
	// PID //
	
	/*
	 * The main-PID method using position
	 * 
	 * @return	Return the requested heading
	 */
	private float getPidPosition(Vector3f dronePosition, Vector3f rotations) {
		updatePositionError(dronePosition, rotations);

		if (ENABLE_LOGGING && printPID) {
			System.out.println("PID - POSITION");
			System.out.println("---proportional: "+proportionalPosition());
			System.out.println("---derivative: "+ 100 *derivativePosition());
		}
		
		// Return the drone its own heading with the requested direction
		float pidPositionFault = (float) (proportionalPosition() + 100*derivativePosition());
		if (pidPositionFault > 20) pidPositionFault = 20;
		else if (pidPositionFault < -20) pidPositionFault = -20;
		String direction = "S";
		switch (stage) {
		case 0:
			direction = dubinsPath.type.identifier.toString().substring(0,1);
			break;
		case 2:
			direction = dubinsPath.type.identifier.toString().substring(2,3);
			break;
		}
		
		if (ENABLE_LOGGING && printPID) System.out.println("---default pidPositionFault: "+pidPositionFault);
		
		if (direction.equals("L"))
			pidPositionFault += 11.5;
		else if (direction.equals("R"))
			pidPositionFault -= 11.5;
		
		return (float) (rotations.x + pidPositionFault*(Math.PI/180));
	}
	
	private float proportionalPosition() {
		return positionError;
	}
	
	private float derivativePosition() {
		return (float) Math.atan(positionError - prevPositionError);
	}
	
	/*
	 * The PID method using position when there is only one cube left
	 * 
	 * @return	Return the requested heading
	 */
	private float getPidPositionOneCube(Vector3f dronePosition, Vector3f rotations) {
		updatePositionErrorOneCube(dronePosition, rotations);

		if (ENABLE_LOGGING && printPID) {
			System.out.println("PID - POSITION ONE CUBE");
			System.out.println("---proportional: "+proportionalPosition());
			System.out.println("---derivative: "+ 100 *derivativePosition());
		}
		
		// Return the drone its own heading with the requested direction
		float pidPositionFault = (float) (proportionalPosition() + 100*derivativePosition());
		if (pidPositionFault > 20) pidPositionFault = 20;
		else if (pidPositionFault < -20) pidPositionFault = -20;
		
		if (ENABLE_LOGGING && printPID)
			System.out.println("---default pidPositionFault: "+pidPositionFault);
		
		Vector3f requestedVector = new Vector3f(
				target.getX() - dronePosition.x, 
				0, 
				target.getZ() - dronePosition.z);
		float deltaHeading = (float) (rotations.x - Math.atan2(-requestedVector.x, -requestedVector.z));
		if (deltaHeading > Math.PI) deltaHeading -= 2*Math.PI;
		else if (deltaHeading < -Math.PI) deltaHeading += 2*Math.PI;
		
		if (deltaHeading > Math.PI/36) return (float) (rotations.x - Math.PI/16);	// Roll of 33.75
		else if (deltaHeading < -Math.PI/36) return (float) (rotations.x + Math.PI/16);	// Roll of 33.75
		else return (float) (rotations.x + pidPositionFault*(Math.PI/180));	
	}
	
	
	// PITCH AND HEADING //
	
	private float getPitch(Vector3f dronePosition, Point3D target) {
		if (target != null) {
			float pitch;
			Vector3f requestedVector = new Vector3f(
					target.getX() - dronePosition.x, 
					target.getY() - dronePosition.y,
					target.getZ() - dronePosition.z);
			
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
	
	
	// HELPER PID DISTANCE
	
	private void updatePositionError(Vector3f dronePosition, Vector3f rotations) {
		prevPositionError = positionError;
		positionError = (float) (getDistanceFault(dronePosition));
	}
	
	private void updatePositionErrorOneCube(Vector3f dronePosition, Vector3f rotations) {
		prevPositionError = positionError;
		positionError = (float) (getDistanceFaultOneCube(dronePosition, target));

		if (ENABLE_LOGGING && printPID) {
			System.out.println("---positionError "+positionError);
			System.out.println("---prevPositionError "+prevPositionError);
		}
	}
	
	private float getDistanceFault(Vector3f dronePosition) {
		switch (stage) {
		case 0:
			return getDistanceFaultFirstCircle(dronePosition);
		case 1:
			return getDistanceFaultOneCube(dronePosition, secondCircleKeyPoint);
			//return getDistanceFaultStraightLine(dronePosition);
		case 2:
			return getDistanceFaultSecondCircle(dronePosition);	
		case 3:
			return getDistanceFaultStraightEndLine(dronePosition);
		default:
			return getDistanceFaultOneCube(dronePosition, target);
		}
	}
	
	private float getDistanceFaultFirstCircle(Vector3f dronePosition) {
		String direction = dubinsPath.type.identifier.toString().substring(0, 1);
		if (direction.equals("L"))
			return -(400 - distanceTo(firstCircleCenter, dronePosition));
		else
			return (400 - distanceTo(firstCircleCenter, dronePosition));
	}
	
	private float getDistanceFaultStraightLine(Vector3f dronePosition) {
		float numerator = (float) Math.abs((secondCircleKeyPoint.getZ() - firstCircleKeyPoint.getZ())*dronePosition.x - 
				(secondCircleKeyPoint.getX() - firstCircleKeyPoint.getX())*dronePosition.z + 
				secondCircleKeyPoint.getX()*firstCircleKeyPoint.getZ() - 
				secondCircleKeyPoint.getZ()*firstCircleKeyPoint.getX());
		float difference = (float) (numerator/distanceTo(firstCircleKeyPoint, secondCircleKeyPoint));
		
		float sign = (float) ((float) (dronePosition.x - firstCircleKeyPoint.getX()) * (secondCircleKeyPoint.getZ() - firstCircleKeyPoint.getZ()) - 
				(dronePosition.z - firstCircleKeyPoint.getZ()) * (secondCircleKeyPoint.getX() - firstCircleKeyPoint.getX()));
		if (sign > 0) return -difference;	// Drone on the right of the line
		else return difference;	// Drone on the left of the line
	}
	
	private float getDistanceFaultSecondCircle(Vector3f dronePosition) {
		String direction = dubinsPath.type.identifier.toString().substring(2, 3);
		if (direction.equals("L"))
			return -(400 - distanceTo(secondCircleCenter, dronePosition));
		else
			return (400 - distanceTo(secondCircleCenter, dronePosition));
	}
	
	private float getDistanceFaultStraightEndLine(Vector3f dronePosition) {
		float numerator = (float) Math.abs((secondTarget.getZ() - target.getZ())*dronePosition.x - 
				(secondTarget.getX() - target.getX())*dronePosition.z + 
				secondTarget.getX()*target.getZ() - 
				secondTarget.getZ()*target.getX());
		
		float difference = (float) (numerator/distanceTo(target, secondTarget));
		float sign = (float) ((float) (dronePosition.x - target.getX()) * (secondTarget.getZ() - target.getZ()) - 
				(dronePosition.z - target.getZ()) * (secondTarget.getX() - target.getX()));
		
		if (sign > 0) return -difference;	// Drone on the right of the line
		else return difference;	// Drone on the left of the line	
	}
	
	private float getDistanceFaultOneCube(Vector3f dronePosition, Point3D target) {
		if (dronePosStart == null || Math.abs(positionError) > maxFault)
			dronePosStart = new Point3D(dronePosition.x, dronePosition.y, dronePosition.z);
		
		float numerator = (float) Math.abs((target.getZ() - dronePosStart.getZ())*dronePosition.x - 
				(target.getX() - dronePosStart.getX())*dronePosition.z + 
				target.getX()*dronePosStart.getZ() - 
				target.getZ()*dronePosStart.getX());
		float difference = (float) (numerator/distanceTo(dronePosStart, target));
		
		float sign = (float) ((float) (dronePosition.x - dronePosStart.getX()) * (target.getZ() - dronePosStart.getZ()) - 
				(dronePosition.z - dronePosStart.getZ()) * (target.getX() - dronePosStart.getX()));
		if (sign > 0) return -difference;	// Drone on the right of the line
		else return difference;	// Drone on the left of the line
	}
	
	
	// HELPER METHODS //
	
	private boolean isReached(Point3D requestedPoint, Vector3f dronePosition, Vector3f rotations) {
		Vector3f targetPointVector = new Vector3f(
				requestedPoint.getX() - dronePosition.x, 
				0, 
				requestedPoint.getZ() - dronePosition.z);
		float reqHeading = (float) Math.atan2(-targetPointVector.x, -targetPointVector.z);
		double headingDifference = Math.min(Math.abs(rotations.x - reqHeading),
				Math.PI * 2 - Math.abs(rotations.x - reqHeading));
		if (Math.abs(rotations.z) < Math.PI/18)
			return (distanceTo(requestedPoint, dronePosition) < 20 || // Successfully reached
					(headingDifference > Math.PI/2 && distanceTo(requestedPoint, dronePosition) < 200));
		else
			return (distanceTo(requestedPoint, dronePosition) < 50 || // Successfully reached
				(headingDifference > Math.PI/2 && distanceTo(requestedPoint, dronePosition) < 200));	// Drone has past it
	}
	
	private boolean isReachedWide(Point3D requestedPoint, Vector3f dronePosition, Vector3f rotations) {
		Vector3f targetPointVector = new Vector3f(
				requestedPoint.getX() - dronePosition.x, 
				0, 
				requestedPoint.getZ() - dronePosition.z);
		float reqHeading = (float) Math.atan2(-targetPointVector.x, -targetPointVector.z);
		double headingDifference = Math.min(Math.abs(rotations.x - reqHeading),
				Math.PI * 2 - Math.abs(rotations.x - reqHeading));
		
		return (distanceTo(requestedPoint, dronePosition) < 100 || // Successfully reached
				(headingDifference > Math.PI/2 && distanceTo(requestedPoint, dronePosition) < 200));	// Drone has past it
	}

	private float distanceTo(Point3D point, Point3D position) {
		return (float) Math.sqrt(Math.pow(point.getX() - position.getX(), 2) + Math.pow(point.getZ() - position.getZ(), 2));
	}

	private float distanceTo(Point3D point, Vector3f position) {
		return (float) Math.sqrt(Math.pow(point.getX() - position.x, 2) + Math.pow(point.getZ() - position.z, 2));
	}
	
	public void resetAll(Vector3f dronePosition) {
		dubinsInitialized = false;
		stage = 0;
		if (targetCoordinates.size() > 1) {
			if (distanceTo(targetCoordinates.get(0), dronePosition) < 100) {
				if (distanceTo(targetCoordinates.get(1), dronePosition) > 1000) {
					reordenPath = true;
				} else {
					reordenPath = false;
				}
			} else {
				if (distanceTo(targetCoordinates.get(0), dronePosition) > 1000) {
					reordenPath = true;
				} else {
					reordenPath = false;
				}
			}
		} else {
			reordenPath = false;
		}
	}
	
	public void crashResetAll() {
		dubinsInitialized = false;
		stage = 0;
		reordenPath = true;
	}

}
