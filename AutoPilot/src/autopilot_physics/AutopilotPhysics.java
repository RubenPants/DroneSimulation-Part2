package autopilot_physics;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketTimeoutException;

import autopilot_utilities.Vector3f;
import autopilot_utilities.Vector4f;
import autopilot_utilities.Matrix4f;

import interfaces.AutopilotInputs;
import interfaces.AutopilotOutputs;

/**
 * A class of physical models for autopiloting a drone.
 */
public class AutopilotPhysics {
	
	private String phoneIp;
	
	public void setPhoneIp(String phoneIp) {
		this.phoneIp = phoneIp;
	}

	/**
	 * Variable denoting whether or not this class logs information.
	 */
	public boolean ENABLE_LOGGING;
	private boolean POSITION_LOGGING = false;
	private boolean DRONE_STAT_LOGGING = false;
	private boolean ERROR_LOGGING = false;
	private boolean ESSENTIAL_LOGGING = false;

	public static enum AutopilotPhysicsStage {
		FLY, TAXI, LAND, TAKE_OFF, PHONE
	}

	// *** VARIABLES *** //
	private final float gravity;
	private final float wingX;
	private final float tailSize;
	private final float engineMass;
	private final float wingMass;
	private final float tailMass;
	private final float maxThrust;
	private final float maxAOA;
	private final float wingLiftSlope;
	private final float horStabSlope;
	private final float verStabSlope;
	private final float engineZ;

	private Matrix4f toDrone;
	private Matrix4f toWorld;
	private Vector3f projAirSpeed;
	private Vector3f projGravity;
	private Vector3f travelVector;

	private float sSquared;
	private float requestedSSquared;
	private float requestedSpeedFactor = 1;
	private float averageWingInclination;
	private float adjustInclination;
	private float thrust;
	private float AOA;
	
	private float maxRoll = (float) (Math.PI / 4); // 45 degrees
	private float maxAdjustInclination = (float) (Math.PI / 90); // 2 degrees
	private int maxFrontBrake;
	private int maxBackBrake;
	private float requestedHeading;
	private float requestedPitch;
	private float frontBrake;
	private float rightBrake;
	private float leftBrake;
	private boolean isLanded = false;
	private boolean isMaxRoll;

	private float x = 0;
	private float y = 0;
	private float z = 0;
	private float heading = 0;
	private float pitch = 0;
	private float roll = 0;
	private float previousRoll = 0;
	private float previousTimeElapsed = -0.017f;
	private float timeElapsed = 0;
	private float deltaTimeElapsed = 0.012f;
	private String stage = "O";
	private Vector3f airSpeed = new Vector3f(0, 0, (float) -Math.sqrt(requestedSSquared));

	DatagramSocket serverSocket;
	byte[] receiveData;
	float droneX;
	float droneY;
	float droneZ;

	
	// *** CONSTRUCTOR *** //

	public AutopilotPhysics(float gravity, float wingX, float tailSize, float engineMass, float wingMass,
			float tailMass, float maxThrust, float maxAOA, float wingLiftSlope, float horStabSlope,
			float verStabSlope, float RMax) {
		super();
		this.gravity = gravity;
		this.wingX = wingX;
		this.tailSize = tailSize;
		this.engineMass = engineMass;
		this.wingMass = wingMass;
		this.tailMass = tailMass;
		this.maxThrust = maxThrust;
		// The most optimal maxAOA = 0.86rad. 0.8f is chosen for certainty.
		if (maxAOA > 0.80f)
			this.maxAOA = 0.80f;
		else
			this.maxAOA = maxAOA;
		this.wingLiftSlope = wingLiftSlope;
		this.horStabSlope = horStabSlope;
		this.verStabSlope = verStabSlope;
		this.engineZ = calculateEnginePos(tailMass, tailSize, engineMass);
		this.requestedSSquared = (float) Math.abs((1.2 * gravity * getTotalMass())
				/ (maxAOA * Math.cos(maxAOA / 2) * wingLiftSlope * Math.cos(roll) * Math.cos(pitch)));
		if (RMax > 1500) {
			maxFrontBrake = 1000;
			maxBackBrake = 1500;
		} else {
			maxFrontBrake = (int) RMax;
			maxBackBrake = (int) RMax;
		}
	}

	public void getInputsStart(AutopilotInputs inputs) {
		// Not used
	}

	public void getInputs(AutopilotInputs inputs) {
		float currentX = inputs.getX();
		float currentY = inputs.getY();
		float currentZ = inputs.getZ();
		droneX = inputs.getX();
		droneY = inputs.getY();
		droneZ = inputs.getZ();

		heading = inputs.getHeading();
		pitch = inputs.getPitch();
		previousRoll = roll;
		roll = inputs.getRoll();
		timeElapsed = inputs.getElapsedTime();
		deltaTimeElapsed = (timeElapsed - previousTimeElapsed);
		previousTimeElapsed = timeElapsed;
		if (deltaTimeElapsed == 0.0) {
			deltaTimeElapsed = 0.017f;
		}
		airSpeed = autopilot_utilities.Utilities.scaleVector(
				Vector3f.sub(new Vector3f(currentX, currentY, currentZ), new Vector3f(x, y, z), null),
				(float) (1.0 / deltaTimeElapsed));

		x = currentX;
		y = currentY;
		z = currentZ;

		requestedSSquared = (float) Math.abs((1.2 * gravity * getTotalMass())
				/ (maxAOA * Math.cos(maxAOA / 2) * wingLiftSlope * Math.cos(roll) * Math.cos(pitch)));

		if (pitch > -Math.PI/90 || y > 200)
			isMaxRoll = true;
		if (!isMaxRoll || (y < 50 && pitch < 0) || (y < 100 && pitch < -Math.PI/30 && requestedPitch > 0) || 
				pitch < -Math.PI/18) {
			isMaxRoll = false;
			maxRoll = (float) Math.PI/6;	// 30 degrees
		} else {
			maxRoll = (float) Math.PI/4;	// 60 degrees
		}
		
		if (ENABLE_LOGGING) {
			System.out.println("_____________________________________________________");
			System.out.println("");
			switch (stage) {
			case "O":
				System.out.println("STAGE: TAKE_OFF");
				break;
			case "L":
				System.out.println("STAGE: LAND");
				break;
			case "T":
				System.out.println("STAGE: TAXI");
				break;
			default:
				System.out.println("STAGE: FLY");
				break;
			}
		}

		if (ENABLE_LOGGING && POSITION_LOGGING) {
			System.out.println("POSITION: ");
			System.out.println("---X: " + x);
			System.out.println("---Y: " + y);
			System.out.println("---Z: " + z);
			System.out.println("---Time elapsed: " + deltaTimeElapsed);
		}
	}

	public AutopilotOutputs output(float reqPitch, float reqHeading, Vector3f reqPosition,
			AutopilotPhysicsStage stage) {
		if (ENABLE_LOGGING && DRONE_STAT_LOGGING)
			System.out.println("DRONE STATS:");
		
		toDrone = getWorldToDroneTransformationMatrix(heading, pitch, roll);
		toWorld = getDroneToWorldTransformationMatrix(heading, pitch, roll);

		float gY = getTotalMass() * gravity;
		projGravity = transformVector(toDrone, new Vector3f(0, -gY, 0));

		projAirSpeed = transformVector(toDrone, airSpeed);
		sSquared = (float) projAirSpeed.lengthSquared();

		if (Math.abs(reqHeading) < 10 && Math.abs(reqPitch) < 10) {
			requestedHeading = reqHeading;
			requestedPitch = reqPitch;
		}

		if (reqPosition != null) // Ignore y
			travelVector = new Vector3f(reqPosition.x - x, 0, reqPosition.z - z);

		switch (stage) {
		case FLY:
			configFly();
			break;
		case TAXI:
			configTaxi();
			break;
		case LAND:
			configLand();
			break;
		case TAKE_OFF:
			configTakeOff();
			break;
		case PHONE:
			configPhone();
			break;
		}

		float[] leftAndRightInclination;
		leftAndRightInclination = getLeftAndRightAdjIncl(averageWingInclination, adjustInclination, wingLiftSlope,
				sSquared);

		if (ENABLE_LOGGING && DRONE_STAT_LOGGING) {
			System.out.println("---pitch: " + Math.toDegrees(pitch));
			System.out.println("---requestedPitch: " + Math.toDegrees(requestedPitch));
			System.out.println("---heading: " + Math.toDegrees(heading));
			System.out.println("---requestedHeading: " + Math.toDegrees(requestedHeading));
			System.out.println("---roll: " + Math.toDegrees(roll));
			System.out.println("---inclination: " + Math.toDegrees(averageWingInclination));
			System.out.println("---adjustInclination: " + Math.toDegrees(adjustInclination));
			System.out.println("---leftIncl: " + Math.toDegrees(leftAndRightInclination[0]));
			System.out.println("---rightIncl: " + Math.toDegrees(leftAndRightInclination[1]));
			System.out.println("---sSquared: " + sSquared);
			System.out.println("---requestedSSquared: " + requestedSSquared);
			System.out.println("---thrust: " + thrust);
			if (thrust == maxThrust)
				System.out.println("------maxThrust reached");
			System.out.println("---frontBrake: " + frontBrake);
			System.out.println("---leftBrake: " + leftBrake);
			System.out.println("---rightBrake: " + rightBrake);
		} else if (ENABLE_LOGGING && ESSENTIAL_LOGGING) {
			System.out.println("DRONE STATS - ESSENTIALS:");
			System.out.println("---pitch: " + Math.toDegrees(pitch));
			System.out.println("---requestedPitch: " + Math.toDegrees(requestedPitch));
			System.out.println("---heading: " + Math.toDegrees(heading));
			System.out.println("---requestedHeading: " + Math.toDegrees(requestedHeading));
			System.out.println("---sSquared: " + sSquared);
			System.out.println("---requestedSSquared: " + requestedSSquared);
		}

		return new AutopilotOutputs() {
			@Override
			public float getRightWingInclination() {
				return leftAndRightInclination[1];
			}

			@Override
			public float getLeftWingInclination() {
				return leftAndRightInclination[0];
			}

			@Override
			public float getHorStabInclination() {
				return 0;
			}

			@Override
			public float getVerStabInclination() {
				return 0;
			}

			@Override
			public float getThrust() {
				return thrust;
			}

			@Override
			public float getFrontBrakeForce() {
				return frontBrake;
			}

			@Override
			public float getLeftBrakeForce() {
				return leftBrake;
			}

			@Override
			public float getRightBrakeForce() {
				return rightBrake;
			}
		};
	}

	// Stage methods
	private void configFly() {
		stage = "F";
		frontBrake = 0;
		rightBrake = 0;
		leftBrake = 0;
		
		if (Math.abs(roll) > Math.PI/4.5)	// 40 degrees
			requestedPitch += Math.PI / 20;	// 9 degrees
		else if (Math.abs(roll) > Math.PI / 5) // 36 degrees
			requestedPitch += Math.PI / 36; // 5 degrees
		else if (Math.abs(roll) > Math.PI / 9) // 20 degrees
			requestedPitch += Math.PI / 45; // 4 degrees
		else if (Math.abs(roll) > Math.PI / 18) // 10 degrees
			requestedPitch += Math.PI / 90; // 2 degree
		
		// Make sure that the requestedPitch is reasonable
		if (requestedPitch > Math.PI / 12) 	// 15
			requestedPitch = (float) (Math.PI / 12);
		else if (requestedPitch > Math.PI / 18 && sSquared < 3.0 / 4.0 * requestedSSquared) // 10
			requestedPitch = (float) (Math.PI / 18);
		else if (requestedPitch < -Math.PI / 12) // -15
			requestedPitch = (float) (-Math.PI / 12);

		// Try to make plane crash-proof
		if (y < 20)
			requestedPitch = (float) Math.PI / 36;
		else if (sSquared > 3 * requestedSSquared && requestedPitch < -Math.PI / 90 && y < 50)
			requestedPitch = (float) Math.PI / 90;
		else if (sSquared > 2 * requestedSSquared && pitch < -Math.PI / 360
				&& y < 10.0 * (1.0 + sSquared / requestedSSquared))
			requestedPitch = (float) Math.PI / 36;
		else if (y < 20f && pitch < (float) -Math.PI / 90)
			requestedPitch = (float) Math.PI / 90;
		else if (y < 15f && pitch < (float) -Math.PI / 90)
			requestedPitch = (float) Math.PI / 36;
		else if (sSquared > 3 * requestedSSquared && requestedPitch < -Math.PI / 36)
			requestedPitch = (float) -Math.PI / 36;

		if (sSquared < requestedSSquared * (2.0 / 3.0) && requestedPitch > Math.PI / 18 && pitch > Math.PI / 18
				&& requestedPitch > (pitch - Math.PI / 180.0))
			requestedPitch = (float) (pitch - Math.PI / 180.0); // 1 degree

		averageWingInclination = findInclinationZeroForceDroneY(wingLiftSlope, sSquared, projGravity, requestedPitch);
		checkAverageWingInclinationMaxAOA();
		
		thrust = findThrustZeroForceDroneZ(averageWingInclination, wingLiftSlope, sSquared, projGravity);
		adjustInclination = adjustInclinationToMatchHeading(averageWingInclination, requestedHeading, wingLiftSlope,
				sSquared);
	}

	private void configTaxi() {
		stage = "T";
		
		// Handle the requestedHeading
		float deltaHeading = (requestedHeading - heading);
		if (deltaHeading > Math.PI)	deltaHeading -= 2 * Math.PI;
		else if (deltaHeading < -Math.PI) deltaHeading += 2 * Math.PI;

		// Handle brakeFactor with deltaDistance
		float distanceToPoint = 100;
		if (travelVector != null)
			distanceToPoint = (float) travelVector.length();
		
		if ((distanceToPoint < 3 && y < 5) || isLanded) {
			if (ENABLE_LOGGING) System.out.println("LANDED");
			if (sSquared < 100) isLanded = true;
			else isLanded = false;
			requestedSSquared = 0;
		} else if (distanceToPoint < 100)
			requestedSSquared = 2 * distanceToPoint;
		else
			requestedSSquared = 200; // Max taxi speed of 51 km/h

		float brakeFactor = (float) ((sSquared - requestedSSquared) / (0.5 * requestedSSquared));
		if (brakeFactor > 1)
			brakeFactor = 1;
		if (ENABLE_LOGGING && DRONE_STAT_LOGGING)
			System.out.println("---brakeFactor: " + brakeFactor);
		if ((requestedSSquared < 0.01)) {
			frontBrake = maxFrontBrake;
			rightBrake = maxBackBrake;
			leftBrake = maxBackBrake;
		} else if (brakeFactor > 0) {
			frontBrake = brakeFactor * maxFrontBrake;
			rightBrake = brakeFactor * maxBackBrake;
			leftBrake = brakeFactor * maxBackBrake;
		} else {
			frontBrake = 0;
			rightBrake = 0;
			leftBrake = 0;
		}

		if (deltaHeading > 0.01) { // 0.57 degrees
			leftBrake *= 2;
			if (leftBrake < 50 + 5 * sSquared) {
				if (Math.abs(deltaHeading) < Math.PI / 36) // 5 degrees
					leftBrake = (float) (50 + 5 * Math.abs(deltaHeading * 36 / Math.PI) * sSquared);
				else leftBrake = 150 + 5 * sSquared;
			}
		} else if (deltaHeading < -0.01) {
			rightBrake *= 2;
			if (rightBrake < 50 + 5 * sSquared) {
				if (Math.abs(deltaHeading) < Math.PI / 36) // 5 degrees
					rightBrake = (float) (50 + 5 * Math.abs(deltaHeading * 36 / Math.PI) * sSquared);
				else rightBrake = 150 + 5 * sSquared;
			}
		}

		if (frontBrake > maxFrontBrake) frontBrake = maxFrontBrake;
		if (rightBrake > maxBackBrake) rightBrake = maxBackBrake;
		if (leftBrake > maxBackBrake) leftBrake = maxBackBrake;

		requestedPitch = 0;
		averageWingInclination = 0;
		thrust = (float) (findThrustZeroForceDroneZ(averageWingInclination, wingLiftSlope, sSquared, projGravity));
		adjustInclination = 0;
	}

	private void configLand() {
		stage = "L";

		if (y > 50f) {
			requestedPitch = (float) -Math.PI / 18;
			requestedSSquared = 1500f;
		} else {
			if (y > 20) {
				requestedPitch = (float) -Math.PI / 36;
				requestedSSquared = 1500f;
			} else if (y > 15) {
				requestedPitch = (float) -Math.PI / 90;
				requestedSSquared = 1500f;
			} else if (y > 5f) {
				requestedHeading = heading;
				requestedPitch = (float) -Math.PI / 180;
				requestedSSquared = 1300f;
			} else {
				requestedHeading = heading;
				requestedPitch = 0;
				requestedSSquared = 0;
				frontBrake = maxFrontBrake;
				rightBrake = maxBackBrake;
				leftBrake = maxBackBrake;
			}
		}

		averageWingInclination = findInclinationZeroForceDroneY(wingLiftSlope, sSquared, projGravity, requestedPitch);
		checkAverageWingInclinationMaxAOA();
		
		thrust = findThrustZeroForceDroneZ(averageWingInclination, wingLiftSlope, sSquared, projGravity);
		adjustInclination = adjustInclinationToMatchHeading(averageWingInclination, requestedHeading, wingLiftSlope,
				sSquared);
	}

	private void configTakeOff() {
		stage = "O";

		if (sSquared <= requestedSSquared) {
			frontBrake = 0;
			rightBrake = 0;
			leftBrake = 0;
		}

		requestedHeading = heading;
		if (sSquared > 2.0 / 3.0 * requestedSSquared) {
			if (sSquared > 9.0/10.0 * requestedSSquared)
				requestedPitch = (float) Math.PI / 18; // 10 degrees
			else 
				requestedPitch = (float) Math.PI / 36; // 5 degrees
			averageWingInclination = findInclinationZeroForceDroneY(wingLiftSlope, sSquared, projGravity,
					requestedPitch);
			checkAverageWingInclinationMaxAOA();
		} else {
			requestedPitch = 0;
			averageWingInclination = 0;
		}
		
		thrust = findThrustZeroForceDroneZ(averageWingInclination, wingLiftSlope, sSquared, projGravity);
		adjustInclination = adjustInclinationToMatchHeading(averageWingInclination, requestedHeading, wingLiftSlope,
				sSquared);
	}

	private void configPhone() {
		try {
			if (serverSocket == null) {
				serverSocket = new DatagramSocket(9876);
				receiveData = new byte[128];
				System.out.println("Setup server on port 9876");
			}
			DatagramPacket receivePacket = new DatagramPacket(receiveData, 10);
			serverSocket.setSoTimeout(1);
			try {
				while (true) {
					serverSocket.receive(receivePacket);
					String sentence = new String(receivePacket.getData());
					if (ENABLE_LOGGING) System.out.println("INPUT SENTENCE: "+sentence);
					int receivedPitch = Integer.parseInt(sentence.substring(0, 3));
					requestedPitch = (float) (Math.toRadians(receivedPitch) / 4.0);
					int receivedRoll = Integer.parseInt(sentence.substring(3, 6));
					requestedHeading = (float) (heading + Math.toRadians(receivedRoll) / 4.0);
					requestedSpeedFactor = (float) (Integer.parseInt(sentence.substring(6, 9)) / 100.0);
					stage = sentence.substring(9, 10);
				}
			} catch (SocketTimeoutException ignore) {
				/*
				requestedPitch = pitch;
				requestedHeading = heading;
				*/
			}
		} catch (IOException e) {
			e.printStackTrace();
		}

		// Re-scale the requestedSSquared
		requestedSSquared *= requestedSpeedFactor;

		switch (stage) {
		case "O":
			configTakeOff();
			break;
		case "L":
			configLand();
			break;
		case "T":
			configTaxi();
			break;
		default:
			configFly();
			break;
		}		
	}

	
	// *** HELPER METHODS *** //
	
	public void checkAverageWingInclinationMaxAOA() {
		Vector3f axis = new Vector3f(1, 0, 0);
		Vector3f attackVector = new Vector3f(0, (float) sin(averageWingInclination + maxAdjustInclination),
				-(float) cos(averageWingInclination + maxAdjustInclination));
		Vector3f normal = Vector3f.cross(axis, attackVector, null);
		AOA = (float) -Math.atan2(Vector3f.dot(projAirSpeed, normal), Vector3f.dot(projAirSpeed, attackVector));
		while (Math.abs(AOA) >= Math.abs(maxAOA) * 0.99f && Math.abs(averageWingInclination) > 0.01) {
			if (AOA > 0)
				averageWingInclination -= 0.01f; // 0.01 rad equals 0.57 degrees
			else
				averageWingInclination += 0.01f;

			if (averageWingInclination > maxAOA)
				averageWingInclination = -maxAOA;
			else if (averageWingInclination < -maxAOA)
				averageWingInclination = maxAOA;

			attackVector = new Vector3f(0, (float) sin(averageWingInclination + maxAdjustInclination),
					-(float) cos(averageWingInclination + maxAdjustInclination));
			AOA = (float) -Math.atan2(Vector3f.dot(projAirSpeed, normal), Vector3f.dot(projAirSpeed, attackVector));
		}
	}

	float[] getLeftAndRightAdjIncl(float averageInclination, float adjustInclination, float liftSlopeConstant,
			float sSquared) {
		float EPSILON = 0.000001f;
		float leftIncl = (averageInclination - adjustInclination);
		float rightIncl = (averageInclination + adjustInclination);
		float initForceWings = 2 * wingLift(averageInclination, liftSlopeConstant, sSquared);

		float maxAdjIncl = Math.abs(maxAOA - Math.abs(averageInclination));
		if (maxAdjIncl > 0.25f)
			maxAdjIncl = 0.25f;

		float a = -maxAdjIncl;
		float fa = wingLift(leftIncl + a, liftSlopeConstant, sSquared)
				+ wingLift(rightIncl + a, liftSlopeConstant, sSquared) - initForceWings;
		float b = maxAdjIncl;
		float fb = wingLift(leftIncl + b, liftSlopeConstant, sSquared)
				+ wingLift(rightIncl + b, liftSlopeConstant, sSquared) - initForceWings;

		if (fa * fb > 0) {
			return new float[] { leftIncl, rightIncl };
		}

		while ((b - a) > EPSILON) {
			if ((wingLift(leftIncl + (b + a) / 2, liftSlopeConstant, sSquared)
					+ wingLift(rightIncl + (b + a) / 2, liftSlopeConstant, sSquared) - initForceWings) > 0)
				b = (b + a) / 2;
			else
				a = (b + a) / 2;
		}

		return new float[] { leftIncl + (b + a) / 2, rightIncl + (b + a) / 2 };
	}

	private float adjustInclinationToMatchHeading(float averageInclination, float requestedHeading,
			float liftSlopeConstant, float sSquared) {
		float angularVelocity = (roll - previousRoll) / deltaTimeElapsed;
		float requestedRoll;
		float requestedAngularVelocity;

		// Decide which is the fastest way to go to the requested heading
		float deltaHeading = (requestedHeading - heading);
		if (deltaHeading > Math.PI) deltaHeading -= 2 * Math.PI;
		else if (deltaHeading < -Math.PI) deltaHeading += 2 * Math.PI;

		float deltaHeadingFactor = (float) ((1.0/15.0) * (deltaHeading * 180 / Math.PI));
		
		if (deltaHeadingFactor > 1)	deltaHeadingFactor = 1;
		else if (deltaHeadingFactor < -1) deltaHeadingFactor = -1;

		requestedRoll = (float) maxRoll * deltaHeadingFactor;
		if (requestedRoll > maxRoll) requestedRoll = maxRoll;
		else if (requestedRoll < -maxRoll) requestedRoll = maxRoll;

		requestedAngularVelocity = (float) ((requestedRoll - roll) / (20 * deltaTimeElapsed));
		if (requestedAngularVelocity > 0.5) requestedAngularVelocity = (float) (0.5);
		else if (requestedAngularVelocity < -0.5) requestedAngularVelocity = (float) (-0.5);

		if (ENABLE_LOGGING && DRONE_STAT_LOGGING) {
			System.out.println("---requestedAngularVelocity: " + requestedAngularVelocity);
			System.out.println("---requestedRoll: " + Math.toDegrees(requestedRoll));
		}

		// Stabilize towards the requestedAngularVelocity
		return findInclinationAngularVelocity(angularVelocity, requestedAngularVelocity,
				averageInclination, sSquared, (requestedHeading - heading));
	}

	private float findInclinationAngularVelocity(float angularVelocity, float requestedAngularVelocity,
			float averageInclination, float sSquared, float deltaHeading) {
		float EPSILON = 0.000001f;

		float a = -(maxAOA - Math.abs(averageInclination));
		float Fa = forceEquationAdjustInclination(averageInclination, a, sSquared, angularVelocity,
				requestedAngularVelocity);
		float b = (maxAOA - Math.abs(averageInclination));
		float Fb = forceEquationAdjustInclination(averageInclination, b, sSquared, angularVelocity,
				requestedAngularVelocity);

		if (Fa * Fb > 0) {
			if (requestedAngularVelocity > angularVelocity)
				return maxAdjustInclination;
			else
				return -maxAdjustInclination;
		}

		while ((b - a) > EPSILON) {
			if (forceEquationAdjustInclination(averageInclination, (b + a) / 2, sSquared, angularVelocity,
					requestedAngularVelocity) > 0)
				b = (b + a) / 2;
			else
				a = (b + a) / 2;
		}

		float result = (b + a) / 2;

		if (result > maxAdjustInclination) return maxAdjustInclination;
		else if (result < -maxAdjustInclination) return -maxAdjustInclination;
		else return result;
	}

	private float forceEquationAdjustInclination(float averageInclination, float adjustInclination, float sSquared,
			float angularVelocity, float requestedAngularVelocity) {
		// The minus sign is because the direction of the forces are opposite
		return (float) wingLift(averageInclination + adjustInclination, wingLiftSlope, sSquared) - 
				wingLift(averageInclination - adjustInclination, wingLiftSlope, sSquared)
				- (requestedAngularVelocity - angularVelocity) * getWingMass() * getWingX() / deltaTimeElapsed;
	}

	private float findInclinationZeroForceDroneY(float liftSlopeConstant, float sSquared, Vector3f projGravity,
			float requestedPitch) {
		float EPSILON = 0.000001f;

		float elevationFactor = (float) (Math.pow((requestedPitch - pitch) / (Math.PI / 60), 3));

		if (elevationFactor < -1) elevationFactor = -1;
		else if (elevationFactor > 1) elevationFactor = 1;

		if (ENABLE_LOGGING && DRONE_STAT_LOGGING)
			System.out.println("---elevationFactor: " + elevationFactor);

		float a = 0;
		float fa = totalForceDroneY(a, liftSlopeConstant, sSquared);
		float b = maxAOA;
		float fb = totalForceDroneY(b, liftSlopeConstant, sSquared);

		if (fa * fb > 0) {
			a = -maxAOA;
			fa = totalForceDroneY(a, liftSlopeConstant, sSquared);
			b = 0;
			fb = totalForceDroneY(b, liftSlopeConstant, sSquared);

			if (elevationFactor == -1) {
				if (ENABLE_LOGGING && ERROR_LOGGING) System.out.println("ERROR: FREE-FALL");
				return 0;
			} else if (fa * fb > 0) { // i.e. the drone's velocity is to low
				if (ENABLE_LOGGING && ERROR_LOGGING)
					System.out.println("ERROR: NO ZERO FOUND! (inclination)");
				if (requestedPitch == 0) { 
					// In this case, the method is in its second iteration, and still failed
					if (ENABLE_LOGGING && ERROR_LOGGING) System.out.println("ERROR: FREE-FALL");
					return 0;
				} else
					return findInclinationZeroForceDroneY(liftSlopeConstant, sSquared, projGravity, 0);
			}
		}

		while ((b - a) > EPSILON) {
			if (totalForceDroneY((b + a) / 2, liftSlopeConstant, sSquared) > 0)
				b = (b + a) / 2;
			else a = (b + a) / 2;
		}

		return adjustInclinationPitch((a + b) / 2, elevationFactor, liftSlopeConstant, sSquared);
	}

	private float adjustInclinationPitch(float defaultInclination, float elevationFactor, float liftSlopeConstant,
			float sSquared) {
		float EPSILON = 0.000001f;

		float defaultLift = wingLift(defaultInclination, liftSlopeConstant, sSquared);

		float requestedLift;
		if (Math.abs(pitch - requestedPitch) < Math.PI / 36) // 5 degrees
			requestedLift = (float) (defaultLift + getTotalMass() * gravity * elevationFactor / 5.0);
		else
			requestedLift = (float) (defaultLift + getTotalMass() * gravity * elevationFactor / 3.0);

		float a;
		float b;
		if (elevationFactor > 0) {
			a = defaultInclination;
			b = maxAOA;
		} else {
			a = -maxAOA;
			b = defaultInclination;
		}

		float Fa = wingLift(a, liftSlopeConstant, sSquared) - requestedLift;
		float Fb = wingLift(b, liftSlopeConstant, sSquared) - requestedLift;
		if (Fa * Fb > 0) { // The target can't be reached
			return 0;
		}

		while ((b - a) > EPSILON) {
			if (wingLift((b + a) / 2, liftSlopeConstant, sSquared) - requestedLift > 0)
				b = (b + a) / 2;
			else
				a = (b + a) / 2;
		}

		return (b + a) / 2;
	}

	private float findThrustZeroForceDroneZ(float inclinationWings, float liftSlopeConstant, float sSquared,
			Vector3f projGravity) {
		float thrust = (float) (2 * inclinationWings * Math.sin(inclinationWings) * liftSlopeConstant * sSquared
				+ projGravity.z);
		float requestedAcceleration;
		float deltaThrust;

		if (deltaTimeElapsed == 0)
			requestedAcceleration = 0;
		// The factor 3 is there because the requestedAcceleration will be
		// reached in 20 frames assuming 60 FPS (so in 0.33 seconds)
		else
			requestedAcceleration = (requestedSSquared - sSquared) * deltaTimeElapsed * 3;

		deltaThrust = requestedAcceleration * getTotalMass();
		thrust += deltaThrust;

		if (thrust > maxThrust) return maxThrust;
		else if (thrust < 0) return 0;
		else return thrust;
	}

	private float wingLift(float inclination, float liftSlopeConstant, float sSquared) {
		return (float) (inclination * Math.cos(inclination) * liftSlopeConstant * sSquared);
	}

	private float totalForceDroneY(float inclination, float liftSlopeConstant, float sSquared) {
		Matrix4f toDrone = getWorldToDroneTransformationMatrix(heading, pitch, roll);

		float wingLiftY = (float) 2 * wingLift(inclination, liftSlopeConstant, sSquared);

		Vector3f gravityVector = transformVector(toDrone, new Vector3f(0, -getTotalMass() * gravity, 0));
		float gravityY = (float) (gravityVector.y);

		return (wingLiftY + gravityY);
	}

	private float getTotalMass() {
		return (engineMass + 2 * wingMass + tailMass);
	}

	private float getWingMass() {
		return wingMass;
	}

	private float getWingX() {
		return wingX;
	}

	private float calculateEnginePos(float tailMass, float tailSize, float engineMass) {
		return -(tailMass * tailSize) / engineMass;
	}

	private Matrix4f getWorldToDroneTransformationMatrix(float heading, float pitch, float roll) {
		return Matrix4f.invert(getDroneToWorldTransformationMatrix(heading, pitch, roll), null);
	}

	private Matrix4f getDroneToWorldTransformationMatrix(float heading, float pitch, float roll) {
		Matrix4f matrix = new Matrix4f();
		matrix.setIdentity();

		Matrix4f.rotate((float) heading, new Vector3f(0, 1, 0), matrix, matrix);
		Matrix4f.rotate((float) pitch, new Vector3f(1, 0, 0), matrix, matrix);
		Matrix4f.rotate((float) roll, new Vector3f(0, 0, 1), matrix, matrix);

		return matrix;
	}

	private Vector3f transformVector(Matrix4f matrix, Vector3f vector) {
		Vector4f vector4 = new Vector4f(vector.x, vector.y, vector.z, 1);
		Matrix4f.transform(matrix, vector4, vector4);

		return new Vector3f(vector4.x, vector4.y, vector4.z);
	}

	public static void printVector(String name, Vector3f vector) {
		System.out.println(name + ": [" + vector.x + ", " + vector.y + ", " + vector.z + "]");
	}

	public float getSSquared() {
		return sSquared;
	}
	

	// *** UNUSED HELPER METHODS *** //

	private float findInclinationZeroForceY(float thrust, float liftSlopeConstant, float sSquared,
			Vector3f projGravity) {
		return findInclinationZeroForceDroneY(liftSlopeConstant, sSquared, projGravity, 0.1f);
	}

	private float totalForceX(float inclination, float thrust, float liftSlopeConstant, float sSquared) {
		Matrix4f toWorld = getDroneToWorldTransformationMatrix(heading, pitch, roll);

		Vector3f thrustUnitVector = transformVector(toWorld, new Vector3f(0, 0, -1));
		float thrustX = (float) (thrust * thrustUnitVector.x);

		Vector3f wingNormalUnitVector = transformVector(toWorld,
				new Vector3f(0, (float) Math.cos(inclination), (float) Math.sin(inclination)));
		float wingLiftX = (float) ((2 * inclination * liftSlopeConstant * sSquared) * wingNormalUnitVector.x);

		return (thrustX + wingLiftX);
	}

	private float totalForceY(float inclination, float thrust, float liftSlopeConstant, float sSquared) {
		Matrix4f toWorld = getDroneToWorldTransformationMatrix(heading, pitch, roll);

		Vector3f thrustUnitVector = transformVector(toWorld, new Vector3f(0, 0, -1));
		float thrustY = (float) (thrust * thrustUnitVector.y);

		Vector3f wingNormalUnitVector = transformVector(toWorld,
				new Vector3f(0, (float) Math.cos(inclination), (float) Math.sin(inclination)));
		float wingLiftY = (float) ((2 * inclination * liftSlopeConstant * sSquared) * wingNormalUnitVector.y);

		float gravityY = (-getTotalMass() * gravity);

		return (thrustY + wingLiftY + gravityY);
	}

	private float totalForceZ(float inclination, float thrust, float liftSlopeConstant, float sSquared) {
		Matrix4f toWorld = getDroneToWorldTransformationMatrix(heading, pitch, roll);

		Vector3f thrustUnitVector = transformVector(toWorld, new Vector3f(0, 0, -1));
		float thrustZ = (float) (thrust * thrustUnitVector.z);

		Vector3f wingNormalUnitVector = transformVector(toWorld,
				new Vector3f(0, (float) Math.cos(inclination), (float) Math.sin(inclination)));
		float wingLiftZ = (float) ((2 * inclination * liftSlopeConstant * sSquared) * wingNormalUnitVector.z);

		return (thrustZ + wingLiftZ);
	}

	private float totalForceDroneX() {
		Matrix4f toDrone = getWorldToDroneTransformationMatrix(heading, pitch, roll);

		Vector3f gravityVector = transformVector(toDrone, new Vector3f(0, -getTotalMass() * gravity, 0));

		return (float) gravityVector.x;
	}

	private float totalForceDroneZ(float inclinationWings, float thrust, float liftSlopeConstant, float sSquared) {
		Matrix4f toDrone = getWorldToDroneTransformationMatrix(heading, pitch, roll);

		float wingLiftZ = (float) (2 * inclinationWings * liftSlopeConstant * sSquared * Math.sin(inclinationWings));

		Vector3f gravityVector = transformVector(toDrone, new Vector3f(0, -getTotalMass() * gravity, 0));
		float gravityZ = (float) gravityVector.z;

		return (-thrust + wingLiftZ + gravityZ);
	}

}
