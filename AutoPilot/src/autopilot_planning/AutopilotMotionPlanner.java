package autopilot_planning;

import autopilot_planning.DroneTrajectory.DroneTrajectoryType;
import autopilot_utilities.Path3D;
import autopilot_utilities.Vector3f;
import interfaces.AutopilotConfig;
import interfaces.AutopilotInputs;

/**
 * A class of motion planners for drone autopilots, for determining what direction
 *  the drone should fly towards.
 */
public class AutopilotMotionPlanner {

	/**
	 * Variable denoting whether or not this object logs information.
	 */
	public boolean ENABLE_LOGGING = false;
	
	/**
	 * Registers the trajectory for the drone.
	 */
	private DroneTrajectory trajectory;

	/**
	 * Registers the minimum turning radius for the drone.
	 */
	private final static double TURNING_RADIUS = 400.0;

	/**
	 * Initialize this new motion planner.
	 */
	public AutopilotMotionPlanner() {
		loadTrajectory();
	}

	/**
	 * Load trajectory.
	 */
	public void loadTrajectory() {
		trajectory = DroneTrajectory.initializeAlgorithm(DroneTrajectoryType.DUBINS);
	}

	/**
	 * Determine the preferred future motion of the autopilot's drone for the given configuration,
	 *  input and list of detected cubes. 
	 * 
	 * @param 	configuration
	 * 			The current configuration of the autopilot.
	 * @param 	inputs
	 * 			The latest inputs the autopilot received.
	 * @param 	path
	 * 			A 3-dimensional path with locations that are to be visited by the drone.
	 * @return 	An array of 2 floats. 
	 * 				The first float represents the desired pitch. (in radians)
	 * 				The second float represents the desired heading. (in radians)
	 */
	public float[] preferredMotion(AutopilotConfig configuration, AutopilotInputs inputs, Path3D path, boolean landingTrailSet, float deltaTimeElapsed) {
		Vector3f position = new Vector3f(inputs.getX(), inputs.getY(), inputs.getZ());
		Vector3f rotations = new Vector3f(inputs.getHeading(), inputs.getPitch(), inputs.getRoll());
		return trajectory.calculateTrajectory(path, position, rotations, 0.0f, TURNING_RADIUS, landingTrailSet, deltaTimeElapsed);
	}

}