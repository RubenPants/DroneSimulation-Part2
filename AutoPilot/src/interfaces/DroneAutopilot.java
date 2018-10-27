package interfaces;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import autopilot_gui.AutopilotGUI;
import autopilot_physics.AutopilotPhysics;
import autopilot_physics.AutopilotPhysics.AutopilotPhysicsStage;
import autopilot_planning.AutopilotMotionPlanner;
import autopilot_utilities.Path3D;
import autopilot_utilities.Point3D;
import autopilot_utilities.Vector3f;
import autopilot_vision.AutopilotImageAnalyser;

/**
 * A class of autopilots for steering a drone. Autopilots translate a drone's
 * configuration and constantly updated situation to output commands, such that
 * that very drone can reach its desired destination.
 */
public class DroneAutopilot implements Autopilot, ActionListener {

	private boolean ENABLE_LOGGING = false;
	private boolean controlWithPhone = false;
	private boolean flying = false;

	/**
	 * Initialise this autopilot with given image analyser.
	 * 
	 * @param analyser
	 *            The image analyser for this new autopilot.
	 * @post This autopilot's image analyser equals the given one. |
	 *       new.getAnalyser() == analyser
	 * @throws IllegalArgumentException
	 *             The given image analyser is invalid. | analyser == null
	 */
	public DroneAutopilot(AutopilotImageAnalyser analyser) throws IllegalArgumentException {
		if (analyser == null)
			throw new IllegalArgumentException("Invalid image anlyser");
		this.analyser = new AutopilotImageAnalyser(); // No history
		this.planner = new AutopilotMotionPlanner();
		AutopilotGUI gui = new AutopilotGUI();
		gui.addActionListener(this);
	}

	@Override
	public void actionPerformed(ActionEvent arg0) {
		if (controlWithPhone)
			controlWithPhone = false;
		else
			controlWithPhone = true;
	}

	/**
	 * Initialise this autopilot.
	 * 
	 * @effect This autopilot is initialised with a new image analyser.
	 */
	public DroneAutopilot() {
		this(new AutopilotImageAnalyser());
	}

	/**
	 * Variable referencing an image analyser.
	 */
	private AutopilotImageAnalyser analyser;

	/**
	 * Variable referencing a motion planner.
	 */
	private AutopilotMotionPlanner planner;

	/**
	 * This autopilot's physics engine.
	 */
	private AutopilotPhysics physics;

	/**
	 * Variable referencing a gui for this autopilot.
	 */
	@SuppressWarnings("unused")
	private AutopilotGUI gui;

	/**
	 * Set this autopilot's configuration to the given one.
	 * 
	 * @param configuration
	 *            The new configuration for this autopilot.
	 */
	public void setConfiguration(AutopilotConfig configuration) {
		if (configuration == null)
			return;
		this.configuration = configuration;
	}

	/**
	 * Returns this autopilot's current configuration.
	 */
	public AutopilotConfig getConfiguration() {
		return this.configuration;
	}

	/**
	 * This autopilot's current configuration.
	 */
	private AutopilotConfig configuration;

	/**
	 * Returns whether or not this autopilot uses motion planning.
	 */
	public boolean getUsesPlanner() {
		return usesPlanner;
	}

	/**
	 * Set whether or not this autopilot uses motion planning.
	 * 
	 * @param usesPlanner
	 *            True if and only if the planner is to be used.
	 */
	public void setUsesPlanner(boolean usePlanner) {
		this.usesPlanner = usePlanner;
	}

	/**
	 * Variable registering whether or not motion planning is applied.
	 */
	private boolean usesPlanner = false;

	// Time variables
	private float previousTimeElapsed = -0.02f;
	private float timeElapsed = 0;
	private float deltaTimeElapsed = 0.02f;

	/**
	 * Let this autopilot's image analyser analyse the given input.
	 * 
	 * @param inputs
	 *            The autopilot input that is to be analysed.
	 * @return The results of the analysis by the image analyser and the physical
	 *         model.
	 */
	private AutopilotOutputs analyseInputs(AutopilotInputs inputs, boolean firstFrame) {

		timeElapsed = inputs.getElapsedTime();
		deltaTimeElapsed = (timeElapsed - previousTimeElapsed);
		previousTimeElapsed = timeElapsed;
		if (deltaTimeElapsed == 0.0) {
			deltaTimeElapsed = 0.017f;
		}

		if (firstFrame)
			physics.getInputsStart(inputs);
		else
			physics.getInputs(inputs);

		if (controlWithPhone) {
			return physics.output(1000, 1000, null, AutopilotPhysicsStage.PHONE);
		}
		if (ENABLE_LOGGING)
			System.out.println("PATH : " + (path == null ? "null" : path.toString()));

		Point3D droneCoordinate = new Point3D(inputs.getX(), inputs.getY(), inputs.getZ());
		if (landingTrailSet && this.path != null) {
			ArrayList<Point3D> coordinates = (ArrayList<Point3D>) this.path.getCoordinates();
			if (ENABLE_LOGGING)
				System.out.println("check landing");
			if (coordinates.size() > 0) {
				Point3D coordinate = coordinates.get(0);
				if (coordinate.distanceTo(droneCoordinate) < 100.0 && inputs.getY() < 100
						&& (Math.abs(inputs.getHeading()) > Math.PI * 7.0 / 8.0
								|| droneCoordinate.distanceTo(new Point3D(0.0, 0.0, 0.0)) > 1800)) {
					this.path.setVisited(0, true);
				} else if ((coordinate.distanceTo(droneCoordinate) < 50 && inputs.getY() < 100)) {
					coordinates.clear();
					coordinates.add(0, new Point3D(0.0, 25.0, -1000.0));
					coordinates.add(0, new Point3D(0.0, 60.0, -1500.0));
					if (inputs.getHeading() < 0)
						coordinates.add(0, new Point3D(800.0, 60.0, -2000.0));
					else
						coordinates.add(0, new Point3D(-800.0, 60.0, -2000.0));

					this.path.setCoordinates(coordinates);
				}
			}
		}

		// Control only starts when a path is given to the autopilot
		if (this.path == null)
			return physics.output(0.0f, 0.0f, null, AutopilotPhysicsStage.LAND);
		else if (this.path.isEmpty()) {

			Vector3f requestedVector = new Vector3f(0 - inputs.getX(), 0 - inputs.getY(), 0 - inputs.getZ());
			float[] pitchAndHeading = new float[2];
			pitchAndHeading[1] = getPidPositionOneCube(inputs, new Point3D(0, 0, 0));
			float distanceToCenter = (float) requestedVector.length();

			// Fly to circle around start (config landing)
			if (distanceToCenter > 800 && distanceToCenter < 850 && inputs.getY() < 30 && physics.getSSquared() < 3000f)
				readyToLand = true;
			else if (distanceToCenter > 850)
				readyToLand = false;

			if (!landingTrailSet) {
				readyToLand = false;
				Point3D firstPoint = new Point3D(0.0, 60.0, -1500.0);
				// if (droneCoordinate.distanceTo(firstPoint) < 600 && inputs.getY() < 100)
				// return physics.output(0.0f, inputs.getHeading(), null,
				// AutopilotPhysicsStage.FLY);
				ArrayList<Point3D> trail = new ArrayList<Point3D>();
				trail.add(firstPoint);
				trail.add(new Point3D(0.0, 25.0, -1000.0));
				this.path.setCoordinates(trail);
				planner.loadTrajectory();
				landingTrailSet = true;
			}

			if (!readyToLand || distanceToCenter > 800) {
				if (inputs.getY() > 30)
					pitchAndHeading[0] = (float) -Math.PI / 27; // -6.66 degrees
				else if (inputs.getY() > 26)
					pitchAndHeading[0] = (float) -Math.PI / 36; // -5 degrees
				else if (inputs.getY() > 24)
					pitchAndHeading[0] = (float) -Math.PI / 180; // -1 degrees
				else if (inputs.getY() < 21)
					pitchAndHeading[0] = (float) Math.PI / 36; // 5 degrees
				else if (inputs.getY() < 23)
					pitchAndHeading[0] = (float) Math.PI / 180; // 1 degrees
				else
					pitchAndHeading[0] = 0;

				if (!readyToLand && distanceToCenter < 800)
					pitchAndHeading[1] = inputs.getHeading();

				return physics.output(pitchAndHeading[0], pitchAndHeading[1], null, AutopilotPhysicsStage.FLY);
			} else if (readyToLand) { // Land
				if (inputs.getY() > 5 || physics.getSSquared() > 300)
					return physics.output(0.0f, pitchAndHeading[1], null, AutopilotPhysicsStage.LAND);
				else {
					return physics.output(0.0f, pitchAndHeading[1], new Vector3f(0, 0, 0), AutopilotPhysicsStage.TAXI);
				}
			}

		} else if (inputs.getY() <= 30f) {
			if (inputs.getY() <= 5f)
				flying = false;
			if (!flying)
				return physics.output(0.0f, 0.0f, null, AutopilotPhysicsStage.TAKE_OFF);
		}

		// Perform sequential calculation ; vision - motion - physics
		// analyser.locateAllCubes(configuration, inputs, path);
		for (int i = 0; i < this.path.getCoordinates().size(); i++) { // Delete cubes when close-by
			Point3D coordinate = this.path.getCoordinates().get(i);
			if (coordinate.distanceTo(droneCoordinate) < 3.5) {
				if (!landingTrailSet) {
					this.path.setVisited(i, true);
					// TODO
					// new PathReorden().reordenPath(path, droneCoordinate, inputs);
				}
			}
		}

		flying = true;
		float[] preferredMotion = planner.preferredMotion(configuration, inputs, path, landingTrailSet,
				deltaTimeElapsed);
		if (ENABLE_LOGGING)
			System.out.println("preferredmotion0 " + preferredMotion[0] + " preferredmotion1 " + preferredMotion[1]);
		return physics.output(preferredMotion[0], preferredMotion[1], null, AutopilotPhysicsStage.FLY);

	}

	/**
	 * Registers whether or not this drone is ready to land.
	 */
	private boolean readyToLand = false;

	/**
	 * Registers whether or not the landing trail (set of imaginary locations to
	 * follow while landing) has been set already.
	 */
	private boolean landingTrailSet = false;

	/**
	 * Notify the autopilot that the simulation started.
	 * 
	 * @param config
	 *            The configuration of the drone this autopilot represents.
	 * @param inputs
	 *            The current situation of the drone this autopilot steers.
	 * @return Output commands for the drone, based on the given input parameters.
	 * @category API
	 */
	public AutopilotOutputs simulationStarted(AutopilotConfig config, AutopilotInputs inputs) {
		analyser.clearHistory();
		setConfiguration(config);
		if (physics == null) {
			physics = new AutopilotPhysics(configuration.getGravity(), configuration.getWingX(),
					configuration.getTailSize(), configuration.getEngineMass(), configuration.getWingMass(),
					configuration.getTailMass(), configuration.getMaxThrust(), configuration.getMaxAOA(),
					configuration.getWingLiftSlope(), configuration.getHorStabLiftSlope(),
					configuration.getVerStabLiftSlope(), configuration.getRMax());
		}
		AutopilotImageAnalyser.ENABLE_LOGGING = false;
		this.planner.ENABLE_LOGGING = false;
		this.physics.ENABLE_LOGGING = false;
		return analyseInputs(inputs, true);
	}

	/**
	 * Notify this autopilot of the new situation for the drone it's steering.
	 * 
	 * @param inputs
	 *            The updated situation for the drone.
	 * @return Output commands for steering the drone, based on its most recent
	 *         situation.
	 * @category API
	 */
	public AutopilotOutputs timePassed(AutopilotInputs inputs) {
		return analyseInputs(inputs, false);
	}

	/**
	 * End the simulation.
	 * 
	 * @category API
	 */
	public void simulationEnded() {
		analyser.clearHistory(); // Currently done both on ending and starting
	}

	/**
	 * The input path for this autopilot.
	 */
	private Path3D path;

	/**
	 * Set the path for this autopilot.
	 * 
	 * @param path
	 *            The path for this autopilot.
	 * @category API
	 */
	public void setPath(Path path) {
		try {
			landingTrailSet = false;
			this.path = new Path3D(path);
			this.path.flattenPath();
			new PathReorden2().reordenPath(this.path, new Point3D(0.0, 50, -500), 0, (float) Math.PI / 36);
		} catch (Exception exception) {
			System.out.println("Input path was either empty or invalid.");
		}
	}

	// PID using position
	private float positionError = 0;
	private float prevPositionError = 0;
	private Point3D dronePosStart;
	private boolean printPID = false;

	/*
	 * The PID method using position when there is only one cube left
	 * 
	 * @return Return the requested heading
	 */
	private float getPidPositionOneCube(AutopilotInputs inputs, Point3D target) {
		if (dronePosStart == null || Math.abs(positionError) > 10)
			dronePosStart = new Point3D(inputs.getX(), inputs.getY(), inputs.getZ());

		updatePositionErrorOneCube(inputs, target);

		if (printPID && ENABLE_LOGGING) {
			System.out.println("PID - POSITION ONE CUBE");
			System.out.println("---proportional: " + proportionalPosition());
			System.out.println("---derivative: " + 100 * derivativePosition());
			System.out.println("---integral: " + 0);// 0.5 * integralPosition());
		}

		// Return the drone its own heading with the requested direction
		float pidPositionFault = (float) (proportionalPosition() + 100 * derivativePosition());// + 0.5 *
																								// integralPosition());
		if (pidPositionFault > 20)
			pidPositionFault = 20;
		else if (pidPositionFault < -20)
			pidPositionFault = -20;

		if (printPID && ENABLE_LOGGING)
			System.out.println("---default pidPositionFault: " + pidPositionFault);

		return (float) (inputs.getHeading() + pidPositionFault * (Math.PI / 180));
	}

	private float proportionalPosition() {
		return positionError;
	}

	private float derivativePosition() {
		return (float) Math.atan(positionError - prevPositionError);
	}

	private void updatePositionErrorOneCube(AutopilotInputs inputs, Point3D target) {
		prevPositionError = positionError;
		positionError = (float) (getDistanceFaultOneCube(inputs, target));

		if (printPID && ENABLE_LOGGING) {
			System.out.println("---positionError " + positionError);
			System.out.println("---prevPositionError " + prevPositionError);
		}
	}

	private float getDistanceFaultOneCube(AutopilotInputs inputs, Point3D target) {
		float numerator = (float) Math.abs((target.getZ() - dronePosStart.getZ()) * inputs.getX()
				- (target.getX() - dronePosStart.getX()) * inputs.getZ() + target.getX() * dronePosStart.getZ()
				- target.getZ() * dronePosStart.getX());
		float difference = (float) (numerator / distanceTo(dronePosStart, target));

		float sign = (float) ((float) (inputs.getX() - dronePosStart.getX()) * (target.getZ() - dronePosStart.getZ())
				- (inputs.getZ() - dronePosStart.getZ()) * (target.getX() - dronePosStart.getX()));
		if (sign > 0)
			return -difference; // Drone on the right of the line
		else
			return difference; // Drone on the left of the line
	}

	private float distanceTo(Point3D point, Point3D position) {
		return (float) Math
				.sqrt(Math.pow(point.getX() - position.getX(), 2) + Math.pow(point.getZ() - position.getZ(), 2));
	}

}
