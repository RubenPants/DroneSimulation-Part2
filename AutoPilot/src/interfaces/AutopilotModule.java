package interfaces;

/**
 * A class of autopilot modules.
 */
public interface AutopilotModule {

	/**
	 * Define the parameters of airports
	 * 
	 * @param length
	 *            The length of airports.
	 * @param width
	 *            The width of airports.
	 */
	void defineAirportParams(float length, float width);

	/**
	 * Define an airport. (centerToRunway0X, centerToRunway0Z) constitutes a unit
	 * vector pointing from the center of the airport towards runway 0.
	 * 
	 * @param centerX
	 *            The x coordinate of the center of this airport.
	 * @param centerZ
	 *            The z coordinate of the center of this airport.
	 */
	void defineAirport(float centerX, float centerZ, float centerToRunway0X, float centerToRunway0Z);

	/**
	 * Define a new drone at given airport, gate, original orientation and
	 * configuration.
	 * 
	 * @param airport
	 *            The airport at which the drone is located.
	 * @param gate
	 *            The gate in which the drone is.
	 * @param pointingToRunway
	 *            The index of the gate towards which the drone is orientated (= 0
	 *            or 1).
	 * @param config
	 *            The configuration of the drone/
	 */
	void defineDrone(int airport, int gate, int pointingToRunway, AutopilotConfig config);

	/**
	 * Notify the given drone of its current situation.
	 * 
	 * @param drone
	 *            The index of the drone (0-N if N drones have been defined).
	 * @param inputs
	 *            The new inputs for the drone.
	 */
	void startTimeHasPassed(int drone, AutopilotInputs inputs);

	/**
	 * Notify the given drone that the complete time has passed.
	 * 
	 * @param drone
	 *            The index of the drone.
	 * @return The autopilot's output.
	 */
	AutopilotOutputs completeTimeHasPassed(int drone);

	/**
	 * Deliver the package from the given airport and gate to the given airport and
	 * gate.
	 * 
	 * @param fromAirport
	 *            The starting airport.
	 * @param fromGate
	 *            The starting gate.
	 * @param toAirport
	 *            The destination airport.
	 * @param toGate
	 *            The destination gate.
	 */
	void deliverPackage(int fromAirport, int fromGate, int toAirport, int toGate);

	/**
	 * End the simulation.
	 */
	void simulationEnded();

}