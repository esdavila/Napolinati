package frc.robot.Elevator;


// Import static unit conversion methods
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

// Import NetworkTables for telemetry and settings
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// Import SmartDashboard for command buttons and data display
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Import WPILib Command framework base classes
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Import scoring position enum and utility functions for settings
import frc.robot.piecetypeswitcher.ScoringPositions;
import java.util.HashMap;
import java.util.function.DoubleSupplier;
// Import auto-logging utilities (for telemetry and debugging)


// Import helper for motor position resets and miscellaneous utilities
import org.MotorPositionReset;
import org.UtilFunctions;

// Abstract Elevator class providing a template for hardware/simulation implementations
public abstract class Elevator extends SubsystemBase {
  // Constant offset (e.g., mechanical or measurement offset)
  final double OFFSET = 11.4;
  // Gravity compensation voltage
  static final double kG = 0.5;

  // Abstract methods to be implemented by concrete elevator classes:
  // Commands for setting target position and output power.
  public abstract void setPosition(double distance);
  public abstract void setPower(double pct);

  // Methods to retrieve sensor feedback; annotated for auto-logging.

  public abstract double getMotorPosition();

  public abstract double getDistanceSensor();

  // Methods to reset encoder and directly control voltage output.
  public abstract void setEncoderPosition(double position);
  public abstract void setVoltage(double volts);

  // Method to update sensor values (called periodically)
  abstract void updateSensor();

  // Flag indicating whether PID control is active.
  boolean pidEnabled;
  // NetworkTables entry for publishing the current elevator height.
  DoubleEntry heightPub;
  // Subscriber to a NetworkTables boolean setting to enable/disable PID.
  BooleanSubscriber pidEnableNtSub;
  // Flag for collision warnings (e.g., if motion is blocked)
  boolean collisionWarning;
  // Helper object to manage automatic encoder resets when near the elevator bottom.
  MotorPositionReset positionReset;
  
  // Map of preset scoring positions keyed by an enum; each entry supplies a target height.
  private HashMap<ScoringPositions, DoubleSupplier> positions;

  // Constructor initializes telemetry, preset positions, and configuration parameters.
  public Elevator() {
    super();
    // Register this subsystem for auto-logging telemetry.

    // Default to PID disabled.
    pidEnabled = false;
    
    // Add dashboard commands to quickly set elevator to preset positions.
    SmartDashboard.putData("Set Elevator 30", setPositionCmd(30));
    SmartDashboard.putData("Set Elevator 70", setPositionCmd(70));
    
    // Publish elevator height data to NetworkTables for simulation or telemetry.
    heightPub = NetworkTableInstance.getDefault()
                  .getDoubleTopic("/Simulation/Claw Height").getEntry(0);
    
    // Initialize the positions map with preset heights using utility functions.
    positions = new HashMap<>();
    // These preset values (in inches) are arbitrary and can be tuned per mechanism.
    positions.put(ScoringPositions.L1Coral, UtilFunctions.getSettingSub("ElevatorPos/L1Coral", 17.8));
    positions.put(ScoringPositions.L2Coral, UtilFunctions.getSettingSub("ElevatorPos/L2Coral", 24.7));
    positions.put(ScoringPositions.L3Coral, UtilFunctions.getSettingSub("ElevatorPos/L3Coral", 40));
    positions.put(ScoringPositions.L4Coral, UtilFunctions.getSettingSub("ElevatorPos/L4Coral", 69));
    positions.put(ScoringPositions.LoadingPosition, UtilFunctions.getSettingSub("ElevatorPos/LoadingPosition", 16.5));
    positions.put(ScoringPositions.L2Algae, UtilFunctions.getSettingSub("ElevatorPos/L2Algae", 36));
    positions.put(ScoringPositions.L3Algae, UtilFunctions.getSettingSub("ElevatorPos/L3Algae", 52.4));
    positions.put(ScoringPositions.ProcessorAlgae, UtilFunctions.getSettingSub("ElevatorPos/ProcessorAlgae", 16.5));
    positions.put(ScoringPositions.NetAlgae, UtilFunctions.getSettingSub("ElevatorPos/NetAlgae", 80));
    positions.put(ScoringPositions.Lollipop, UtilFunctions.getSettingSub("ElevatorPos/Lollipop", 22.7));
    
    // Subscribe to a NetworkTables setting to enable or disable PID control.
    pidEnableNtSub = UtilFunctions.getSettingSub("Elevator/EnablePid", false);
    // Initialize collision warning flag.
    collisionWarning = false;
    // Instantiate a helper to determine when to reset the encoder based on position thresholds.
    positionReset = new MotorPositionReset(1, 0.5, 0.5);
  }

  // The periodic method runs every cycle; here it updates sensors, publishes telemetry, and checks encoder reset conditions.
  @Override
  public void periodic() {
    updateSensor(); // Refresh sensor readings
    heightPub.set(getPosition()); // Publish the current elevator position
    pidEnabled = pidEnableNtSub.get(); // Update the PID enable flag from NetworkTables

    // Calculate the effective distance including the offset, converting sensor units if needed.
    double distance = OFFSET + Meter.of(getDistanceSensor()).in(Inches);

    // If the sensor indicates the elevator is near the bottom (between 15" and 30"), consider a reset.
    if ((15 < distance) && (distance < 30)) {
      if (positionReset.updateReset(getMotorPosition(), distance)) {
        // Uncomment the next line if you wish to reset the encoder when conditions are met.
        // setEncoderPosition(distance);
      }
    } else {
      // If the elevator is away from the bottom, reset the position reset timer.
      positionReset.resetTimer();
    }
  }

  // Returns a command to drive the elevator using provided percent output and claw angle inputs.
  public Command driveElevator(DoubleSupplier pct, DoubleSupplier clawAngle) {
    return new DriveElevator(this, clawAngle, pct);
  }

  // Returns a command that sets the elevator to a specific numerical position.
  public Command setPositionCmd(double position) {
    return run(() -> setPosition(position));
  }

  // Returns a command that sets the elevator to a preset position based on scoring configuration.
  public Command setPositionCmd(ScoringPositions position) {
    return run(() -> setPosition(getSetPosition(position)))
        .until(() -> Math.abs(getSetPosition(position) - getPosition()) < 1);
  }

  // Helper method to fetch the preset position for a given scoring target.
  public double getSetPosition(ScoringPositions position) {
    return positions.get(position).getAsDouble();
  }

  // Returns the current elevator position; can combine encoder and sensor data if needed.

  public double getPosition() {
    return getMotorPosition();
    // Alternatively, combine the offset and sensor readings:
    // return OFFSET + Meter.of(getDistanceSensor()).in(Inches) + getMotorPosition();
  }

  // Returns a command that holds the elevator at its current position.
  public Command holdElevator() {
    return new HoldElevator(this);
  }

  // Returns a command to reset the elevator's encoder (e.g., during homing).
  public Command resetElevator() {
    return runOnce(() -> setEncoderPosition(16.5));
  }

  // Sets the collision warning flag, which can be used by commands to signal issues.
  public void setCollisionWarning(boolean warningOn) {
    collisionWarning = warningOn;
  }

  // Returns the current collision warning state.
  public boolean getCollisionWarning() {
    return collisionWarning;
  }

  // Returns a command that performs a manual homing of the elevator by resetting the encoder.
  public Command manualHome() {
    return runOnce(
        () -> {
          double distance = OFFSET + Meter.of(getDistanceSensor()).in(Inches);
          setEncoderPosition(distance);
        });
  }

  // Abstract method to return the motor’s current velocity; concrete implementations must define this.
  protected abstract double getVelocity();
}
