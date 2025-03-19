package frc.robot.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.math.filter.LinearFilter; // For smoothing sensor data

// ElevatorHwRev extends the abstract Elevator class with hardware-specific implementations using REV parts
public class ElevatorHw extends Elevator {
    // Declare Spark Max motor controllers for left (master) and right (follower) motors
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    
    // Retrieve the integrated encoder from the left motor (through-bore encoder on the NEO)
    private RelativeEncoder leftEncoder;
    
    // Declare a REV 2m distance sensor instance
    private Rev2mDistanceSensor distanceSensor;
    
    // Create a filter instance to smooth out noisy sensor readings
    private LinearFilter filter;
    
    // Variable to hold the filtered sensor value; initialize as needed
    double sensorValue = 0.0;
    
    // Constructor initializes hardware components and applies configurations
    public ElevatorHw() {
      super(); // Call parent constructor from the abstract Elevator class
      
      // Instantiate the left motor controller on CAN ID 1 using a brushless (NEO) motor
      leftMotor = new SparkMax(1, MotorType.kBrushless);
      // Instantiate the right motor controller on CAN ID 2 using a brushless (NEO) motor
      rightMotor = new SparkMax(2, MotorType.kBrushless);

      SparkMaxConfig config = new SparkMaxConfig();
      config.idleMode(IdleMode.kBrake);
      config.smartCurrentLimit(40);
      config.follow(leftMotor);
      rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      
      // Retrieve the integrated encoder from the left Spark Max controller
      leftEncoder = leftMotor.getEncoder();
      
      // Instantiate the REV 2m distance sensor (adjust the port/parameters as per your sensor)
      distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
      
      // Initialize a single-pole IIR filter to smooth sensor data (tune time constant as needed)
      filter = LinearFilter.singlePoleIIR(0.16, 0.02);
    }
    
    // Command the elevator to move to a target position using built-in PID control if enabled
    @Override
    public void setPosition(double distance) {
      if (pidEnabled) {
        // Use the Spark Max built-in PID controller to set a position reference
         leftMotor.getClosedLoopController().setReference(distance, ControlType.kPosition);
      } else {
        // If PID control is not enabled, stop the motor (without braking)
        leftMotor.stopMotor();
      }
    }
    
    // Directly set the motor's output power (as a percent value between -1 and 1)
    @Override
    public void setPower(double pct) {
      leftMotor.set(pct);
    }
    
    // Return the current position from the integrated encoder (units depend on configuration)
    @Override
    public double getMotorPosition() {
      return leftEncoder.getPosition();
    }
    
    // Return the processed (filtered) sensor reading from the distance sensor
    @Override
    public double getDistanceSensor() {
      return sensorValue;
    }
    
    // Reset or set the integrated encoder's position to a given value
    @Override
    public void setEncoderPosition(double position) {
      leftEncoder.setPosition(position);
    }
    
    // Directly command a specific output voltage to the left motor
    @Override
    public void setVoltage(double voltage) {
      leftMotor.setVoltage(voltage);
    }
    
    // Update the sensor reading by acquiring data from the REV distance sensor and filtering it
    @Override
    void updateSensor() {
      // Get a measurement from the distance sensor; assume it returns a Double in meters
      Double measurement = distanceSensor.getRange();
      if (measurement != null) {
        // Pass the raw measurement through the IIR filter for a stable value
        sensorValue = filter.calculate(measurement);
      }
    }
    
    // Retrieve the current velocity from the integrated encoder (units depend on configuration)
    @Override
    protected double getVelocity() {
      return leftEncoder.getVelocity();
    }
  }