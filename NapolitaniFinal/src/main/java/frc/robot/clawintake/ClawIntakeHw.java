package frc.robot.clawintake;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.math.filter.LinearFilter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import com.revrobotics.spark.SparkLowLevel.MotorType;




public class ClawIntakeHw extends ClawIntake {
  private SparkMax intakeMotor;
  private Rev2mDistanceSensor pieceSensor;
  private double distance;
  private LinearFilter filter;

  public ClawIntakeHw() {
    super();
    intakeMotor = new SparkMax (0, MotorType.kBrushless);
    pieceSensor = new Rev2mDistanceSensor(Port.kOnboard);

    

    // filter out the noisy distance sensor
    filter = LinearFilter.singlePoleIIR(0.06, 0.02);
  }

  @Override
  void setPower(double power) {
    intakeMotor.set(power);
  }

  @Override
  public boolean hasCoral() {
    return 0 <= distance && distance < 40;
  }

  @Override
  public boolean hasAlgae() {
    return false;
  }

  @Override
  void setVelocity(double velocity) {
    intakeMotor.getClosedLoopController().setReference(velocity, ControlType.kVelocity);
  }

  @Override
  void setPosition(double position) {
    intakeMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  @Override
  double getPosition() {
    return intakeMotor.getEncoder().getPosition();
  }

   @Override
  void updateSensors() {
     distance = filter.calculate(pieceSensor.getRange(Unit.kMillimeters));
  }

  @Override
  double getVelocity() {
    return intakeMotor.getEncoder().getVelocity();
  }
}