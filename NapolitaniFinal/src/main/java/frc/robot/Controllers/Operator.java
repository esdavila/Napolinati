package frc.robot.Controllers;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.UtilFunctions;

public class Operator {
  // Use PS5Controller instead of XboxController
  private PS5Controller operator;
  private DoubleSubscriber opDeadband;

  public Operator() {
    // Instantiate the PS5 controller on port 2
    operator = new PS5Controller(2);
    opDeadband = UtilFunctions.getSettingSub("/Operator/Deadband", 0.05);
  }

  public double getElevatorRequest() {
    // Left stick Y axis controls elevator (inverted and with deadband)
    return -UtilFunctions.deadband(operator.getLeftY(), opDeadband.get()) * 0.25;
  }

  public double getPivotRequest() {
    // Right stick Y axis controls pivot (inverted and with deadband)
    return -UtilFunctions.deadband(operator.getRightY(), opDeadband.get()) * 0.16;
  }

  public double getIntakeRequest() {
    // Right trigger minus left trigger for intake
    return operator.getR2Axis() - operator.getL2Axis();
  }

  public boolean getSwitchPiece() {
    // Right stick button remains the same for toggling the game piece
    return operator.getR3Button();
  }

  // Map PS5 face buttons:
  // PS5 Cross button (bottom) for L1 command (equivalent to Xbox A)
  public boolean getL1Command() {
    return operator.getCrossButton();
  }

  // PS5 Circle button (right) for L2 command (equivalent to Xbox B)
  public boolean getL2Command() {
    return operator.getCircleButton();
  }

  // PS5 Square button (left) for L3 command (equivalent to Xbox X)
  public boolean getL3Command() {
    return operator.getSquareButton();
  }

  // PS5 Triangle button (top) for L4 command (equivalent to Xbox Y)
  public boolean getL4Command() {
    return operator.getTriangleButton();
  }

  public boolean getLoadingPositionCommand() {
    // Right bumper remains for loading position command
    return operator.getR1Button();
  }

  public boolean getLollipopCommand() {
    // Left bumper remains for the lollipop command
    return operator.getL1Button();
  }

  public Trigger getSwitchPieceTrigger() {
    return new Trigger(this::getSwitchPiece);
  }

  // For fast intake, use the Options button on the PS5 (instead of Xbox Start)
  public Trigger getFastIntake() {
    return new Trigger(operator::getOptionsButton);
  }

  // For a secondary switch trigger, use the Touchpad button on the PS5 (instead of Xbox Back)
  public Trigger getSwitchPieceTrigger2() {
    return new Trigger(operator::getTouchpadButton);
  }
}
