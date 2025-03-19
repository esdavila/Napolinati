package frc.robot.piecetypeswitcher;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PieceTypeSwitcher extends SubsystemBase {

 private String pieceSelected;

  public PieceTypeSwitcher() {
    super();
    pieceSelected = "Coral";
  }

  public String getPieceSelected() {
    return pieceSelected;
  }

  public void togglePieceSelected() {
    if (pieceSelected == "Coral") {
      pieceSelected = "Algae";
    } else {
      pieceSelected = "Coral";
    }
  }

  public Command switchPieceSelected() {
    return runOnce(this::togglePieceSelected);
  }

  public boolean isCoral() {
    return pieceSelected == "Coral";
  }

  public boolean isAlgae() {
    return pieceSelected == "Algae";
  }

  public Color getPieceColor() {
    if (isCoral()) {
      return Color.kYellow;
    }
    return Color.kAqua;
  }
}
