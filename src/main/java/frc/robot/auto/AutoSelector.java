package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.commands.left.LeftDoublePickup;
import frc.robot.auto.commands.left.LeftSinglePickup;
import frc.robot.auto.commands.right.RightDoublePickup;
import java.util.HashMap;
import java.util.Map;

public class AutoSelector {

  private final SendableChooser<StartPosition> positionChooser = new SendableChooser<>();
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final Map<StartPosition, Map<String, Command>> autos = new HashMap<>();

  private StartPosition lastPosition = null;

  public AutoSelector() {

    positionChooser.setDefaultOption("Center", StartPosition.CENTER);
    positionChooser.addOption("Left", StartPosition.LEFT);
    positionChooser.addOption("Right", StartPosition.RIGHT);

    this.addAuto(StartPosition.LEFT, "Single Pickup", new LeftSinglePickup());
    this.addAuto(StartPosition.LEFT, "Double Pickup", new LeftDoublePickup());

    this.addAuto(StartPosition.RIGHT, "Single Pickup", new RightDoublePickup());
    this.addAuto(StartPosition.RIGHT, "Double Pickup", new RightDoublePickup());

    SmartDashboard.putData("Start Position", positionChooser);
    SmartDashboard.putData("Auto", autoChooser);
  }

  public void addAuto(StartPosition pos, String name, Command auto) {
    autos.computeIfAbsent(pos, k -> new HashMap<>()).put(name, auto);
  }

  public void updateChooser() {

    StartPosition selected = positionChooser.getSelected();

    if (selected == lastPosition) {
      return;
    }

    lastPosition = selected;

    autoChooser = new SendableChooser<>();

    if (autos.containsKey(selected)) {
      autos
          .get(selected)
          .forEach(
              (name, command) -> {
                autoChooser.addOption(name, command);
              });
    }

    SmartDashboard.putData("Auto", autoChooser);
  }

  public Command getSelectedAuto() {
    return autoChooser.getSelected();
  }

  private enum StartPosition {
    LEFT,
    CENTER,
    RIGHT
  }
}
