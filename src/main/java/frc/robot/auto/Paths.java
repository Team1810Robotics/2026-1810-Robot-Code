package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerPath;

public class Paths {
  public static PathPlannerPath rightShootToPickup;
  public static PathPlannerPath rightPickupToShoot;
  public static PathPlannerPath leftShootToPickup;
  public static PathPlannerPath leftPickupToShoot;

  static {
    try {
      rightPickupToShoot = PathPlannerPath.fromPathFile("RightPickupToShoot");
      rightShootToPickup = PathPlannerPath.fromPathFile("RightShootToPickup");
      leftShootToPickup = PathPlannerPath.fromPathFile("LeftShootToPickup");
      leftPickupToShoot = PathPlannerPath.fromPathFile("LeftPickupToShoot");
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
