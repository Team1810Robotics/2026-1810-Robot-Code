package frc.robot.auto;

import com.pathplanner.lib.path.PathPlannerPath;

public class Paths {
  public static PathPlannerPath rightShootToPickup;
  public static PathPlannerPath rightPickupToShoot;
  public static PathPlannerPath leftShootToPickup;
  public static PathPlannerPath leftPickupToShoot;

  public static PathPlannerPath leftShootToLowPickup;
  public static PathPlannerPath leftLowPickupToShoot;
  public static PathPlannerPath rightLowPickupToShoot;
  public static PathPlannerPath rightShootToLowPickup;

  public static PathPlannerPath midShootToDepotPickup;
  public static PathPlannerPath depotPickupToShoot;

  public static PathPlannerPath tuning1;
  public static PathPlannerPath tuning2;

  static {
    try {
      rightPickupToShoot = PathPlannerPath.fromPathFile("RightPickupToShoot");
      rightShootToPickup = PathPlannerPath.fromPathFile("RightShootToPickup");
      leftShootToPickup = PathPlannerPath.fromPathFile("LeftShootToPickup");
      leftPickupToShoot = PathPlannerPath.fromPathFile("LeftPickupToShoot");

      leftShootToLowPickup = PathPlannerPath.fromPathFile("LeftShootToLowPickup");
      leftLowPickupToShoot = PathPlannerPath.fromPathFile("LeftLowPickupToShoot");
      rightLowPickupToShoot = PathPlannerPath.fromPathFile("RightLowPickupToShoot");
      rightShootToLowPickup = PathPlannerPath.fromPathFile("RightShootToLowPickup");

      midShootToDepotPickup = PathPlannerPath.fromPathFile("MidShootToPickupDepot");
      depotPickupToShoot = PathPlannerPath.fromPathFile("DepotToShoot");

      tuning1 = PathPlannerPath.fromPathFile("TuningPath1");
      tuning2 = PathPlannerPath.fromPathFile("TuningPath2");
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
