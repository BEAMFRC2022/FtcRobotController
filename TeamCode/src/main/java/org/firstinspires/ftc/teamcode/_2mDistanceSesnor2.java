package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "_2mDistanceSesnor2 (Blocks to Java)")
public class _2mDistanceSesnor2 extends LinearOpMode {

  private DistanceSensor DistanceFront;
  private DistanceSensor DistanceSide;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    DistanceFront = hardwareMap.get(DistanceSensor.class, "DistanceFront");
    DistanceSide = hardwareMap.get(DistanceSensor.class, "DistanceSide");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        telemetry.addData("front_Distance", DistanceFront.getDistance(DistanceUnit.INCH));
        telemetry.addData("side_Distance", DistanceSide.getDistance(DistanceUnit.INCH));
        // Put loop blocks here.
        telemetry.update();
      }
    }
  }
}
