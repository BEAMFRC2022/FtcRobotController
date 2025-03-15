package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "_2mDistanceSesnor (Blocks to Java)")
public class _2mDistanceSesnor extends LinearOpMode {

  private DistanceSensor leftDistSensor;
  private DistanceSensor rightDistSensor;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    leftDistSensor = hardwareMap.get(DistanceSensor.class, "leftDistSensor");
    rightDistSensor = hardwareMap.get(DistanceSensor.class, "rightDistSensor");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        telemetry.addData("Left_Distance", leftDistSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("right_Distance", rightDistSensor.getDistance(DistanceUnit.CM));
        // Put loop blocks here.
        telemetry.update();
      }
    }
  }
}
