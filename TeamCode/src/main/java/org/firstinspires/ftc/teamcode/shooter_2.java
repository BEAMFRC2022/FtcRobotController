package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "shooter (Blocks to Java)")
public class shooter_2 extends LinearOpMode {

  private CRServo shooterright;
  private CRServo shooterleft;
  private double shooterPower = 0.5;
  private boolean wasDpadUp = false;
  private boolean wasDpadDown = false;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    shooterright = hardwareMap.get(CRServo.class, "shooterright");
    shooterleft = hardwareMap.get(CRServo.class, "shooterleft");

    // Put initialization blocks here.
    shooterright.setDirection(CRServo.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        if (gamepad2.dpad_up && !wasDpadUp) {
          shooterPower = Math.min(1.0, shooterPower + 0.1);
        } else if (gamepad2.dpad_down && !wasDpadDown) {
          shooterPower = Math.max(0.0, shooterPower - 0.1);
        }
        wasDpadUp = gamepad2.dpad_up;
        wasDpadDown = gamepad2.dpad_down;

        if (gamepad2.y) {
          shooterleft.setPower(shooterPower);
          shooterright.setPower(shooterPower);
        } else {
          shooterleft.setPower(0);
          shooterright.setPower(0);
        }

        telemetry.addData("Shooter Power", "%.2f", shooterPower);
        telemetry.update();
      }
    }
  }
}
