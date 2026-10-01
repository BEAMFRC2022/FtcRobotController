package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Simple Test Auto", group="Robot")
public class testauto extends LinearOpMode {

    private DcMotor leftFrontDrive  = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive  = null;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize Motors (Ensure these match your Config on the Driver Station)
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "front_left");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "back_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "back_right");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized. Press Play.");
        telemetry.update();


        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            setDrivePower(0.5, 0.5, 0.5, 0.5); // Forward
            sleep(1500);
            setDrivePower(0, 0, 0, 0);         // Stop
            sleep(1000);
        }
    }

    // Helper method to set power to all motors
    public void setDrivePower(double lf, double lb, double rf, double rb) {
        leftFrontDrive.setPower(lf);
        leftBackDrive.setPower(lb);
        rightFrontDrive.setPower(rf);
        rightBackDrive.setPower(rb);
    }
}
