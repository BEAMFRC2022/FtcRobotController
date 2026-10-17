package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SimplifiedOdometryRobot {
    private LinearOpMode myOpMode;
    public DcMotor intake;
    public PIDController yawController = new PIDController(0, 0, 0);

    public SimplifiedOdometryRobot(LinearOpMode opMode) {
        this.myOpMode = opMode;
    }

    public void initialize(boolean telemetryOn) {
        try {
            intake = myOpMode.hardwareMap.get(DcMotor.class, "intakeMotorAsDcMotor");
        } catch (Exception e) {
            try {
                intake = myOpMode.hardwareMap.get(DcMotor.class, "intake");
            } catch (Exception ex) {
                // optional
            }
        }
    }

    public void readSensors() {}
    public void resetOdometry() {}
    public double getTurnRate() { return 0.0; }
    public double getHeading() { return 0.0; }
    public void moveRobot(double drive, double strafe, double yaw) {}
    public void drive(double distance, double power, double wait) {
        myOpMode.sleep((long)(wait * 1000));
    }
    public void turn(double degrees, double power, double wait) {
        myOpMode.sleep((long)(wait * 1000));
    }

    public static class PIDController {
        public PIDController(double p, double i, double d) {}
        public void reset(double setpoint) {}
        public double getOutput(double current) { return 0.0; }
    }
}
