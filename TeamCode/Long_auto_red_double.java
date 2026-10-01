package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous(name = "LONG_AUTO_RedTeam_DOUBLE (Blocks to Java)")
public class LONG_AUTO_RedTeam_DOUBLE extends LinearOpMode {

    private DcMotor intakeMotor;
    private DcMotor backleftMotor;
    private DcMotor frontleftmotor;
    private DcMotor backrightmotor;
    private DcMotor frontrightmotor;
    private DcMotor flywheel1;
    private IMU imu;
    private DcMotor Loader;

    ElapsedTime runtime;
    double COUNTS_PER_INCH;

    /**
     * This OpMode illustrates the basics of AprilTag recognition and pose estimation, using the easy way.
     * All settings have default values. To customize settings with the Builder pattern, see the sample
     * OpMode called ConceptAprilTag. For an introduction to AprilTags, see the FTC-DOCS link below:
     * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html In this
     * sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
     * "TagLibrary" will have their position and orientation information displayed. This default TagLibrary
     * contains the current Season's AprilTags and a small set of "test Tags" in the high number range. When an
     * AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative
     * to the camera. This information is provided in the "ftcPose" member of the returned "detection", and is
     * explained in the ftc-docs page linked below. https://ftc-docs.firstinspires.org/apriltag-detection-values
     */
    @Override
    public void runOpMode() {
        boolean USE_WEBCAM;
        double FORWARD_SPEED;
        double TURN_SPEED;
        int COUNTS_PER_MOTOR_REV;
        double DRIVE_GEAR_REDUCTION;
        int WHEEL_DIAMETER_INCHES;
        double travelspeed;
        int First_Distance;
        int Second_Distance;
        int redvsblue;
        int flywheelspeedshooting;
        ImuOrientationOnRobot orientationOnRobot;
        int travel_left;
        int travel_right;

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotorAsDcMotor");
        backleftMotor = hardwareMap.get(DcMotor.class, "backleftMotor");
        frontleftmotor = hardwareMap.get(DcMotor.class, "frontleftmotor");
        backrightmotor = hardwareMap.get(DcMotor.class, "backrightmotor");
        frontrightmotor = hardwareMap.get(DcMotor.class, "frontrightmotor");
        flywheel1 = hardwareMap.get(DcMotor.class, "flywheel1AsDcMotor");
        imu = hardwareMap.get(IMU.class, "imu");
        Loader = hardwareMap.get(DcMotor.class, "LoaderAsDcMotor");

        // Wait for the match to begin.
        // This is actually a power setting and not speed
        COUNTS_PER_MOTOR_REV = 1440;
        // Gear reduction number was based off testing
        DRIVE_GEAR_REDUCTION = 0.285;
        WHEEL_DIAMETER_INCHES = 3;
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
        travelspeed = 0.85;
        TURN_SPEED = 0.5;
        runtime = new ElapsedTime();
        First_Distance = 21;
        Second_Distance = 43;
        redvsblue = 1;
        flywheelspeedshooting = 1600;
        // To drive forward, most robots need the motor on one side to be reversed, because the
        // axles point in opposite directions. When run, this OpMode should start both motors driving
        // forward. Adjust these motor directions based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.
        // Gear reduction or 90 degree drives may require direction flips.
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        backleftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontleftmotor.setDirection(DcMotor.Direction.REVERSE);
        backrightmotor.setDirection(DcMotor.Direction.FORWARD);
        frontrightmotor.setDirection(DcMotor.Direction.FORWARD);
        backleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Make all motors run in active brake mode
        backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Edit this to match your mounting configuration.
        orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        // Now initialize the IMU with this mounting orientation.
        // Rest Yaw (Z-axis) to 0 degree
        // Send telemetry message to signify robot waiting.
        telemetry.addData("Status", "Ready to run");
        telemetry.update();
        // Wait for the game to start (driver presses START).
        waitForStart();
        if (opModeIsActive()) {
            ((DcMotorEx) flywheel1).setVelocityPIDFCoefficients(200, 5, 10, 0);
            // Put run blocks here.
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw();
            // Don't want to go to the far spike mark and interfere with alliance partner
            telemetry.update();
            // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.
            // SECTION 1: 21
            // Step 1: Move Forward and Rotate to Long Distance Shot
            drive_encoder(travelspeed, 4, 4, 4);
            // Clockwise: Positive left, Negative right
            // Counter Clockwise: Negative left, Positive right
            drive_encoder(TURN_SPEED, (int) (4.3 * redvsblue), (int) (-4.3 * redvsblue), 4);
            // Step 2: Launch Artifact with (x) power
            // Wait is added after setting speed
            ((DcMotorEx) flywheel1).setVelocity(flywheelspeedshooting);
            launch(3000, 1560, 1300);
            // Step 3: Going to First Distance
            // Going to 21 (28 inches)
            drive_encoder(travelspeed, First_Distance, First_Distance, 4);
            // Step 4: Rotating to face artifacts (CHANGE THIS NUMBER)
            // COUNTER CLOCKWISE
            drive_encoder(TURN_SPEED, -20 * redvsblue, 20 * redvsblue, 4);
            // Step 5: Move Backwards and Intake
            intakeMotor.setPower(-1);
            // These have to add to x
            // NOT SURE IF IT IS 6
            drive_encoder(0.3, -28, -28, 4);
            sleep(50);
            // Step 6: Move Forward towards center line
            drive_encoder(travelspeed, 26, 26, 2);
            intakeMotor.setPower(0);
            // Step 7: Rotate to be facing straight again
            // CLOCK WISE
            drive_encoder(TURN_SPEED, 19 * redvsblue, -19 * redvsblue, 4);
            // Step 8: Drive back to the launch zone and rotate towards the target
            // Going back from 21's distance
            ((DcMotorEx) flywheel1).setVelocity(flywheelspeedshooting);
            drive_encoder(travelspeed, (int) (First_Distance * -0.95), (int) (First_Distance * -0.95), 2);
            // Step 9: Launch Artifacts
            launch(3000, 1560, 1300);
            // SECTION 2: 22
            // Step 1: Rotate slightly in order to get second set of artifacts
            // COUNTER CLOCK WISE
            drive_encoder(TURN_SPEED, (int) (-2.4 * redvsblue), (int) (2.4 * redvsblue), 4);
            // Step 2: Going to Second Distance
            // Going to 22 (47 inches)
            drive_encoder(travelspeed, Second_Distance, Second_Distance, 2);
            // Step 6: Rotate to face artifacts
            // COUNTER CLOCK WISE
            drive_encoder(TURN_SPEED, (int) (-18.5 * redvsblue), (int) (18.5 * redvsblue), 4);
            // Step 7: Move Backwards and Intake
            intakeMotor.setPower(-1);
            // These have to add to x
            // NOT SURE IF IT IS 9
            drive_encoder(0.3, -28, -28, 3);
            sleep(50);
            // Step 8: Move Forward towards center line
            drive_encoder(travelspeed, 27, 27, 2);
            intakeMotor.setPower(0);
            // Step 9: Rotate to be facing straight again
            // CLOCKWISE
            drive_encoder(TURN_SPEED, (int) (17.5 * redvsblue), (int) (-17.5 * redvsblue), 4);
            // Step 10: Drive to the LONG launch zone and rotate towards the target
            // Going back from 22s distance
            drive_encoder(travelspeed, (int) (Second_Distance * -0.95), (int) (Second_Distance * -0.95), 2);
            // CLOCKWISE
            drive_encoder(TURN_SPEED, (int) (1.7 * redvsblue), (int) (-1.7 * redvsblue), 4);
            // Step 11: Launch Artifacts
            ((DcMotorEx) flywheel1).setVelocity(flywheelspeedshooting);
            launch(3000, 1560, 1300);
            // Step 12: Leave Launch Zone
            drive_time2(300, 0.85, 50);
        }
    }

    /**
     * Describe this function...
     */
    private void drive_encoder(double speed, int leftinche, int rightinchess, int timeoutS) {
        double newLeftTarget;
        double newRightTarget;

        // Ensure that the OpMode is still active.
        if (opModeIsActive()) {
            // Rest all 4 motor decoder to 0
            backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backleftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Determine new target position, and pass to motor controller.
            // Assumes by reversing left side distance will alos be reveresed
            newLeftTarget = backleftMotor.getCurrentPosition() + Math.floor(leftinche * COUNTS_PER_INCH);
            newRightTarget = backrightmotor.getCurrentPosition() + Math.floor(rightinchess * COUNTS_PER_INCH);
            telemetry.addData("leftinche", leftinche);
            telemetry.addData("rightinchess", rightinchess);
            telemetry.addData("count/inches", COUNTS_PER_INCH);
            telemetry.update();
            backleftMotor.setTargetPosition((int) newLeftTarget);
            backrightmotor.setTargetPosition((int) newRightTarget);
            newLeftTarget = frontleftmotor.getCurrentPosition() + Math.floor(leftinche * COUNTS_PER_INCH);
            newRightTarget = frontrightmotor.getCurrentPosition() + Math.floor(rightinchess * COUNTS_PER_INCH);
            frontleftmotor.setTargetPosition((int) newLeftTarget);
            frontrightmotor.setTargetPosition((int) newRightTarget);
            // Turn On RUN_TO_POSITION.
            backleftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Reset the timeout time and start motion.
            runtime.reset();
            backleftMotor.setPower(Math.abs(speed));
            backrightmotor.setPower(Math.abs(speed));
            frontleftmotor.setPower(Math.abs(speed));
            frontrightmotor.setPower(Math.abs(speed));
            telemetry.addData("Running to", JavaUtil.formatNumber(newLeftTarget, 7, 0) + " :" + JavaUtil.formatNumber(newRightTarget, 7, 0));
            telemetry.addData("Currently at", JavaUtil.formatNumber(backleftMotor.getCurrentPosition(), 7, 0) + " :" + JavaUtil.formatNumber(backrightmotor.getCurrentPosition(), 7, 0) + JavaUtil.formatNumber(frontleftmotor.getCurrentPosition(), 7, 0) + JavaUtil.formatNumber(frontrightmotor.getCurrentPosition(), 7, 0));
            telemetry.update();
            // Keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() and isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() or isBusy()) in the loop test.
            while (opModeIsActive() && runtime.seconds() < timeoutS && frontleftmotor.isBusy() && frontrightmotor.isBusy() && backleftMotor.isBusy() && backrightmotor.isBusy()) {
                // Display it for the driver.
                telemetry.addData("Running to", JavaUtil.formatNumber(newLeftTarget, 7, 0) + " :" + JavaUtil.formatNumber(newRightTarget, 7, 0));
                telemetry.addData("Currently at", JavaUtil.formatNumber(backleftMotor.getCurrentPosition(), 7, 0) + " :" + JavaUtil.formatNumber(backrightmotor.getCurrentPosition(), 7, 0) + JavaUtil.formatNumber(frontleftmotor.getCurrentPosition(), 7, 0) + JavaUtil.formatNumber(frontrightmotor.getCurrentPosition(), 7, 0));
                // Added to allow time to read display
                telemetry.update();
            }
            // Stop all motion.
            backleftMotor.setPower(0);
            backrightmotor.setPower(0);
            frontleftmotor.setPower(0);
            frontrightmotor.setPower(0);
            // Turn off RUN_TO_POSITION.
            backleftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backleftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Optional pause after each move.
        }
    }

    /**
     * Describe this function...
     */
    private void launch(int time2, int speed, int Wait) {
        if (opModeIsActive()) {
            Loader.setPower(0);
            intakeMotor.setPower(0);
            runtime.reset();
            // Giving the flywheel more time to speed up before the first shot on 12/6/2025
            sleep(Wait);
            for (int count = 0; count < 3; count++) {
                Loader.setPower(-1);
                intakeMotor.setPower(-1);
                sleep(800);
                // Stop all motion.
                Loader.setPower(0);
                intakeMotor.setPower(0);
            }
            ((DcMotorEx) flywheel1).setVelocity(0);
        }
    }

    /**
     * Function to perform a relative move, based on encoder counts. Encoders
     * are not reset as the move is based on the current position. Move will
     * stop if any of three conditions occur: 1) Move gets to the desired
     * position 2) Move runs out of time 3) Driver stops the OpMode running.
     */
    private void drive_time2(int DriveTime, double DrivePwr, int Wait) {
        double Left_Drive_Scale;
        int Right_Drive_Scale;

        if (opModeIsActive()) {
            // Added so we can drive more straight 12/06
            Left_Drive_Scale = 0.98;
            Right_Drive_Scale = 1;
            runtime.reset();
            while (opModeIsActive() && runtime.milliseconds() < DriveTime) {
                backleftMotor.setPower(DrivePwr * Left_Drive_Scale);
                backrightmotor.setPower(DrivePwr * Right_Drive_Scale);
                frontleftmotor.setPower(DrivePwr * Left_Drive_Scale);
                frontrightmotor.setPower(DrivePwr * Right_Drive_Scale);
                // Stop all motion.
            }
            backleftMotor.setPower(0);
            backrightmotor.setPower(0);
            frontleftmotor.setPower(0);
            frontrightmotor.setPower(0);
            // Optional pause after each move.
            sleep(Wait);
        }
    }
}