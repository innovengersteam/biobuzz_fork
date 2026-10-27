package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Wait Shoot Park", group = "BioBuzz")
public class WaitShootPark extends LinearOpMode {
    // ==========================================
    // AUTONOMOUS PLAN (PSEUDOCODE)
    // ==========================================
    // 1. Drive forward 4 inches.
    // 2. Turn left 43 degrees to face the shooting direction.
    // 3. Wait 15 seconds for our alliance partner to tip the hive.
    // 4. Start the outtake at 2,800 RPM and the transfer together.
    // 5. Shoot for 5 seconds, then stop both mechanisms.
    // 6. Turn right to return to the starting heading.
    // 7. Drive forward 11 inches.
    // 8. Strafe right 8 inches to the parking spot.
    // If a movement fails or STOP is pressed, stop and end the routine.

    // ==========================================
    // PINPOINT SETTINGS
    // ==========================================
    // These are the same settings used by the working LazyMove program.
    private static final boolean POD_SETUP_VERIFIED = true;
    private static final double X_POD_OFFSET_MM = 152.4;
    private static final double Y_POD_OFFSET_MM = 203.2;
    private static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    private static final GoBildaPinpointDriver.EncoderDirection X_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    private static final GoBildaPinpointDriver.EncoderDirection Y_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // ==========================================
    // AUTONOMOUS SETTINGS
    // ==========================================
    // Change these values if the timing or parking distance needs adjustment.
    private static final double WAIT_SECONDS = 15.0;
    private static final double SHOOT_SECONDS = 5.0;
    private static final double OUTTAKE_RPM = 2850.0;
    private static final double OUTTAKE_TICKS_PER_REV = 28.0;
    private static final double TRANSFER_POWER = 1.0;
    private static final double INITIAL_FORWARD_INCHES = 4.0;
    private static final double PARK_FORWARD_INCHES = 11.0;
    private static final double PARK_STRAFE_RIGHT_INCHES = 10.0;
    // Acceptable distance and heading errors, plus movement safety limits.
    private static final double DISTANCE_TOLERANCE = 0.25;
    private static final double HEADING_TOLERANCE = Math.toRadians(2.0);
    private static final double HEADING_KP = 1.5;
    private static final double MAX_TURN_POWER = 0.22;
    private static final double TURN_TIMEOUT_SECONDS = 8.0;
    private static final double DRIVE_TIMEOUT_SECONDS = 8.0;

    private DcMotor[] drive;
    private DcMotorEx outtake;
    private CRServo transfer;
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() throws InterruptedException {
        // STEP 0: Set up the four drive motors using their configured names.
        DcMotor lf = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rf = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor lr = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rr = hardwareMap.get(DcMotor.class, "rightRear");
        drive = new DcMotor[]{lf, rf, lr, rr};
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotor motor : drive) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Keep the intake and its servos stopped during autonomous.
        hardwareMap.get(DcMotor.class, "intake").setPower(0);
        hardwareMap.get(CRServo.class, "leftIntakeServo").setPower(0);
        hardwareMap.get(CRServo.class, "rightIntakeServo").setPower(0);
        // Set up the transfer and outtake with the same directions as TeleOp.
        transfer = hardwareMap.get(CRServo.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setPower(0);
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Match the 1:1 outtake motor configuration in BioBuzz TeleOp.
        MotorConfigurationType outtakeType = outtake.getMotorType().clone();
        outtakeType.setTicksPerRev(OUTTAKE_TICKS_PER_REV);
        outtakeType.setGearing(1.0);
        outtakeType.setMaxRPM(6000.0);
        outtake.setMotorType(outtakeType);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setVelocity(0);

        // Set up Pinpoint so it can measure position and heading.
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
        odo.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(POD_TYPE);
        odo.setEncoderDirections(X_DIRECTION, Y_DIRECTION);
        odo.resetPosAndIMU(); // Keep the robot stationary during calibration.

        try {
            // Before PLAY: wait while Pinpoint calibrates and show its status.
            while (!isStarted() && !isStopRequested()) {
                odo.update();
                telemetry.addData("Odometry status", odo.getDeviceStatus());
                telemetry.addData("Heading (degrees)", "%.1f",
                        odo.getPosition().getHeading(AngleUnit.DEGREES));
                telemetry.addLine(POD_SETUP_VERIFIED
                        ? "Keep robot stationary; press PLAY when READY."
                        : "CHECK MODE: verify pods before enabling motion.");
                telemetry.update();
                sleep(20);
            }
            if (isStopRequested() || !POD_SETUP_VERIFIED) return;
            if (!odometryReady()) return;
            // Record the starting position and heading as zero.
            odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));

            // STEP 1: Drive forward 4 inches from the starting position.
            if (!driveForward(INITIAL_FORWARD_INCHES)) return;

            // STEP 2: Turn left 43 degrees toward the shooting direction.
            // If the turn fails, stop here instead of continuing.
            if (!turnTo(Math.toRadians(43))) return;
            outtake.setVelocity(OUTTAKE_RPM * OUTTAKE_TICKS_PER_REV / 60.0);

            // STEP 3: Stay still for 15 seconds while the alliance partner tips the hive.
            if (!waitWithTelemetry(WAIT_SECONDS)) return;

            // STEP 4: Start the outtake at 2,800 RPM and start the transfer.
            // Both start together; the flywheel may still be spinning up as feeding begins.
            // outtake.setVelocity(OUTTAKE_RPM * OUTTAKE_TICKS_PER_REV / 60.0);
            transfer.setPower(TRANSFER_POWER);
            // STEP 5: Keep shooting for 5 seconds and display the outtake speed.
            ElapsedTime shootTimer = new ElapsedTime();
            while (opModeIsActive() && shootTimer.seconds() < SHOOT_SECONDS) {
                telemetry.addData("Step", "Shooting");
                telemetry.addData("Seconds remaining", "%.1f", SHOOT_SECONDS - shootTimer.seconds());
                telemetry.addData("Target RPM", "%.0f", OUTTAKE_RPM);
                telemetry.addData("Actual signed RPM", "%.0f",
                        outtake.getVelocity() * 60.0 / OUTTAKE_TICKS_PER_REV);
                telemetry.update();
                sleep(20);
            }
            // Stop feeding and stop commanding the flywheel after shooting.
            transfer.setPower(0);
            outtake.setVelocity(0);
            if (!opModeIsActive()) return;

            // STEP 6: Turn right to the original starting heading (0 degrees).
            if (!turnTo(0)) return;
            if (!odometryReady()) return;
            // Start the parking move in the robot's current direction.
            odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
            // STEP 7: Drive forward 11 inches.
            if (!driveForward(PARK_FORWARD_INCHES)) return;
            if (!odometryReady()) return;

            // Reset the local pose so the strafe starts from zero at the
            // robot's current position and heading.
            odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));

            // STEP 8: Strafe right 8 inches to the parking spot.
            if (!strafeRight(PARK_STRAFE_RIGHT_INCHES)) return;
            telemetry.addLine("Parking complete");
            telemetry.update();
        } finally {
            // Always stop the drive, transfer and outtake when the routine ends.
            setDrive(0, 0, 0);
            transfer.setPower(0);
            outtake.setVelocity(0);
        }
    }

    // ==========================================
    // HELPER: TURN TO A HEADING
    // ==========================================
    // Compare the target heading with Pinpoint, turn toward it, and stop
    // within the tolerance. A fault or timeout prevents later steps.
    private boolean turnTo(double targetHeading) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < TURN_TIMEOUT_SECONDS) {
            if (!odometryReady()) { setDrive(0, 0, 0); return false; }
            double heading = odo.getPosition().getHeading(AngleUnit.RADIANS);
            if (!Double.isFinite(heading)) return fail("Invalid heading");
            double error = wrap(targetHeading - heading);
            if (Math.abs(error) <= HEADING_TOLERANCE) {
                setDrive(0, 0, 0);
                return true;
            }
            // Pinpoint positive heading is left; positive drive turn is right.
            double turn = -Math.copySign(
                    Range.clip(Math.abs(error) * HEADING_KP, 0.10, MAX_TURN_POWER), error);
            setDrive(0, 0, turn);
            telemetry.addData("Step", "Turning");
            telemetry.addData("Target heading", "%.1f degrees", Math.toDegrees(targetHeading));
            telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(heading));
            telemetry.addData("Heading error", "%.1f degrees", Math.toDegrees(error));
            telemetry.update();
            sleep(20);
        }
        return fail("Turn timed out or stopped");
    }

    // HELPER: Wait while showing the remaining time on the Driver Station.
    private boolean waitWithTelemetry(double seconds) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            telemetry.addData("Step", "Waiting for alliance partner");
            telemetry.addData("Seconds remaining", "%.1f", seconds - timer.seconds());
            telemetry.update();
            sleep(20);
        }
        return opModeIsActive();
    }

    // HELPER: Drive forward using Pinpoint for distance, sideways drift,
    // and heading corrections. Stop on a fault or timeout.
    private boolean driveForward(double target) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < DRIVE_TIMEOUT_SECONDS) {
            if (!odometryReady()) { setDrive(0, 0, 0); return false; }
            Pose2D pose = odo.getPosition();
            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.RADIANS);
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading))
                return fail("Invalid odometry");
            double error = target - x;
            double wrappedHeading = wrap(heading);
            boolean distanceReached = Math.abs(error) <= DISTANCE_TOLERANCE;
            boolean headingReached = Math.abs(wrappedHeading) <= HEADING_TOLERANCE;
            if (distanceReached && headingReached) {
                setDrive(0, 0, 0);
                return true;
            }
            // Slow down near the target and correct drift and heading.
            double forward = distanceReached ? 0.0 : Math.copySign(
                    Range.clip(Math.abs(error) * 0.05, 0.10, 0.30), error);
            double strafe = Range.clip(y * 0.04, -0.12, 0.12);
            double turn = headingReached ? 0.0 : Range.clip(
                    wrappedHeading * HEADING_KP, -MAX_TURN_POWER, MAX_TURN_POWER);
            setDrive(forward, strafe, turn);
            telemetry.addData("Step", "Driving forward");
            telemetry.addData("Forward inches", "%.2f / %.2f", x, target);
            telemetry.addData("Left inches", "%.2f", y);
            telemetry.addData("Heading degrees", "%.1f", Math.toDegrees(wrappedHeading));
            telemetry.update();
            sleep(20);
        }
        return fail("Parking drive timed out or stopped");
    }

    // HELPER: Strafe right using the Y pod and hold the starting heading.
    // Pinpoint Y is positive to the left, so right is negative Y.
    private boolean strafeRight(double inches) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        double targetY = -inches;
        while (opModeIsActive() && timer.seconds() < DRIVE_TIMEOUT_SECONDS) {
            if (!odometryReady()) { setDrive(0, 0, 0); return false; }
            Pose2D pose = odo.getPosition();
            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.RADIANS);
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading))
                return fail("Invalid odometry");

            double errorY = targetY - y;
            double wrappedHeading = wrap(heading);
            boolean distanceReached = Math.abs(errorY) <= DISTANCE_TOLERANCE;
            boolean headingReached = Math.abs(wrappedHeading) <= HEADING_TOLERANCE;
            if (distanceReached && headingReached) {
                setDrive(0, 0, 0);
                return true;
            }
            // Positive drive strafe is right, opposite to Pinpoint positive Y.
            double strafe = distanceReached ? 0.0 : -Math.copySign(
                    Range.clip(Math.abs(errorY) * 0.05, 0.10, 0.30), errorY);
            // Correct unwanted forward/backward drift while strafing.
            double forward = Range.clip(-x * 0.04, -0.12, 0.12);
            double turn = headingReached ? 0.0 : Range.clip(
                    wrappedHeading * HEADING_KP, -MAX_TURN_POWER, MAX_TURN_POWER);
            setDrive(forward, strafe, turn);
            telemetry.addData("Step", "Strafing right to parking spot");
            telemetry.addData("Right inches", "%.2f / %.2f", -y, inches);
            telemetry.addData("Forward drift (inches)", "%.2f", x);
            telemetry.addData("Heading degrees", "%.1f", Math.toDegrees(wrappedHeading));
            telemetry.update();
            sleep(20);
        }
        return fail("Parking strafe timed out or stopped");
    }

    // HELPER: Update Pinpoint and make sure it is ready to guide the robot.
    private boolean odometryReady() {
        odo.update();
        if (odo.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            telemetry.addData("Stopped: odometry", odo.getDeviceStatus());
            telemetry.update();
            return false;
        }
        return true;
    }

    // HELPER: Stop the wheels and report why a movement could not finish.
    private boolean fail(String message) {
        setDrive(0, 0, 0);
        telemetry.addLine(message);
        telemetry.update();
        return false;
    }

    // HELPER: Keep a heading error between -180 and +180 degrees.
    private double wrap(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    // HELPER: Convert forward, sideways, and turning commands into wheel power.
    private void setDrive(double forward, double strafe, double turn) {
        double[] power = {forward + strafe + turn, forward - strafe - turn,
                forward - strafe + turn, forward + strafe - turn};
        double scale = 1.0;
        for (double value : power) scale = Math.max(scale, Math.abs(value));
        for (int i = 0; i < drive.length; i++) drive[i].setPower(power[i] / scale);
    }
}
