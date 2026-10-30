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

@Autonomous(name = "Shoot Collect Park", group = "BioBuzz")
public class ShootCollectPark extends LinearOpMode {
    // ==========================================
    // AUTONOMOUS PLAN (PSEUDOCODE)
    // ==========================================
    // 1. Run outtake at 2,650 RPM alone for 2 seconds. Then run outtake
    //    and transfer together for 4 seconds. Stop both.
    // 2. Move forward 4 inches.
    // 3. Start intake motor and both intake servos while strafing LEFT 48 inches.
    // 4. Move backward 4 inches, then remain stationary for 3 seconds; intake ON.
    // 5. Stop all intake components, then move forward 4 inches.
    // 6. Strafe RIGHT 7 inches, then move forward 78 inches and stop.
    // 7. If a movement makes no meaningful progress for 1 second, stop
    //    the wheels and continue from the actual position to the next step.
    // The one-second detection interval is the only obstacle delay; no extra wait.
    // STOP, odometry faults, movement safety timeouts, or the 30-second limit
    // still end the routine. Field testing must confirm completion in time.

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
    // Change these values if the timing or test distance needs adjustment.
    private static final double SPINUP_SECONDS = 2.0;
    private static final double SHOOT_SECONDS = 5.0;
    private static final double OUTTAKE_RPM = 2650.0;
    private static final double OUTTAKE_TICKS_PER_REV = 28.0;
    private static final double TRANSFER_POWER = 1.0;
    private static final double FORWARD_INCHES = 4.0;
    private static final double STRAFE_LEFT_INCHES = 50.0;
    private static final double BACKWARD_INCHES = 4.0;
    private static final double HOLD_SECONDS = 3.0;
    private static final double STRAFE_RIGHT_INCHES = 7.0;
    private static final double FINAL_FORWARD_INCHES = 78.0;
    // Only the last forward move uses this higher power limit.
    // Start at 0.45 and verify stopping distance on the actual robot.
    private static final double FINAL_FORWARD_MAX_POWER = 0.45;
    private static final double INTAKE_POWER = 1.0;
    private static final double AUTO_SECONDS = 30.0;
    private final ElapsedTime autoTimer = new ElapsedTime();
    // Acceptable distance and heading errors, plus movement safety limits.
    private static final double DISTANCE_TOLERANCE = 0.25;
    private static final double HEADING_TOLERANCE = Math.toRadians(2.0);
    private static final double HEADING_KP = 1.5;
    private static final double MAX_TURN_POWER = 0.22;
    private static final double DRIVE_TIMEOUT_SECONDS = 18.0;
    private static final double NO_PROGRESS_SECONDS = 1.0;
    private static final double MIN_PROGRESS_INCHES = 0.10;
    // Convert heading error to an equivalent distance for progress monitoring.
    private static final double HEADING_PROGRESS_RADIUS_INCHES = 4.0;
    private int skippedMoves = 0;
    private String lastSkippedMove = "None";

    private DcMotor[] drive;
    private DcMotorEx outtake;
    private CRServo transfer;
    private DcMotor intake;
    private CRServo leftIntakeServo, rightIntakeServo;
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

        // Match intake directions and modes to the attached TeleOp.
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        setIntake(false);
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

            autoTimer.reset();

            // STEP 1A: Spin up the outtake at 2,650 RPM for 2 seconds.
            // Keep transfer OFF so no pollen is fed during spin-up.
            transfer.setPower(0);
            outtake.setVelocity(OUTTAKE_RPM * OUTTAKE_TICKS_PER_REV / 60.0);
            if (!waitShootingPhase(SPINUP_SECONDS, "Step 1: Outtake spin-up; transfer OFF")) return;

            // STEP 1B: Keep outtake running and feed with transfer for 5  seconds.
            transfer.setPower(TRANSFER_POWER);
            if (!waitShootingPhase(SHOOT_SECONDS, "Step 1: Outtake and transfer ON")) return;
            transfer.setPower(0);
            outtake.setVelocity(0);
            if (!autoActive()) return;

            // STEP 2: Move FORWARD 4 inches from the starting position.
            if (!resetLocalPose()) return;
            if (!moveLocal(FORWARD_INCHES, 0, "Step 2: Forward 4 inches")) return;

            // STEP 3: Start intake motor and BOTH intake servos for the left strafe.
            setIntake(true);

            // STEP 3 (continued): Strafe LEFT 50 inches without turning; intake ON.
            if (!resetLocalPose()) return;
            if (!moveLocal(0, STRAFE_LEFT_INCHES, "Step 3: Left 50 inches")) return;

            // STEP 4: Move BACKWARD 4 inches; keep intake ON.
            if (!resetLocalPose()) return;
            if (!moveLocal(-BACKWARD_INCHES, 0, "Step 4: Backward 4 inches")) return;

            // STEP 4 (continued): Stay here for 3 seconds with intake ON.
            // Wheels are stopped with BRAKE zero-power behavior.
            setDrive(0, 0, 0);
            ElapsedTime holdTimer = new ElapsedTime();
            while (autoActive() && holdTimer.seconds() < HOLD_SECONDS) {
                telemetry.addData("Step 4", "Stationary for 3 seconds; intake ON");
                telemetry.addData("Hold seconds left", "%.1f",
                        Math.max(0, HOLD_SECONDS - holdTimer.seconds()));
                telemetry.update();
                sleep(20);
            }
            if (!autoActive()) return;

            // STEP 5: Stop the intake motor and BOTH intake servos.
            setIntake(false);

            // STEP 5 (continued): Move FORWARD 4 inches with intake OFF.
            if (!resetLocalPose()) return;
            if (!moveLocal(FORWARD_INCHES, 0, "Step 5: Forward 4 inches")) return;

            // STEP 6: Strafe RIGHT 6 inches; right is negative Pinpoint Y.
            if (!resetLocalPose()) return;
            if (!moveLocal(0, -STRAFE_RIGHT_INCHES, "Step 6: Right 6 inches")) return;

            // STEP 6 (continued): Move FORWARD 78 inches, then stop.
            if (!resetLocalPose()) return;
            if (!moveLocal(FINAL_FORWARD_INCHES, 0, "Step 6: Forward 78 inches", FINAL_FORWARD_MAX_POWER)) return;
            telemetry.addLine("Routine complete (blocked movements may have been skipped).");
            telemetry.addData("Skipped movements", skippedMoves);
            telemetry.addData("Last skipped movement", lastSkippedMove);
            telemetry.update();
        } finally {
            // Always stop ALL moving hardware when autonomous ends or a step fails.
            setDrive(0, 0, 0);
            transfer.setPower(0);
            outtake.setVelocity(0);
            setIntake(false);
        }
    }

    // HELPER: Wait during a shooting phase while showing RPM and remaining time.
    private boolean waitShootingPhase(double seconds, String phase)
            throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        while (autoActive() && timer.seconds() < seconds) {
            telemetry.addData("Step", phase);
            telemetry.addData("Phase seconds left", "%.1f", Math.max(0, seconds - timer.seconds()));
            telemetry.addData("Target RPM", OUTTAKE_RPM);
            telemetry.addData("Actual signed RPM", "%.0f",
                    outtake.getVelocity() * 60.0 / OUTTAKE_TICKS_PER_REV);
            telemetry.addData("Auto seconds left", "%.1f", Math.max(0, AUTO_SECONDS - autoTimer.seconds()));
            telemetry.update();
            sleep(20);
        }
        return autoActive();
    }

    // HELPER: Keep all steps within the autonomous time limit and honor STOP.
    private boolean autoActive() {
        return opModeIsActive() && autoTimer.seconds() < AUTO_SECONDS;
    }

    // HELPER: Define a new local frame at the current position and heading.
    // Each move starts from zero at the current robot position and direction.
    private boolean resetLocalPose() {
        if (!autoActive() || !odometryReady()) return false;
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
        return true;
    }

    // HELPER: Run the intake motor and both intake servos together.
    private void setIntake(boolean enabled) {
        double power = enabled ? INTAKE_POWER : 0.0;
        intake.setPower(power);
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }

    // HELPER: Move to a local X/Y target while holding the starting heading.
    // Positive X is forward; positive Y is left. Distances are in inches.
    // Reset the local pose before calling so distances are relative to this step.
    private boolean moveLocal(double targetX, double targetY, String step)
            throws InterruptedException {
        // Earlier steps retain the working 0.30 forward power limit.
        return moveLocal(targetX, targetY, step, 0.30);
    }

    // Overload lets the final forward move use a higher limit.
    // Proportional control still slows the robot as remaining distance decreases.
    private boolean moveLocal(double targetX, double targetY, String step,
                              double maxForwardPower) throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime progressTimer = new ElapsedTime();
        double bestRemainingError = Double.POSITIVE_INFINITY;
        while (autoActive() && timer.seconds() < DRIVE_TIMEOUT_SECONDS) {
            if (!odometryReady()) { setDrive(0, 0, 0); return false; }
            Pose2D pose = odo.getPosition();
            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.RADIANS);
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading))
                return fail("Invalid odometry");
            double errorX = targetX - x;
            double errorY = targetY - y;
            double headingError = wrap(heading);
            boolean xReached = Math.abs(errorX) <= DISTANCE_TOLERANCE;
            boolean yReached = Math.abs(errorY) <= DISTANCE_TOLERANCE;
            boolean headingReached = Math.abs(headingError) <= HEADING_TOLERANCE;
            if (xReached && yReached && headingReached) {
                setDrive(0, 0, 0);
                return true;
            }
            // Pinpoint estimates motion; it cannot identify an obstacle directly.
            // Track improvement toward the target rather than wheel rotation.
            // Include only heading error outside its acceptable tolerance.
            double remainingError = Math.hypot(errorX, errorY)
                    + HEADING_PROGRESS_RADIUS_INCHES
                    * Math.max(0, Math.abs(headingError) - HEADING_TOLERANCE);
            if (remainingError <= bestRemainingError - MIN_PROGRESS_INCHES) {
                bestRemainingError = remainingError;
                progressTimer.reset();
            }
            if (progressTimer.seconds() >= NO_PROGRESS_SECONDS) {
                // Stop pushing; the one second without progress has already elapsed.
                // Return success-to-continue, but record that the target was NOT met.
                setDrive(0, 0, 0);
                skippedMoves++;
                lastSkippedMove = step;
                telemetry.addData("Movement skipped", step);
                telemetry.addLine("No progress for 1 second; continuing from actual position.");
                telemetry.addData("Forward inches", "%.2f / %.2f", x, targetX);
                telemetry.addData("Left inches", "%.2f / %.2f", y, targetY);
                telemetry.update();
                return autoActive();
            }
            // Forward travel uses this step's limit; strafe keeps its original limit.
            // Limit cross-axis corrections to reduce unwanted drift.
            double forward = xReached ? 0 : (targetX == 0
                                             ? Range.clip(errorX * 0.04, -0.12, 0.12)
                                             : Math.copySign(Range.clip(Math.abs(errorX) * 0.05, 0.10, maxForwardPower), errorX));
            double strafe = yReached ? 0 : (targetY == 0
                                            ? Range.clip(-errorY * 0.04, -0.12, 0.12)
                                            : -Math.copySign(Range.clip(Math.abs(errorY) * 0.05, 0.10, 0.30), errorY));
            double turn = headingReached ? 0 : Range.clip(
                    headingError * HEADING_KP, -MAX_TURN_POWER, MAX_TURN_POWER);
            setDrive(forward, strafe, turn);
            telemetry.addData("Step", step);
            telemetry.addData("Forward power limit", "%.2f", maxForwardPower);
            telemetry.addData("No-progress seconds", "%.2f / %.1f",
                    progressTimer.seconds(), NO_PROGRESS_SECONDS);
            telemetry.addData("Skipped movements", skippedMoves);
            telemetry.addData("Last skipped movement", lastSkippedMove);
            telemetry.addData("Auto seconds left", "%.1f", Math.max(0, AUTO_SECONDS - autoTimer.seconds()));
            telemetry.addData("Move seconds left", "%.1f", Math.max(0, DRIVE_TIMEOUT_SECONDS - timer.seconds()));
            telemetry.addData("Forward inches", "%.2f / %.2f", x, targetX);
            telemetry.addData("Left inches", "%.2f / %.2f", y, targetY);
            telemetry.addData("Heading degrees", "%.1f", Math.toDegrees(headingError));
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested() || !opModeIsActive()) return fail(step + ": STOP requested");
        if (autoTimer.seconds() >= AUTO_SECONDS) return fail(step + ": 30-second auto limit reached");
        return fail(step + ": movement timeout; position/heading tolerance not reached");
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
