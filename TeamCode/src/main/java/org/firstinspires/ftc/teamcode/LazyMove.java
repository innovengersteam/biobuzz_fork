package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "lazy move", group = "BioBuzz")
public class LazyMove extends LinearOpMode {
    // Pod model and mounting offsets are supplied. Verify encoder directions
    // by pushing the robot by hand before changing this flag to true.
    private static final boolean POD_SETUP_VERIFIED = true;
    private static final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    // Pinpoint X port MUST connect to the pod that rolls forward/backward.
    // Updated picture: X pod, 6 inches behind front and 6 inches left of center.
    // Forward-tracking pod: sideways offset from robot center; left positive.
    private static final double X_POD_OFFSET_MM = 152.4; // 6 inches left of center
    // Sideways-tracking pod: forward offset from robot center; forward positive.
    // Updated picture: Y pod, 1 inch behind front and 6 inches left of center.
    // Connect this sideways-tracking pod to the Pinpoint Y port.
    private static final double Y_POD_OFFSET_MM = 203.2; // 9 - 1 = 8 inches ahead of center
    private static final GoBildaPinpointDriver.EncoderDirection X_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED; // VERIFY
    private static final GoBildaPinpointDriver.EncoderDirection Y_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD; // VERIFY

    // Robot coordinates match Pinpoint: +X forward, +Y left.
    private static final double TARGET_INCHES = 15.0;
    private static final double TOLERANCE_INCHES = 0.25;
    private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(2.0);
    private static final double HEADING_KP = 1.5;
    private static final double MAX_TURN_POWER = 0.22;
    private static final double TIMEOUT_SECONDS = 8.0;
    private DcMotor[] drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // Control Hub motor ports 0, 1, 3; Expansion Hub motor port 2.
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
        hardwareMap.get(DcMotor.class, "intake").setPower(0);
        hardwareMap.get(DcMotor.class, "outtake").setPower(0);
        hardwareMap.get(CRServo.class, "transfer").setPower(0);
        hardwareMap.get(CRServo.class, "leftIntakeServo").setPower(0);
        hardwareMap.get(CRServo.class, "rightIntakeServo").setPower(0);
        // LED types are unspecified; this routine does not command them.

        GoBildaPinpointDriver odo = hardwareMap.get(
                GoBildaPinpointDriver.class, "odometry"); // Control Hub I2C port 1
        odo.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(POD_TYPE);
        odo.setEncoderDirections(X_DIRECTION, Y_DIRECTION);
        odo.resetPosAndIMU(); // Keep robot stationary during calibration.

        if (!POD_SETUP_VERIFIED) {
            while (!isStopRequested() && !isStarted()) {
                odo.update();
                Pose2D checkPose = odo.getPosition();
                telemetry.addData("Odometry status", odo.getDeviceStatus());
                telemetry.addData("X forward (inches)", "%.2f", checkPose.getX(DistanceUnit.INCH));
                telemetry.addData("Y left (inches)", "%.2f", checkPose.getY(DistanceUnit.INCH));
                telemetry.addData("Heading (degrees)", "%.1f", checkPose.getHeading(AngleUnit.DEGREES));
                telemetry.addLine("CHECK MODE: motors stopped. Do not press PLAY.");
                telemetry.addLine("Keep still until READY, then push by hand to check axes.");
                telemetry.addLine("Set POD_SETUP_VERIFIED=true only after verification.");
                telemetry.update();
                sleep(50);
            }
            return; // Never drive with unverified encoder directions.
        }

        try {
            while (!isStarted() && !isStopRequested()) {
                odo.update();
                telemetry.addData("Odometry status", odo.getDeviceStatus());
                telemetry.addLine("Keep stationary. Forward target: 15 inches.");
                telemetry.addData("Heading (degrees)", "%.1f",
                        odo.getPosition().getHeading(AngleUnit.DEGREES));
                telemetry.update();
                sleep(20);
            }
            if (isStopRequested()) return;
            odo.update();
            if (odo.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                telemetry.addLine("Not READY: check pods and calibration. No motion.");
                telemetry.update();
                return;
            }
            odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
            ElapsedTime timer = new ElapsedTime();
            String result = "Timeout: target not reached";
            while (opModeIsActive() && timer.seconds() < TIMEOUT_SECONDS) {
                odo.update();
                if (odo.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                    result = "Stopped: odometry fault";
                    break;
                }
                Pose2D pose = odo.getPosition();
                // Robot coordinates match Pinpoint: +X forward, +Y left.
                double robotX = pose.getX(DistanceUnit.INCH);
                double robotY = pose.getY(DistanceUnit.INCH);
                double heading = pose.getHeading(AngleUnit.RADIANS);
                if (!Double.isFinite(robotX) || !Double.isFinite(robotY)
                        || !Double.isFinite(heading)) {
                    result = "Stopped: invalid odometry";
                    break;
                }
                double error = TARGET_INCHES - robotX;
                double wrappedHeading = Math.atan2(Math.sin(heading), Math.cos(heading));
                boolean distanceReached = Math.abs(error) <= TOLERANCE_INCHES;
                boolean headingReached = Math.abs(wrappedHeading) <= HEADING_TOLERANCE_RADIANS;
                if (distanceReached && headingReached) {
                    result = "Target reached: within 0.25 inch and 2 degrees";
                    break;
                }
                double forward = distanceReached ? 0.0 : Math.copySign(
                        Range.clip(Math.abs(error) * 0.05, 0.10, 0.30), error);
                // Correct sideways drift toward robot Y = 0.
                // Robot +Y is left; the drive strafe command is positive right.
                double strafe = Range.clip(robotY * 0.04, -0.12, 0.12);
                // Pinpoint heading is positive CCW; drive turn is positive CW.
                double turn = headingReached ? 0.0 : Range.clip(
                        wrappedHeading * HEADING_KP, -MAX_TURN_POWER, MAX_TURN_POWER);
                setDrive(forward, strafe, turn);
                telemetry.addData("Robot X (forward inches)", "%.2f / 15.00", robotX);
                telemetry.addData("Robot Y (left inches)", "%.2f", robotY);
                telemetry.addData("Heading (degrees)", "%.1f", Math.toDegrees(wrappedHeading));
                telemetry.addData("Target heading (degrees)", "%.1f", 0.0);
                telemetry.addData("Heading error (degrees)", "%.1f", -Math.toDegrees(wrappedHeading));
                telemetry.update();
                sleep(20);
            }
            setDrive(0, 0, 0);
            telemetry.addLine(isStopRequested() ? "Stopped by driver" : result);
            telemetry.addData("Last heading (degrees)", "%.1f",
                    odo.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.update();
        } finally {
            setDrive(0, 0, 0);
        }
    }

    private void setDrive(double forward, double strafe, double turn) {
        double[] power = {forward + strafe + turn, forward - strafe - turn,
                forward - strafe + turn, forward + strafe - turn};
        double scale = 1.0;
        for (double value : power) scale = Math.max(scale, Math.abs(value));
        for (int i = 0; i < drive.length; i++) drive[i].setPower(power[i] / scale);
    }
}
