package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "BioBuzz TeleOp", group = "BioBuzz")
public class BioBuzzTeleOp extends LinearOpMode {

    // ==========================================
    // HARDWARE
    // ==========================================

    // CONTROL HUB
    // Motor Port 0 = leftFront
    // Motor Port 1 = rightFront
    // Motor Port 3 = leftRear
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;

    // EXPANSION HUB
    // Motor Port 0 = intake
    // Motor Port 1 = outtake (encoder connected)
    // Motor Port 2 = rightRear
    private DcMotor intake;
    private DcMotorEx outtake;
    private DcMotor rightRear;

    // CONTROL HUB SERVOS
    // Servo Port 0 = transfer
    // Servo Port 2 = leftIntakeServo
    private CRServo transfer;
    private CRServo leftIntakeServo;

    // EXPANSION HUB SERVOS
    // Servo Port 0 = rightIntakeServo
    private CRServo rightIntakeServo;

    // CONTROL HUB LED
    // Servo Port 3 = intakeLED (white PWM headlight)
    private Servo intakeLED;

    // EXPANSION HUB LEDS
    // Servo Port 2 = outtakeLED (white PWM headlight)
    // Servo Port 4 = transferLED (white PWM headlight)
    private Servo outtakeLED;
    private Servo transferLED;

    // ==========================================
    // SETTINGS
    // ==========================================

    private static final double DRIVE_SPEED = 0.75;
    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 1.0;
    private static final double INTAKE_SERVO_POWER = 1.0;

    // PWM headlight brightness positions (verify ON/OFF on hardware).
    private static final double LED_ON = 1.0;
    private static final double LED_OFF = 0.0;

    private static final double START_RPM = 2500.0;
    private static final double RPM_STEP = 50.0;
    private static final double MAX_RPM = 6000.0;

    // Converted goBILDA 1:1 outtake motor
    private static final double OUTTAKE_TICKS_PER_REV = 28.0;
    private static final double OUTTAKE_GEAR_RATIO = 1.0;
    private static final double OUTTAKE_RATED_RPM = 6000.0;

    // Change this in software if the shooting direction
    // needs reversing. Keep the corrected power wiring.
    private static final DcMotorSimple.Direction OUTTAKE_DIRECTION =
            DcMotorSimple.Direction.FORWARD;

    private double targetRPM = START_RPM;

    // ==========================================
    // TOGGLE STATES
    // ==========================================

    private boolean intakeOn = false;
    private boolean transferOn = false;
    private boolean outtakeOn = false;

    // Previous inputs for one action per press
    private boolean lastLeftTrigger = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightTrigger = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void runOpMode() {

        // ------------------------------------------
        // GET HARDWARE
        // ------------------------------------------

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        transfer = hardwareMap.get(CRServo.class, "transfer");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");

        // PWM-controlled white LED headlights.
        intakeLED = hardwareMap.get(Servo.class, "intakeLED");
        outtakeLED = hardwareMap.get(Servo.class, "outtakeLED");
        transferLED = hardwareMap.get(Servo.class, "transferLED");

        // ------------------------------------------
        // OUTTAKE MOTOR CONFIGURATION OVERRIDE
        // ------------------------------------------

        // The generic dropdown definition does not match
        // the converted 1:1 gearbox.
        // Clone first so other motors are unaffected.
        MotorConfigurationType outtakeType =
                outtake.getMotorType().clone();

        outtakeType.setTicksPerRev(OUTTAKE_TICKS_PER_REV);
        outtakeType.setGearing(OUTTAKE_GEAR_RATIO);
        outtakeType.setMaxRPM(OUTTAKE_RATED_RPM);

        outtake.setMotorType(outtakeType);

        // ------------------------------------------
        // DIRECTIONS
        // ------------------------------------------

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake.setDirection(OUTTAKE_DIRECTION);

        // ------------------------------------------
        // ZERO-POWER BEHAVIOR
        // ------------------------------------------

        leftFront.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        outtake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT);

        // ------------------------------------------
        // MOTOR MODES
        // ------------------------------------------

        // Drive and intake encoders are not connected.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Restore closed-loop outtake velocity control.
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Keep existing PIDF coefficients unchanged.
        stopEverything();

        // ------------------------------------------
        // INITIAL TELEMETRY
        // ------------------------------------------

        telemetry.addLine("=== BIOBUZZ TELEOP READY ===");
        telemetry.addLine("Left stick = Drive / Strafe");
        telemetry.addLine("Right stick X = Turn");
        telemetry.addLine("LT = Toggle intake");
        telemetry.addLine("LB = Toggle transfer");
        telemetry.addLine("RT = Toggle outtake");
        telemetry.addLine("D-pad Up/Down = +/- 50 RPM");

        telemetry.addData(
                "Starting target RPM", "%.0f", targetRPM);

        telemetry.addData(
                "Outtake ticks/rev",
                "%.2f",
                outtake.getMotorType().getTicksPerRev()
        );

        telemetry.addData(
                "Outtake gearing",
                "%.2f",
                outtake.getMotorType().getGearing()
        );

        telemetry.addData(
                "Outtake rated RPM",
                "%.0f",
                outtake.getMotorType().getMaxRPM()
        );

        telemetry.addLine("Press PLAY to start.");
        telemetry.update();

        try {
            waitForStart();

            if (isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {

                // ==================================
                // DRIVE
                // ==================================

                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                double lf = forward + strafe + turn;
                double rf = forward - strafe - turn;
                double lr = forward - strafe + turn;
                double rr = forward + strafe - turn;

                // Preserve wheel-power ratios while keeping
                // each normalized command within -1 to +1.
                double max = Math.max(
                        1.0,
                        Math.max(
                                Math.abs(lf),
                                Math.max(
                                        Math.abs(rf),
                                        Math.max(
                                                Math.abs(lr),
                                                Math.abs(rr)
                                        )
                                )
                        )
                );

                leftFront.setPower(lf / max * DRIVE_SPEED);
                rightFront.setPower(rf / max * DRIVE_SPEED);
                leftRear.setPower(lr / max * DRIVE_SPEED);
                rightRear.setPower(rr / max * DRIVE_SPEED);

                // ==================================
                // READ BUTTONS
                // ==================================

                boolean leftTrigger =
                        gamepad1.left_trigger > 0.5;

                boolean leftBumper =
                        gamepad1.left_bumper;

                boolean rightTrigger =
                        gamepad1.right_trigger > 0.5;

                boolean dpadUp =
                        gamepad1.dpad_up;

                boolean dpadDown =
                        gamepad1.dpad_down;

                // ==================================
                // TOGGLE MECHANISMS
                // ==================================

                if (leftTrigger && !lastLeftTrigger) {
                    intakeOn = !intakeOn;
                }

                if (leftBumper && !lastLeftBumper) {
                    transferOn = !transferOn;
                }

                if (rightTrigger && !lastRightTrigger) {
                    outtakeOn = !outtakeOn;
                }

                // ==================================
                // ADJUST TARGET RPM
                // ==================================

                if (dpadUp && !lastDpadUp) {
                    targetRPM += RPM_STEP;
                }

                if (dpadDown && !lastDpadDown) {
                    targetRPM -= RPM_STEP;
                }

                targetRPM = Range.clip(
                        targetRPM, 0.0, MAX_RPM);

                // Remember inputs for the next loop.
                lastLeftTrigger = leftTrigger;
                lastLeftBumper = leftBumper;
                lastRightTrigger = rightTrigger;
                lastDpadUp = dpadUp;
                lastDpadDown = dpadDown;

                // ==================================
                // COMMAND MECHANISMS
                // ==================================

                intake.setPower(
                        intakeOn ? INTAKE_POWER : 0.0);

                // Run both intake servos with the intake motor.
                leftIntakeServo.setPower(
                        intakeOn ? INTAKE_SERVO_POWER : 0.0);
                rightIntakeServo.setPower(
                        intakeOn ? INTAKE_SERVO_POWER : 0.0);

                transfer.setPower(
                        transferOn ? TRANSFER_POWER : 0.0);

                double commandedRPM =
                        outtakeOn ? targetRPM : 0.0;

                double requestedTicksPerSecond =
                        commandedRPM
                                * OUTTAKE_TICKS_PER_REV
                                / 60.0;

                outtake.setVelocity(requestedTicksPerSecond);

                // ==================================
                // LED HEADLIGHTS
                // ==================================

                // LEDs follow the commanded mechanism states.
                // Outtake LED stays OFF when target RPM is zero.
                intakeLED.setPosition(intakeOn ? LED_ON : LED_OFF);
                transferLED.setPosition(transferOn ? LED_ON : LED_OFF);
                outtakeLED.setPosition(
                        commandedRPM > 0.0 ? LED_ON : LED_OFF);

                // ==================================
                // READ OUTTAKE FEEDBACK
                // ==================================

                // One velocity sample per loop.
                double actualTicksPerSecond =
                        outtake.getVelocity();

                double signedActualRPM =
                        actualTicksPerSecond
                                * 60.0
                                / OUTTAKE_TICKS_PER_REV;

                double actualRPM =
                        Math.abs(signedActualRPM);

                double signedRPMError =
                        commandedRPM - signedActualRPM;

                // ==================================
                // TELEMETRY
                // ==================================

                telemetry.addLine("=== BIOBUZZ TELEOP ===");

                telemetry.addData(
                        "Intake", intakeOn ? "ON" : "OFF");
                telemetry.addData(
                        "Left Intake Servo", intakeOn ? "ON" : "OFF");
                telemetry.addData(
                        "Right Intake Servo", intakeOn ? "ON" : "OFF");
                telemetry.addData(
                        "Transfer", transferOn ? "ON" : "OFF");
                telemetry.addData(
                        "Outtake", outtakeOn ? "ON" : "OFF");

                telemetry.addLine("--- LED HEADLIGHTS ---");
                telemetry.addData("Intake LED", intakeOn ? "ON" : "OFF");
                telemetry.addData("Transfer LED", transferOn ? "ON" : "OFF");
                telemetry.addData("Outtake LED",
                        commandedRPM > 0.0 ? "ON" : "OFF");

                telemetry.addLine("--- OUTTAKE ---");

                telemetry.addData(
                        "Target RPM",
                        "%.0f",
                        targetRPM
                );

                telemetry.addData(
                        "Commanded RPM",
                        "%.0f",
                        commandedRPM
                );

                telemetry.addData(
                        "Actual RPM",
                        "%.0f",
                        actualRPM
                );

                telemetry.addData(
                        "Actual SIGNED RPM",
                        "%.0f",
                        signedActualRPM
                );

                telemetry.addData(
                        "Signed RPM error",
                        "%.0f",
                        signedRPMError
                );

                telemetry.addData(
                        "Requested ticks/sec",
                        "%.1f",
                        requestedTicksPerSecond
                );

                telemetry.addData(
                        "Measured SIGNED ticks/sec",
                        "%.1f",
                        actualTicksPerSecond
                );

                telemetry.addData(
                        "Encoder position",
                        outtake.getCurrentPosition()
                );

                telemetry.addLine("--- CONFIGURATION ---");

                telemetry.addData(
                        "Motor mode",
                        outtake.getMode()
                );

                telemetry.addData(
                        "Motor direction",
                        outtake.getDirection()
                );

                telemetry.addData(
                        "Configured ticks/rev",
                        "%.2f",
                        outtake.getMotorType().getTicksPerRev()
                );

                telemetry.addData(
                        "Configured gearing",
                        "%.2f",
                        outtake.getMotorType().getGearing()
                );

                telemetry.addData(
                        "Configured maximum RPM",
                        "%.0f",
                        outtake.getMotorType().getMaxRPM()
                );

                telemetry.addData(
                        "Velocity PIDF",
                        outtake.getPIDFCoefficients(
                                DcMotor.RunMode.RUN_USING_ENCODER
                        )
                );

                telemetry.update();
                idle();
            }

        } finally {
            stopEverything();
        }
    }

    // ==========================================
    // STOP EVERYTHING
    // ==========================================

    private void stopEverything() {
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);

        intake.setPower(0.0);
        leftIntakeServo.setPower(0.0);
        rightIntakeServo.setPower(0.0);
        transfer.setPower(0.0);
        outtake.setVelocity(0.0);

        // Turn off all three LED headlights at init and stop.
        intakeLED.setPosition(LED_OFF);
        transferLED.setPosition(LED_OFF);
        outtakeLED.setPosition(LED_OFF);
    }
}
