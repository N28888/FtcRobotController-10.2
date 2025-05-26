package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx; // Using DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TankDrive4_Gemini", group = "TeleOp4")
public class Tankdrive4_Gemini extends LinearOpMode {

    // --- constants ---
    // MECANUM DRIVE
    static final double DRIVE_SPEED_MULTIPLIER = 1.0;
    static final double STRAFE_SPEED_MULTIPLIER = 1.1;

    // VIPER SLIDE
    static final double SLIDE_MOTOR_TICKS_PER_REVOLUTION = 537.7; // Example: GoBILDA 5203 series motor (19.2:1)
    static final double SLIDE_GEAR_RATIO = 1.0;
    static final double SLIDE_PULLEY_DIAMETER_MM = 30.0; // Effective diameter
    static final double SLIDE_MM_PER_REVOLUTION = Math.PI * SLIDE_PULLEY_DIAMETER_MM;
    static final double SLIDE_TICKS_PER_MM = (SLIDE_MOTOR_TICKS_PER_REVOLUTION * SLIDE_GEAR_RATIO) / SLIDE_MM_PER_REVOLUTION;

    static final int SLIDE_MAX_EXTENSION_MM = 240 * 4; // 4 stages at 240mm each
    static final int SLIDE_MIN_POSITION_TICKS = 0;
    static final int SLIDE_MAX_POSITION_TICKS = (int) (SLIDE_MAX_EXTENSION_MM * SLIDE_TICKS_PER_MM);
    static final int SLIDE_TARGET_POSITION_TOLERANCE = 15; // Ticks for RUN_TO_POSITION

    // Preset positions (in ticks)
    static final int SLIDE_GROUND_JUNCTION_TICKS = (int) (10 * SLIDE_TICKS_PER_MM);  // Adjust as needed (e.g., 10mm)
    static final int SLIDE_LOW_JUNCTION_TICKS = (int) (343 * SLIDE_TICKS_PER_MM);
    static final int SLIDE_MEDIUM_JUNCTION_TICKS = (int) (597 * SLIDE_TICKS_PER_MM);
    static final int SLIDE_HIGH_JUNCTION_TICKS = (int) (851 * SLIDE_TICKS_PER_MM);

    static final double SLIDE_MANUAL_POWER_MULTIPLIER = 0.9; // Max power for manual control
    static final double SLIDE_AUTO_POWER = 0.8;         // Power for RUN_TO_POSITION movements
    // ** NEW ** Power to apply to hold position if BRAKE mode is not enough.
    // Start with a small value like 0.05. If BRAKE is sufficient, this can be 0.0.
    static final double SLIDE_HOLD_POWER = 0.05;

    // ODOMETRY
    static final double ODOMETRY_WHEEL_DIAMETER_INCHES = 2.0;
    static final double ODOMETRY_TICKS_PER_REVOLUTION = 8192; // REV Through Bore Encoder
    static final double ODOMETRY_INCHES_PER_TICK = (Math.PI * ODOMETRY_WHEEL_DIAMETER_INCHES) / ODOMETRY_TICKS_PER_REVOLUTION;
    // For odometry calculations, you might need to reverse the sign of readings
    // from some encoders depending on their orientation.
    // Example: static final boolean REVERSE_LEFT_ODO = false;
    //          static final boolean REVERSE_RIGHT_ODO = true;
    //          static final boolean REVERSE_PERP_ODO = false;


    // --- Hardware Declarations ---
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotorEx viperSlideMotor = null;
    private DcMotor leftOdometryEncoder = null;
    private DcMotor rightOdometryEncoder = null;
    private DcMotor perpendicularOdometryEncoder = null;

    // --- Odometry Variables ---
    private int currentLeftOdoTicks = 0;
    private int currentRightOdoTicks = 0;
    private int currentPerpendicularOdoTicks = 0;
    private int prevLeftOdoTicks = 0;
    private int prevRightOdoTicks = 0;
    private int prevPerpendicularOdoTicks = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        viperSlideMotor = hardwareMap.get(DcMotorEx.class, "viperSlideMotor");
        leftOdometryEncoder = hardwareMap.get(DcMotor.class, "leftOdo");
        rightOdometryEncoder = hardwareMap.get(DcMotor.class, "rightOdo");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // ** Crucial for stopping without coasting (canceling inertia) **
        // Motors will actively brake when power is set to 0.
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        viperSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD); // Adjust if needed
        viperSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Helps hold position
        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setTargetPosition(SLIDE_MIN_POSITION_TICKS);
        viperSlideMotor.setTargetPositionTolerance(SLIDE_TARGET_POSITION_TOLERANCE);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Start in RUN_TO_POSITION

        leftOdometryEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometryEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdometryEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdometryEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.addData("Slide Hold Power", SLIDE_HOLD_POWER);
        telemetry.addData(">", "Press Play to start.");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // --- Read Gamepad Inputs ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * STRAFE_SPEED_MULTIPLIER;
            double rx = gamepad1.right_stick_x;

            double manualSlideInput = gamepad2.left_stick_y * -1; // Up is positive
            boolean slideUpPreset = gamepad2.dpad_up;
            boolean slideDownPreset = gamepad2.dpad_down;
            boolean slideGround = gamepad2.a;
            boolean slideLow = gamepad2.x;
            boolean slideMedium = gamepad2.y;
            boolean slideHigh = gamepad2.b;

            // --- Mecanum Drive Logic ---
            // When y, x, and rx are 0 (joysticks centered), motor powers will be 0.
            // Combined with ZeroPowerBehavior.BRAKE, this makes the robot stop quickly.
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower * DRIVE_SPEED_MULTIPLIER);
            backLeftMotor.setPower(backLeftPower * DRIVE_SPEED_MULTIPLIER);
            frontRightMotor.setPower(frontRightPower * DRIVE_SPEED_MULTIPLIER);
            backRightMotor.setPower(backRightPower * DRIVE_SPEED_MULTIPLIER);

            // --- Viper Slide Logic ---
            int currentSlidePos = viperSlideMotor.getCurrentPosition();
            boolean isManualSlideIntentActive = Math.abs(manualSlideInput) > 0.15; // Joystick deadband

            if (isManualSlideIntentActive) {
                viperSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Switch to manual power control
                double slidePower = manualSlideInput * SLIDE_MANUAL_POWER_MULTIPLIER;

                // Prevent going past limits in manual mode
                if (slidePower > 0 && currentSlidePos >= SLIDE_MAX_POSITION_TICKS) {
                    slidePower = 0; // At top, don't allow further up
                } else if (slidePower < 0 && currentSlidePos <= SLIDE_MIN_POSITION_TICKS) {
                    slidePower = 0; // At bottom, don't allow further down
                }
                viperSlideMotor.setPower(slidePower);
            } else { // No manual intent: use presets or hold current position
                boolean presetButtonPressed = slideUpPreset || slideDownPreset || slideGround || slideLow || slideMedium || slideHigh;

                if (presetButtonPressed) {
                    if (slideUpPreset) { setSlideTarget(currentSlidePos + (int)(50 * SLIDE_TICKS_PER_MM)); }
                    else if (slideDownPreset) { setSlideTarget(currentSlidePos - (int)(50 * SLIDE_TICKS_PER_MM)); }
                    else if (slideGround) { setSlideTarget(SLIDE_GROUND_JUNCTION_TICKS); }
                    else if (slideLow) { setSlideTarget(SLIDE_LOW_JUNCTION_TICKS); }
                    else if (slideMedium) { setSlideTarget(SLIDE_MEDIUM_JUNCTION_TICKS); }
                    else if (slideHigh) { setSlideTarget(SLIDE_HIGH_JUNCTION_TICKS); }
                } else {
                    // No manual intent AND no preset button pressed.
                    // If motor was not in RUN_TO_POSITION (e.g., just came from manual),
                    // set its target to current position to make it hold.
                    if (viperSlideMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                        viperSlideMotor.setTargetPosition(currentSlidePos);
                        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        // Power will be managed below, ensure it starts moving to this new "hold" target
                        viperSlideMotor.setPower(SLIDE_AUTO_POWER);
                    }
                }

                // Common power management for RUN_TO_POSITION mode (when not in manual control)
                if (viperSlideMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                    if (viperSlideMotor.isBusy()) {
                        // If power was turned off by previous logic or just set to hold,
                        // ensure it's on if it's busy moving to a new target.
                        if (Math.abs(viperSlideMotor.getPower()) < SLIDE_AUTO_POWER * 0.5) { // If power is too low to move
                            viperSlideMotor.setPower(SLIDE_AUTO_POWER);
                        }
                    } else {
                        // Not busy: means it has reached its targetPosition (or is within tolerance).
                        // Apply SLIDE_HOLD_POWER to maintain position.
                        viperSlideMotor.setPower(SLIDE_HOLD_POWER);
                    }
                }
            }

            // --- Odometry Reading ---
            prevLeftOdoTicks = currentLeftOdoTicks;
            prevRightOdoTicks = currentRightOdoTicks;
            prevPerpendicularOdoTicks = currentPerpendicularOdoTicks;

            currentLeftOdoTicks = leftOdometryEncoder.getCurrentPosition();
            currentRightOdoTicks = rightOdometryEncoder.getCurrentPosition(); // May need sign flip
            currentPerpendicularOdoTicks = perpendicularOdometryEncoder.getCurrentPosition(); // May need sign flip

            double deltaLeftInches = (currentLeftOdoTicks - prevLeftOdoTicks) * ODOMETRY_INCHES_PER_TICK;
            double deltaRightInches = (currentRightOdoTicks - prevRightOdoTicks) * ODOMETRY_INCHES_PER_TICK;
            double deltaPerpendicularInches = (currentPerpendicularOdoTicks - prevPerpendicularOdoTicks) * ODOMETRY_INCHES_PER_TICK;

            // --- Telemetry ---
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("--- Drive ---", "");
            telemetry.addData("Joystick", "Y: (%.2f), X: (%.2f), RX: (%.2f)", y, x, rx);
            // telemetry.addData("Wheels", "FL: (%.2f), FR: (%.2f), BL: (%.2f), BR: (%.2f)",
            // frontLeftMotor.getPower(), frontRightMotor.getPower(), backLeftMotor.getPower(), backRightMotor.getPower());

            telemetry.addData("--- Viper Slide ---", "");
            telemetry.addData("Mode", viperSlideMotor.getMode());
            telemetry.addData("Current Ticks", currentSlidePos);
            telemetry.addData("Target Ticks", viperSlideMotor.getTargetPosition());
            telemetry.addData("Power", "%.2f", viperSlideMotor.getPower());
            telemetry.addData("Is Busy", viperSlideMotor.isBusy());
            telemetry.addData("Manual Input", "%.2f", manualSlideInput);


            telemetry.addData("--- Odometry (Raw Deltas) ---", "");
            telemetry.addData("L Delta (in)", "%.3f", deltaLeftInches);
            telemetry.addData("R Delta (in)", "%.3f", deltaRightInches);
            telemetry.addData("P Delta (in)", "%.3f", deltaPerpendicularInches);
            // telemetry.addData("L Enc Ticks", currentLeftOdoTicks);
            // telemetry.addData("R Enc Ticks", currentRightOdoTicks);
            // telemetry.addData("P Enc Ticks", currentPerpendicularOdoTicks);
            telemetry.update();
        }
    }

    /**
     * Helper method to set the viper slide target position, respecting limits,
     * and ensures the motor is in the correct mode with power to move.
     * @param targetTicks The desired target position in encoder ticks.
     */
    private void setSlideTarget(int targetTicks) {
        int newTarget = Math.max(SLIDE_MIN_POSITION_TICKS, Math.min(SLIDE_MAX_POSITION_TICKS, targetTicks));

        viperSlideMotor.setTargetPosition(newTarget);
        if (viperSlideMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        // Ensure power is applied to start moving towards the new target if not already busy with sufficient power
        if (!viperSlideMotor.isBusy() || Math.abs(viperSlideMotor.getPower()) < SLIDE_AUTO_POWER * 0.5) {
            viperSlideMotor.setPower(SLIDE_AUTO_POWER);
        }
        telemetry.addData("Slide New Target Set", newTarget);
    }
}
