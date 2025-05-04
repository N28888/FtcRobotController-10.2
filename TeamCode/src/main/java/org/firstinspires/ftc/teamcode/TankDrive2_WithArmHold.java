package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx; // Import DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range; // Import Range for clipping power

@TeleOp(name = "TankDrive2_WithArmHold", group = "TeleOp2") // Renamed slightly
public class TankDrive2_WithArmHold extends OpMode { // Renamed class

    // Drive Motors
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    // Arm Motor (Using DcMotorEx is recommended for PID tuning)
    private DcMotorEx topMotor; // Changed to DcMotorEx

    // Servos
    private Servo frontServo;
    private Servo rotateServo;

    // Arm Control State Variables
    private int armTargetPosition;
    private boolean isHoldingArm = false;

    // --- Tuning Constants --- (ADJUST THESE FOR YOUR ROBOT!)
    private final double MANUAL_ARM_POWER = 0.6;   // Power when D-pad is pressed
    private final double HOLDING_ARM_POWER = 0.5;  // Max power allowed for holding position
    // Lower this if the arm drifts down, raise if it bounces or is jerky
    private final double ARM_HOLD_P_COEFFICIENT = 5.0; // Proportional coefficient for holding (start here, tune!)

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");

        // --- Drive Motors ---
        try {
            leftMotor = hardwareMap.get(DcMotor.class, "left");
            rightMotor = hardwareMap.get(DcMotor.class, "right");
            frontServo = hardwareMap.get(Servo.class,"front");
            rotateServo = hardwareMap.get(Servo.class,"rotate");

            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Good practice for drive motors too
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } catch (Exception e) {
            telemetry.addData("Error", "Drive/Servo Init Failed: " + e.getMessage());
            // Consider stopping initialization if drive fails
        }

        // --- Arm Motor (topMotor) ---
        try {
            topMotor = hardwareMap.get(DcMotorEx.class, "top"); // Get as DcMotorEx

            // Set Arm Motor Direction (Uncomment and change if needed)
            // topMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            // *** Crucial Arm Setup ***
            // Set motor to BRAKE when power is zero (helps hold position)
            topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Reset the encoder count to zero
            topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Initialize the target position to the starting position (which is now 0)
            armTargetPosition = 0;

            // Set the initial mode. We'll switch between RUN_USING_ENCODER and RUN_TO_POSITION
            // Let's start it ready to hold position 0.
            topMotor.setTargetPosition(armTargetPosition);
            topMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            isHoldingArm = true; // Start in holding mode

            // Set PID coefficients for position control (Tune this!)
            // Start with just P, increase slowly until it holds well without oscillation
            topMotor.setPositionPIDFCoefficients(ARM_HOLD_P_COEFFICIENT); // Set P-gain for RUN_TO_POSITION
            // You might need to tune other coefficients (I, D, F) for better performance:
            // topMotor.setVelocityPIDFCoefficients(p, i, d, f); // Use if RUN_USING_ENCODER needs tuning
            // More advanced: new PIDFCoefficients(p, i, d, f, algorithm)

            // Apply initial power for holding. It will hold at position 0.
            topMotor.setPower(HOLDING_ARM_POWER);


        } catch (Exception e) {
            telemetry.addData("Error", "topMotor Init Failed: " + e.getMessage());
            topMotor = null; // Prevent errors in loop if init fails
        }


        telemetry.addData("Status", "Initialized");
        telemetry.update(); // Show status update on Driver Station
    }

    @Override
    public void loop() {

        // --- Drive Motors ---
        // Arcade Drive (using only left stick)
        double drive = gamepad1.left_stick_y; // Forward/Backward
        double turn = -gamepad1.left_stick_x;  // Turn Left/Right

        // Combine drive and turn for arcade drive.
        double leftPower = drive - turn;
        double rightPower = drive + turn;

        // Normalize the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) { // Fix: Should be 1.0, not 10
            leftPower /= max;
            rightPower /= max;
        }

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);


        // --- Top Motor (Arm) Control ---
        if (topMotor != null) { // Check if topMotor initialized correctly
            boolean dpadUpPressed = gamepad1.dpad_up;
            boolean dpadDownPressed = gamepad1.dpad_down;

            if (dpadUpPressed || dpadDownPressed) {
                // --- Manual Control ---
                isHoldingArm = false; // Stop holding specific target

                // Set mode for direct power control (using encoders helps maintain speed)
                topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // Set power based on D-pad input
                if (dpadUpPressed) {
                    topMotor.setPower(-MANUAL_ARM_POWER);
                } else { // dpadDownPressed must be true
                    topMotor.setPower(MANUAL_ARM_POWER);
                }
            } else {
                // --- Hold Position ---
                if (!isHoldingArm) {
                    // Transitioning from Manual to Hold: Capture current position as the new target
                    armTargetPosition = topMotor.getCurrentPosition();
                    topMotor.setTargetPosition(armTargetPosition);
                    topMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    topMotor.setPower(HOLDING_ARM_POWER); // Apply power to move towards/hold the target
                    isHoldingArm = true;
                } else {
                    // Already in holding mode. Ensure motor is still set correctly.
                    // Re-setting mode/target/power on every loop iteration while holding
                    // is generally okay for RUN_TO_POSITION and ensures it stays active.
                    if (topMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                        // If mode somehow got changed, reset it
                        topMotor.setTargetPosition(armTargetPosition);
                        topMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    // Always ensure power is applied when holding
                    topMotor.setPower(HOLDING_ARM_POWER);
                }
            }
        } // end if (topMotor != null)


        // --- Wheel Servo (frontServo) ---
        if (frontServo != null) {
            if (gamepad1.left_bumper) {
                frontServo.setPosition(0); // Position 0
            } else if (gamepad1.right_bumper) {
                frontServo.setPosition(1); // Position 1
            }
            // Note: Servo will stay at the last commanded position if neither bumper is pressed.
        }

        // --- Rotate Servo (rotateServo) ---
        if (rotateServo != null) {
            if (gamepad1.b) {
                rotateServo.setPosition(0.81);
            } else if (gamepad1.y) {
                rotateServo.setPosition(0.5); // Center position (maybe?)
            } else if (gamepad1.x) {
                rotateServo.setPosition(0.17);
            }
            // Note: Servo will stay at the last commanded position if X/Y/B are not pressed.
        }


        // --- Telemetry ---
        telemetry.addData("--- Drive ---", "");
        telemetry.addData("Left Power", "%.2f", leftMotor.getPower());
        telemetry.addData("Right Power", "%.2f", rightMotor.getPower());
        telemetry.addData("Left Stick Y", "%.2f", gamepad1.left_stick_y);
        telemetry.addData("Left Stick X", "%.2f", gamepad1.left_stick_x);

        telemetry.addData("--- Arm (topMotor) ---", "");
        if (topMotor != null) {
            telemetry.addData("Mode", topMotor.getMode());
            telemetry.addData("Holding Arm", isHoldingArm);
            telemetry.addData("Target Pos", armTargetPosition);
            telemetry.addData("Current Pos", topMotor.getCurrentPosition());
            telemetry.addData("Power", "%.2f", topMotor.getPower());
            telemetry.addData("Is Busy", topMotor.isBusy()); // Useful for RUN_TO_POSITION
        } else {
            telemetry.addData("topMotor", "Not Initialized");
        }

        telemetry.addData("--- Servos ---", "");
        if (frontServo != null) {
            telemetry.addData("Front Servo Pos", "%.2f", frontServo.getPosition());
        }
        if (rotateServo != null) {
            telemetry.addData("Rotate Servo Pos", "%.2f", rotateServo.getPosition());
        }
        telemetry.addData("--- Buttons ---", "");
        telemetry.addData("Dpad Up", gamepad1.dpad_up);
        telemetry.addData("Dpad Down", gamepad1.dpad_down);
        telemetry.addData("Bumpers", "L:%b R:%b", gamepad1.left_bumper, gamepad1.right_bumper);
        telemetry.addData("X, Y, B", "X:%b Y:%b B:%b", gamepad1.x, gamepad1.y, gamepad1.b);

        telemetry.update(); // Send telemetry data to the Driver Station
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Optional: Explicitly stop motors, although the system usually handles this.
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        if (topMotor != null) {
            topMotor.setPower(0);
            // Optionally set back to a non-position mode
            topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}