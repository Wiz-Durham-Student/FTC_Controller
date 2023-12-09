package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ControlledPeriod extends LinearOpMode {
    // Declare OpMode members.
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    static final double claw_min_position = 0.7;
    static final double claw_max_position =  0.95;
    static final double shoulder_min_pos = 0.2;
    static final double shoulder_max_pos = 0.75;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FRM;
    private DcMotor FLM;
    private DcMotor BRM;
    private DcMotor BLM;
    private DcMotor Slide;
    private DcMotor DroneDc;
    private Servo Claw;
    private Servo Wrist;
    private Servo ShoulderLeft;
    private Servo ShoulderRight;
    double  positionClaw = claw_min_position + (claw_max_position - claw_min_position) / 2;
    double  positionWrist = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double positionShoulderLeft = (MAX_POS - MIN_POS) / 2;
    double positionShoulderRight = (MAX_POS - MIN_POS) / 2;

    @Override
    public void runOpMode() {

        FRM  = hardwareMap.get(DcMotor.class, "FRM");
        FLM = hardwareMap.get(DcMotor.class, "FLM");
        BRM  = hardwareMap.get(DcMotor.class, "BRM");
        BLM = hardwareMap.get(DcMotor.class, "BLM");
        ShoulderLeft = hardwareMap.get(Servo.class, "ShoulderLeft");
        ShoulderRight = hardwareMap.get(Servo.class, "ShoulderRight");

        Claw = hardwareMap.get(Servo.class, "ClawServo");
        Slide = hardwareMap.get(DcMotor.class, "SlideMotor");
        Wrist = hardwareMap.get(Servo.class, "WristServo");

        DroneDc = hardwareMap.get(DcMotor.class, "DroneDc");

        FLM.setDirection(DcMotor.Direction.REVERSE);
        FRM.setDirection(DcMotor.Direction.FORWARD);
        BLM.setDirection(DcMotor.Direction.REVERSE);
        BRM.setDirection(DcMotor.Direction.FORWARD);

        DroneDc.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double horizontal = gamepad1.left_stick_x;
            double vertical = -gamepad1.left_stick_y;
            double pivot = -gamepad1.right_stick_x;
            double speed = 1;
            boolean leftStickIsTrue = false;

            if (horizontal != 0 || vertical != 0) leftStickIsTrue = true;
            else if (pivot != 0) leftStickIsTrue = false;

            if (leftStickIsTrue) {
                FRM.setPower((-vertical + (-horizontal)) * speed);
                BRM.setPower((-vertical - (-horizontal)) * speed);
                FLM.setPower((-vertical - horizontal) * speed);
                BLM.setPower((-vertical + horizontal) * speed);
            }
            else {
                FRM.setPower(-pivot);
                BRM.setPower(-pivot);
                FLM.setPower(pivot);
                BLM.setPower(pivot);
            }

            if (gamepad2.a && positionClaw < claw_max_position) {
                positionClaw += INCREMENT;
            }
            else if (gamepad2.b && positionClaw > claw_min_position) {
                positionClaw -= INCREMENT;
            }
            if (gamepad2.right_trigger > 0.1 && positionWrist < MAX_POS) {
                positionWrist += INCREMENT;
            }
            else if (gamepad2.left_trigger > 0.1 && positionWrist > MIN_POS) {
                positionWrist -= INCREMENT;
            }
            if (gamepad2.right_bumper) {
                Slide.setPower(0.75);
            }
            else if (gamepad2.left_bumper) {
                Slide.setPower(-0.75);
            }
            else if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
                Slide.setPower(0);
            }
            if (gamepad2.dpad_up && positionShoulderRight  > shoulder_min_pos){
                positionShoulderLeft += INCREMENT*2;
                positionShoulderRight -= INCREMENT*2;
            }
            else if (gamepad2.dpad_down && positionShoulderRight < shoulder_max_pos) {
                positionShoulderLeft -= INCREMENT*2;
                positionShoulderRight += INCREMENT*2;
            }
            if (gamepad2.y) {
                DroneDc.setPower(1);
            }
            else {
                DroneDc.setPower(0);
            }
            Claw.setPosition(positionClaw);
            Wrist.setPosition(positionWrist);
            ShoulderRight.setPosition(positionShoulderRight);
            ShoulderLeft.setPosition(positionShoulderLeft);
            sleep(CYCLE_MS);
            idle();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Shoulder", "current pos (%.2f)", ShoulderRight.getPosition());
            telemetry.addData("Claw", "Current Claw Pos (%.2f)", Claw.getPosition());
            telemetry.addData("X", "x (%.2f)", gamepad1.left_stick_x);
            telemetry.addData("Y", "y (%.2f)", gamepad1.left_stick_y);
            telemetry.update();

        }
    }
}