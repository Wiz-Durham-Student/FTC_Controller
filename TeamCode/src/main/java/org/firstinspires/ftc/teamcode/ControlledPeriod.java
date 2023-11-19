/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp
public class ControlledPeriod extends LinearOpMode {
    // Declare OpMode members.
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FRM;
    private DcMotor FLM;
    private DcMotor BRM;
    private DcMotor BLM;
    private DcMotor Slide;
    private Servo Claw;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double horizontal = gamepad1.left_stick_x;
    double vertical = -gamepad1.left_stick_y;
    double pivot = -gamepad1.right_stick_x;
    double speed = 1;
    boolean leftStickIsTrue = false;


    @Override
    public void runOpMode() {
        FRM  = hardwareMap.get(DcMotor.class, "FRM");
        FLM = hardwareMap.get(DcMotor.class, "FLM");
        BRM  = hardwareMap.get(DcMotor.class, "BRM");
        BLM = hardwareMap.get(DcMotor.class, "BLM");
        Claw = hardwareMap.get(Servo.class, "ClawServo");
        Slide = hardwareMap.get(DcMotor.class, "SlideMotor");

        FLM.setDirection(DcMotor.Direction.REVERSE);
        FRM.setDirection(DcMotor.Direction.FORWARD);
        BLM.setDirection(DcMotor.Direction.REVERSE);
        BRM.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (horizontal != 0 || vertical != 0) leftStickIsTrue = true;
            else if (pivot != 0) leftStickIsTrue = false;

            if (leftStickIsTrue) {
                FRM.setPower((-vertical + horizontal) * speed);
                BRM.setPower((-vertical - horizontal) * speed);
                FLM.setPower((-vertical - horizontal) * speed);
                BLM.setPower((-vertical + horizontal) * speed);
            }
            else {
                FRM.setPower(-pivot);
                BRM.setPower(-pivot);
                FLM.setPower(pivot);
                BLM.setPower(pivot);
            }

            if (gamepad2.right_trigger > 0.1 && position < MAX_POS) {
                position += INCREMENT;
            }
            else if (gamepad2.left_trigger > 0.1 && position > MIN_POS) {
                position -= INCREMENT;
            }
            if (gamepad2.right_bumper) {
                Slide.setPower(0.2);
            }
            else if (gamepad2.left_bumper) {
                Slide.setPower(-0.2);
            }
            else if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
                Slide.setPower(0);
            }
            Claw.setPosition(position);
            sleep(CYCLE_MS);
            idle();
            //Slide.setPower(gamepad1.right_trigger);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

        }
    }
}
