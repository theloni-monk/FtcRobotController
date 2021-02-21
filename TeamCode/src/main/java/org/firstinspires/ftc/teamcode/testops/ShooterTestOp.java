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

package org.firstinspires.ftc.teamcode.testops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.TimerTask;


@TeleOp(name="Shooter Test Script - kitbot config", group="Tests")
@Disabled
public class ShooterTestOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private Boolean debounced = new Boolean(true);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  =  hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        boolean spinMotors = false;

        double powerLevel = 0.5;

        TimerTask debounce = new TimerTask() {
            @Override
            public void run() {
                synchronized (debounced){
                    debounced = false;
                    sleep(500);
                    debounced = true;
                }
            }
        };


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.a) spinMotors = true;
            if(gamepad1.b) spinMotors = false; // press B to stop
            if(gamepad1.y && debounced) {powerLevel += 0.1; debounce.run();}
            if(gamepad1.x && debounced) {powerLevel -= 0.1; debounce.run();}

            powerLevel    = Range.clip(powerLevel, -1.0, 1.0) ;

            // Send calculated power to wheels
            if(spinMotors) {
                leftDrive.setPower(powerLevel);
                rightDrive.setPower(-powerLevel);
            }
            else{
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Instructions", "Press A to enable motors and B to disable, press X to increase power level by 0.1 and Y to decrease by 0.1 " );
            telemetry.addData("Status", "Run Time: " + runtime.toString() + " MotorController: " + leftDrive.getController().toString());
            telemetry.addData("Motors", "OutputPower(-1 to 1): " + powerLevel  + " Disabled: " + spinMotors + " Bouncing: " + !debounced);
            telemetry.update();
        }
    }
}
