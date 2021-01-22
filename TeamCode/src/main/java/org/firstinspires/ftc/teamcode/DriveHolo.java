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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



@TeleOp(name="POV EMU HOLO Driving OpMode", group="Linear Opmode")
//@Disabled
public class DriveHolo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorImplEx r1 = null;
    private DcMotorImplEx r2 = null;
    private DcMotorImplEx l1 = null;
    private DcMotorImplEx l2 = null;

    private final double MAX_RPM = 6000; //free speed rpm
    private final double GEAR_RATIO = 20; // 20:1 reduction
    private final double WHEEL_RAD = 0.0375; //meters

    private double MAX_RAD_PER_SEC = MAX_RPM / 60 / GEAR_RATIO * 2 * Math.PI;
    private double MAX_VEL_LIN  = MAX_RAD_PER_SEC * (WHEEL_RAD*WHEEL_RAD) ; //meters/sec  V = w * r^2 get linear vel by taking angular vel (rpm in rads) * r^2

    public void initDriveOp(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        r1  = (DcMotorImplEx) hardwareMap.get(DcMotor.class, "right_drive1");
        r2 = (DcMotorImplEx) hardwareMap.get(DcMotor.class, "right_drive2");
        l1  = (DcMotorImplEx) hardwareMap.get(DcMotor.class, "left_drive1");
        l2 = (DcMotorImplEx) hardwareMap.get(DcMotor.class, "left_drive2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        r1.setDirection(DcMotor.Direction.FORWARD);
        r2.setDirection(DcMotor.Direction.FORWARD);
        l1.setDirection(DcMotor.Direction.REVERSE);
        l2.setDirection(DcMotor.Direction.REVERSE);

        runtime.reset();
    }

    public void executeDriveLogic(){
        double triggerSpeed = gamepad1.right_trigger * MAX_RAD_PER_SEC; // master speed on trigger
        if (gamepad1.b) triggerSpeed = 0; // press B to stop

        // POV Mode uses left stick to go forward, and right stick to turn.
        double turn = gamepad1.right_stick_x;
        double drive = gamepad1.left_stick_y;


        double leftAngVel = Range.clip(triggerSpeed * (drive + turn), -MAX_RAD_PER_SEC, MAX_RAD_PER_SEC);
        double rightAngVel = Range.clip(triggerSpeed * (drive - turn), -MAX_RAD_PER_SEC, MAX_RAD_PER_SEC);

        // Send calculated power to wheels
        l1.setVelocity(leftAngVel, AngleUnit.RADIANS);
        l2.setVelocity(leftAngVel, AngleUnit.RADIANS);
        r1.setVelocity(rightAngVel, AngleUnit.RADIANS);
        r2.setVelocity(rightAngVel, AngleUnit.RADIANS);


        telemetry.addData("Instructions", "Left stick y for direction, right stick x for turning, right trigger for master scaling");
        telemetry.addData("Status", "Run Time: " + runtime.toString() + " MotorController: " + r1.getController().toString());
        // Shift speeds to linear speed when reporting
        telemetry.addData("Motors", "Speeds: left (%.2f)m/s, right (%.2f)m/s", 2 * leftAngVel * (WHEEL_RAD * WHEEL_RAD), 2 * rightAngVel * (WHEEL_RAD * WHEEL_RAD));
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        initDriveOp();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            executeDriveLogic();
        }
    }
}
