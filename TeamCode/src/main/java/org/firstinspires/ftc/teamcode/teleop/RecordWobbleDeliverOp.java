package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.botfunctionality.RecorderBot;
@TeleOp (name="Record Wobble Deliveries", group="Recording")
public class RecordWobbleDeliverOp extends RecorderBot {
    boolean isRecording = false;

    boolean recordingA,recordingB,recordingC = false;
    @Override
    public void runOpMode() throws InterruptedException {
        initRecorderBot();
        waitForStart();
        while(opModeIsActive()){
            executeControllerDriveLogic();

            if(gamepad1.a && debounced && !(recordingB || recordingC)){
                isRecording = !isRecording;
                recordingA = isRecording;
                if(isRecording) {initMap(); RobotLog.vv("RECORDING A START","\n\nBEGINS NOW:\n\n");}
                else saveFile();
                debounce.run();
            }
            if(gamepad1.b && debounced && !(recordingA || recordingC)){
                isRecording = !isRecording;
                recordingB = isRecording;
                if(isRecording) {initMap(); RobotLog.vv("RECORDING B START","\n\nBEGINS NOW:\n\n");}
                else saveFile();
                debounce.run();
            }
            if(gamepad1.x && debounced && !(recordingA || recordingB)){
                isRecording = !isRecording;
                recordingC = isRecording;
                if(isRecording) {initMap(); RobotLog.vv("RECORDING C START","\n\nBEGINS NOW:\n\n");}
                else saveFile();
                debounce.run();
            }
            telemetry.addData("instructions","Press a to start/stop recording A, press b to start stop recording B, press x to start/stop recording C");
            telemetry.addData("ABC", "CURRENTLY RECORDING A: %s, B: %s, C: %s".format(String.valueOf(recordingA),String.valueOf(recordingB),String.valueOf(recordingC)));
            telemetry.update();

            if(isRecording) executeRecorderLogic();
        }
    }
}
