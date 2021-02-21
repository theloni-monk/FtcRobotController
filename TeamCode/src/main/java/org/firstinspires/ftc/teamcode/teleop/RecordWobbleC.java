package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.botfunctionality.RecorderBot;
@TeleOp(name="Record Wobble C", group="Recording")
public class RecordWobbleC extends RecorderBot {
    final String fileName = "C";
    boolean isRecording = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initRecorderBot();
        waitForStart();
        while(opModeIsActive()){
            executeControllerDriveLogic();

            if(gamepad1.x && debounced){
                isRecording = true;
                initMap();
                debounce.run();
            }
            if(gamepad1.y && debounced){
                isRecording = false;
                saveFile();
                debounce.run();
            }

            if(isRecording) executeRecorderLogic();
        }
    }
}
