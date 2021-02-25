package org.firstinspires.ftc.teamcode.botfunctionality;

import com.google.gson.Gson;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.ObjectOutputStream;
import java.util.TreeMap;

public abstract class RecorderBot extends HolonomicDriveBot{
    ObjectOutputStream objOutStream;
    TreeMap<Long, String> outputMap;
    Gson gson;

    double prevVelL;
    double prevVelR;

    protected long initTime = 0;

    public RecorderBot(){

    }

    public void initMap(){
        outputMap = new TreeMap<>();
        gson = new Gson();

    }

    public void initRecorderBot(){
        initDriveOp();
        initMap();
    }

    protected void executeRecorderLogic(){
        if(initTime == 0) { initTime = System.currentTimeMillis();}
        if(prevVelL != this.leftAngVel || prevVelR != this.rightAngVel){
            long currTime = System.currentTimeMillis() - initTime;
            RobotLog.vv("recording: ", currTime+": "+this.leftAngVel+","+ this.rightAngVel);
            outputMap.put(currTime, this.leftAngVel+","+ this.rightAngVel);
            RobotLog.vv("growing map: ", outputMap.toString());
        }
    }

    protected void saveFile(){
            outputMap.put(System.currentTimeMillis() - initTime, "0,0");
            RobotLog.vv("Map",outputMap.toString());
            RobotLog.vv("s", "\n\n\n");
            RobotLog.vv("Wobble", gson.toJson(outputMap));
            RobotLog.vv("s","\n\n\n");

    }

}
