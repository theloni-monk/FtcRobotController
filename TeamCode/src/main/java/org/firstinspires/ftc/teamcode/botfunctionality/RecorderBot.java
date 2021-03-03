package org.firstinspires.ftc.teamcode.botfunctionality;

import android.util.Size;

import com.google.gson.Gson;
import com.qualcomm.robotcore.robot.Robot;
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
            RobotLog.vv("Currently recording: ", currTime+": "+this.leftAngVel+","+ this.rightAngVel+"; TreeMap keys len: " +  outputMap.keySet().size());
            //RobotLog.vv("Free memory remaining", String.valueOf(Runtime.getRuntime().freeMemory()));
            outputMap.put(currTime, this.leftAngVel+","+ this.rightAngVel);
            this.prevVelL = this.leftAngVel;
            this.prevVelR = this.rightAngVel;
            //RobotLog.vv("growing map: ", outputMap.toString());
        }
    }

    protected void saveFile(){
            outputMap.put(System.currentTimeMillis() - initTime, "0,0");
            RobotLog.vv("Map",outputMap.toString());
            System.out.flush();
            String jstr = gson.toJson(outputMap);
            RobotLog.vv("truncated map", jstr);
            logMapJson(jstr);
            System.out.flush();

    }

    void logMapJson(String json){
        for(String s: json.split(",\"")){
            RobotLog.vv("jsondata", ",\""+s);
        }
    }

}
