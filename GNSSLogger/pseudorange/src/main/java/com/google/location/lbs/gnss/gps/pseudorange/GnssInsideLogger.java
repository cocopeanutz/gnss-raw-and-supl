package com.google.location.lbs.gnss.gps.pseudorange;

import android.os.Environment;
import android.util.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class GnssInsideLogger {
    File logFile;
    File baseDirectory;
    boolean haveCreatedFile = false;
    public GnssInsideLogger(String filename){
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state)) {
            baseDirectory = new File(Environment.getExternalStorageDirectory(), "gnssInsideFolder");
            baseDirectory.mkdirs();
        } else if (Environment.MEDIA_MOUNTED_READ_ONLY.equals(state)) {
            Log.e("smallTag", "Cannot write to external storage.");
            return;
        } else {
            Log.e("smallTag", "Cannot read external storage.");
            return;
        }
        logFile = new File(baseDirectory, filename);
    }
    public void appendLog(String text) {

        if(!haveCreatedFile){
            if(logFile.exists()){
                logFile.delete();
            }
            try {
                logFile.createNewFile();
            }catch (IOException e){
                e.printStackTrace();
            }
            haveCreatedFile = true;
        }

        try {
            BufferedWriter buf = new BufferedWriter(new FileWriter(logFile, true));
            buf.append(text);
            buf.newLine();
            buf.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
