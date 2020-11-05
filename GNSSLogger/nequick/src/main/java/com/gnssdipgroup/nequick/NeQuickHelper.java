package com.gnssdipgroup.nequick;

public class NeQuickHelper {
    static {
        System.loadLibrary("nequickNative"); // myjni.dll (Windows) or libmyjni.so (Unixes)
    }

    static public native double nequickComputeNative(double[] solarCoef, int month, double UTC,
                                               double lng, double lat, double height,
                                               double satLng, double satLat, double satHeight);

    static public double neQuickCompute(double[] solarCoef, int month, double UTC,
                                 double lng, double lat, double height,
                                 double satLng, double satLat, double satHeight) throws NeQuickException{
        double tec = nequickComputeNative(solarCoef, month, UTC,
        lng, lat, height,
        satLng, satLat, satHeight);
        if(tec==-1){
            throw new NeQuickException("TEC Error!");
        }
        else return tec;
    };
}
