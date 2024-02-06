// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.utils;

import java.util.Arrays;
import java.util.Comparator;

/** Add your docs here. */
public class shootFromAnyPlace {
    private double[][] lookupTable = {
        {0,0,0},{0,0,0},{0,0,0}
    };

    public shootFromAnyPlace(){}

    private void sort(double[][] arr){
        Arrays.sort(arr, new Comparator<double[]>() {
            @Override
            public int compare(double[] entry1, double[] entry2){
                return Double.compare(entry1[0], entry2[0]);
            }
        });
    }

    private double[][] addIndex(double[][] arr, double[] x){
        double[][] newArr = new double[arr.length + 1][];
        for (int i=0; i < arr.length; i++){
            newArr[i] = arr[i];
        }
        newArr[arr.length] = x;
        return newArr;
    }

    public double getAngle(double dis){
        for (double[] i : lookupTable) {
            if  (i[0] == dis){
                return i[1];
            }
        }
        
        double[] top = {0,0};
        double[] bottom = {0,0};
        double m;
        double b;
        double[] x = {dis, 0, 0};
        double[][] copy = addIndex(lookupTable, x);
        sort(copy);

        for (int i=0; i < copy.length; i++){
            if (copy[i][0] == dis){
                bottom = copy[i==0 ? 0 : i-1];
                top = copy[i==copy.length-1 ? -1 : i+1];
            }
        }

        m = (bottom[1] - top[1]) / (bottom[0] - top[0]);

        b = -1*(m*bottom[0] - bottom[1]);

        return m*dis + b;
    }


    public double getPow(double dis){
        for (double[] i : lookupTable) {
            if  (i[0] == dis){
                return i[2];
            }
        }
        
        double[] top = {0,0,0};
        double[] bottom = {0,0,0};
        double m;
        double b;
        double[] x = {dis, 0 , 0};
        double[][] copy = addIndex(lookupTable, x);
        sort(copy);

        for (int i=0; i < copy.length; i++){
            if (copy[i][0] == dis){
                bottom = copy[i==0 ? 0 : i-1];
                top = copy[i==copy.length-1 ? -1 : i+1];
            }
        }

        m = (bottom[2] - top[2]) / (bottom[0] - top[0]);

        b = -1*(m*bottom[0] - bottom[2]);

        return m*dis + b;
    }
}