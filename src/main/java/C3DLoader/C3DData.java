package C3DLoader;


import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.csv.CSVPrinter;
import org.apache.commons.cli.ParseException;
//import org.bytedeco.opencv.opencv_core.Point;
import org.j3d.loaders.c3d.C3DHeader;
import org.j3d.loaders.c3d.C3DParser;
import org.j3d.loaders.c3d.C3DTrajectoryData;
//import org.nd4j.linalg.api.ops.impl.layers.convolution.SConv2DDerivative;
import org.opencv.core.Point3;
import P.Hasiec.opencv.DronBBoxComputer;
/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */

/**
 *
 * @author Piotrek
 */
public class C3DData {
    
    //transpozyycja macierzy K
    private float[][][] coordinates;
    private static Double [][] quats;
    
    private String[] labels;
    private float sampleRate;
    private static DronBBoxComputer BBox3d;

    public double[][][] getMarkersAsDouble() {
        double[][][] out = new double[coordinates.length][][];
        for (int i = 0; i < coordinates.length; i++) {
            out[i] = new double[coordinates[i].length][];
            for (int j = 0; j < coordinates[i].length; j++) {
                out[i][j] = new double[coordinates[i][j].length];
                for (int k = 0; k < coordinates[i][j].length; k++) {
                    out[i][j][k] = coordinates[i][j][k];
                }
            }
        }
        return out;
    }
    public void loadData(File file) throws FileNotFoundException, IOException {
        loadData(new FileInputStream(file));
    }
    
    public void loadData(InputStream iS) throws IOException {
        C3DParser parser = new C3DParser(iS);
        parser.parse(true);
        C3DHeader header = parser.getHeader();
        labels = new String[header.numTrajectories];
        coordinates = new float[header.numTrajectories][header.numTrajectorySamples][];
        sampleRate = header.trajectorySampleRate;
        int i = 0;
        for (C3DTrajectoryData data : parser.getTrajectories()) {
            labels[i] = data.label;
            for (int j = 0; j < getCoordinates()[i].length; j++) {
                coordinates[i][j] = new float[]{data.coordinates[j * 3], data.coordinates[j * 3 + 1], data.coordinates[j * 3 + 2]};
            }
            //     System.out.println(data.label+" "+getCoordinates()[i].length+" "+header.numTrajectorySamples);
            i++;
        }

    }

    public float[][] getCoordinates(String label) {
        return getCoordinates()[getLabelId(label)];
    }

    public int getLabelId(String label) {
        for (int i = 0; i < getLabels().length; i++) {
            if (getLabels()[i].equals(label)) {
                return i;
            }
        }
        return -1;
    }

    public float[] getCoordinates(int labelsId, int frame) {
        return getCoordinates()[labelsId][frame];
    }

    public float[] getCoordinates(String labels, int frame) {
        return getCoordinates(getLabelId(labels), frame);
    }
    
    public int getNumberOFrames() {
        return getCoordinates()[0].length;
    }

    public static void main(String[] args) throws Exception {

        Options options = new Options();
        Option inputPath = new Option("i","inputpath",true,"Input File");
        Option outputPath = new Option("o","outputpath",true,"Output File");
        inputPath.setRequired(true);
        outputPath.setRequired(false);
        options.addOption(outputPath);
        options.addOption(inputPath);
        CommandLineParser parser = new DefaultParser();
        CommandLine cmd;
        HelpFormatter formatter = new HelpFormatter();
        try {
            cmd = parser.parse(options, args);
        } catch (ParseException e) {
            System.out.println(e.getMessage());
            formatter.printHelp("User Profile Info", options);
            System.exit(1);
            return;
        }
        System.out.println(cmd.getOptionValue("inputpath"));
        C3DData data=new C3DData();
        data.loadData(new FileInputStream(cmd.getOptionValue("inputpath")));
        System.out.println(data.coordinates.length+" "+data.getSampleRate());
        System.out.println(Arrays.toString(data.coordinates[1]    [0]));
           computeQuats(cmd.getOptionValue("inputpath"));
         exportToCSV(cmd.getOptionValue("inputpath"));
    }
    
    

    public static void computeQuats(String file) throws Exception {
        C3DData data = new C3DData();
        Point3[] drone1 = new Point3[]{new Point3(80,0,0),new Point3(0,160,0),new Point3(-40,0,0),new Point3(0,-20,0.0)};
        BBox3d = new DronBBoxComputer(drone1,true);
        data.loadData(new FileInputStream(file));
        float[][][] markers = data.getCoordinates();
        quats = new Double[markers[0].length][4];
        for (int j = 1; j < markers[0].length; j++) {
//            System.out.println("\n\nFrame " + j);
            Point3[] fP = new Point3[markers.length];

            for (int i = 0; i < markers.length; i++) {
                //System.out.println(data.labels[i] + " " + Arrays.toString(markers[i][1]));
                fP[i] = new Point3(markers[i][j][0], markers[i][j][1], markers[i][j][2]);
            }
            
            Point3[] cross= BBox3d.cross3d(fP);
            BBox3d.compute_versor(cross);
            var q = (BBox3d.rotation_quaternion().toArray());
            for(int z =0;z<4;z++)
            {
                quats[j][z] = (Double)q[z];
            }
            
//            for (int k = 0; k < cross.length; k++) {
//                System.out.println(cross[k]);
//            }
//            System.out.println(q1);
//                for(int k=0;k<markers.length;k++) {
//                    double d=0;
//                    for(int l=0;l<markers[i][j].length;l++)
//                        d+=Math.pow(markers[i][j][l]-markers[k][j][l], 2);
//                    System.out.print(Math.sqrt(d)+" ; ");
//                }
//                System.out.println();

        }
//        System.out.println(data.coordinates.length+" "+data.getSampleRate());
//        System.out.println(Arrays.toString(data.coordinates[1]    [0]));
    }

    public double[][][] getCoordinatesSynchronized(double newSampleRate) {
        return getCoordinatesSynchronized(newSampleRate, 0);
    }

    public void saveCoordinatesToCSV(File csvFile) throws IOException {
        CSVPrinter csvPrinter=new CSVPrinter(new FileWriter(csvFile), CSVFormat.ORACLE);
        List<String> cols=new java.util.LinkedList<>();
        for(String label:getLabels()) {
            cols.add(label.trim()+"_X");
            cols.add(label.trim()+"_Y");
            cols.add(label.trim()+"_Z");
            

            
//            cols.add(label.trim()+"_bb1");
//            cols.add(label.trim()+"_bb2");
//            cols.add(label.trim()+"_bb3");
//            cols.add(label.trim()+"_bb4");
//            cols.add(label.trim()+"_bb5");
//            cols.add(label.trim()+"_bb6");
//            cols.add(label.trim()+"_bb7");
//            cols.add(label.trim()+"_bb8");
            
        }
            cols.add("qw");
            cols.add("qx");
            cols.add("qy");
            cols.add("qz");
        csvPrinter.printRecord(cols.toArray());
        
        for(int frame=1;frame<getNumberOFrames();frame++) {
            List<String> values=new LinkedList<>();
            for(String label:getLabels()) {
                float[] coords=getCoordinates(label, frame);
                for(int i=0;i<coords.length;i++) {
                    values.add(Float.toString(coords[i]));
                }                    
            }
            for(int q =0;q<4;q++){
               
            values.add(Double.toString(quats[frame][q]));
            }
            csvPrinter.printRecord(values.toArray());
        
        }
        
        csvPrinter.close();
    }
    
    public static void exportToCSV(String file) throws IOException {
        File fileC3D=new File(file);
        File fileCSV=new File(fileC3D.getParentFile(),fileC3D.getName().substring(0,fileC3D.getName().length()-4)+".csv");
        C3DData c3d=new C3DData();
        c3d.loadData(new FileInputStream(fileC3D));
        c3d.saveCoordinatesToCSV(fileCSV);
    }
    
    public String printCoordinates() {
        StringBuilder sB=new StringBuilder();
        sB.append(coordinates.length+" "+coordinates[0].length+" "+coordinates[0][0].length+"\n");
        for(int i=0;i<coordinates.length;i++) {
            for(int j=0;j<coordinates[i].length;j++) {
                for(int k=0;k<coordinates[i][j].length;k++)
                    sB.append(coordinates[i][j][k]+" ");
                sB.append(" ; ");
            }
                sB.append("\n*******************\n");
            }
        return sB.toString();
    }
    
    
    public double[][][] getCoordinatesSynchronized(double newSampleRate, int shitf) {
        double[][][] out = getMarkersAsDouble();
        int newLenght = (int) (out[0].length * newSampleRate / sampleRate);
        System.out.println("newLenght:" + newLenght + " OldLenght:" + coordinates[0].length + " newSampleRate:" + newSampleRate + " oldSamplerate:" + sampleRate);
        double[][][] out2 = new double[out.length][][];
        for (int i = 0; i < out.length; i++) {
            out2[i] = new double[newLenght][];
            for (int j = 0; j < out2[i].length; j++) {
                int jj = (int) Math.round(j * sampleRate / newSampleRate);
                jj += shitf;
                if (jj < 0) {
                    jj = 0;
                }
                out2[i][j] = new double[out[i][jj].length];
                for (int k = 0; k < out[i][jj].length; k++) {
                    out2[i][j][k] = out[i][jj][k];
                }
            }
        }
        return out2;
    }

    /**
     * @return the coordinates
     */
    public float[][][] getCoordinates() {
        return coordinates;
    }

    /**
     * @return the labels
     */
    public String[] getLabels() {
        return labels;
    }

    /**
     * @return the sampleRate
     */
    public float getSampleRate() {
        return sampleRate;
    }
    
   
    
}
