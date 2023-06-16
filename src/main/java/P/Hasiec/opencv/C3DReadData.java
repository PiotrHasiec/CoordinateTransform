/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package P.Hasiec.opencv;

import com.opencsv.CSVWriter;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.ParseException;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.opencv.opencv_java;
import org.j3d.loaders.c3d.C3DHeader;
import org.j3d.loaders.c3d.C3DParser;
import org.j3d.loaders.c3d.C3DTrajectoryData;
import org.opencv.core.Point3;
import java.util.Dictionary;
import java.util.Hashtable;

/**
 *
 * @author Piotrek
 */
public class C3DReadData {
    String[] labels;
   public static void main(String[] args) 
   {
            Loader.load(opencv_java.class);

            Options options = new Options();
            Option inputPath = new Option("i","inputpath",true,"Input File");
            Option outputPath = new Option("o","outputpath",true,"Output File");
            inputPath.setRequired(true);
            outputPath.setRequired(true);
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
            var inputFile = cmd.getOptionValue("inputpath");
            var outputFile = cmd.getOptionValue("outputpath");
            C3DReadData loader = new C3DReadData();
            var data = loader.loadData(inputFile); 
            var labels = new String[3*loader.labels.length];
            for(int i = 0; i<loader.labels.length;i++)
            {
                labels[3*i]     = loader.labels[i] + "_X";
                labels[3*i+1]   = loader.labels[i] + "_Y";
                labels[3*i+2]   = loader.labels[i] + "_Z";
            }
//            Dictionary <String,Integer> indexLabels = new Hashtable<>();
//            for( int i = 0; i<labels.length;i++)
//            {
//                index
//            }
//            ArrayList<Point3>[] drones = new ArrayList<Point3>[labels.length];
            try ( CSVWriter csvWriter = new CSVWriter(new FileWriter(outputFile));) {
                csvWriter.writeNext(labels);
                for( int frame = 0; frame< data[0].length;frame++)
                {
                    ArrayList<String> framestring = new ArrayList<>();
                    for(int marker = 0; marker <data.length;marker++)
                    {
                        try{
                        for(var coordinate:data[marker][frame])
                        {
                            framestring.add(Float.toString(coordinate));
                        }
                        }
                        catch(Exception ex){
                            System.out.println(String.valueOf(frame) + String.valueOf(marker));
                            continue;
                        }
                    }
                    csvWriter.writeNext(framestring.toArray(new String[0]));
                }
            
            
            } catch (IOException ex) {
            Logger.getLogger(C3DReadData.class.getName()).log(Level.SEVERE, null, ex);
        }
            
            
        
   }
   public  float[][][] loadData(String filename)
   {
             
                  try {
            C3DParser parserC3D = new C3DParser(new FileInputStream(filename));
            parserC3D.parse(true);
            C3DHeader header = parserC3D.getHeader();
            labels = new String[header.numTrajectories];
            var coordinates = new float[header.numTrajectories][header.numTrajectorySamples][];
            var sampleRate = header.trajectorySampleRate;
            int i = 0;
            for (C3DTrajectoryData data : parserC3D.getTrajectories()) {
                labels[i] = data.label;
                for (int j = 0; j < data.coordinates.length/3; j++) {
                    coordinates[i][j] = new float[]{data.coordinates[j * 3], data.coordinates[j * 3 + 1], data.coordinates[j * 3 + 2]};
                }
//                     System.out.println(data.label+" "+getCoordinates()[i].length+" "+header.numTrajectorySamples);
                i++;
        
            }
            return coordinates;
            }catch (Exception ex) {
            Logger.getLogger(C3DReadData.class.getName()).log(Level.SEVERE, null, ex);
           
            }
                 return null; 
   }
}
