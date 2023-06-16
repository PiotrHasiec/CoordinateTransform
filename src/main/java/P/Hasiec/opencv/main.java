/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package P.Hasiec.opencv;

import com.opencsv.CSVReader;
import com.opencsv.CSVWriter;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.opencv.opencv_java;
import org.opencv.core.Point3;

/**
 *
 * @author Piotrek
 */
public class main {

    public static void main(String[] args) {
        try {
            Loader.load(opencv_java.class);
            Point3[] drone1 = new Point3[]{new Point3(0.,90,0.0),new Point3(0.0,-60,0.0),new Point3(-70,0,0.0),new Point3(80,0.0,0),new Point3(0.0,0.0,35)};
            Point3[] drone2 = new Point3[]{new Point3(0,50,0.0),new Point3(0.0,-70,0.0),new Point3(90,0,0.0),new Point3(-60,0.0,0),new Point3(0.0,0.0,5)};
            ArrayList<Point3[]> drones = new ArrayList<>();
            drones.add(drone1);
            drones.add(drone2);
            Point3[] bbox = new Point3[]{new Point3(250,250,250/2),new Point3(250,-250,250/2),new Point3(-250,250,250/2),new Point3(-250,-250,250/2),new Point3(250,250,-250/2),new Point3(250,-250,-250/2),new Point3(-250,250,-250/2),new Point3(-250,-250,-250/2)};
            var camerastab = new String[]{"55260362","56339527","60881257","61016180"};
            ArrayList<String> cameras = new ArrayList<>(Arrays.asList(camerastab));
            
         
            //var c = Camera.loadCamera("C:\\Users\\Piotrek\\Documents\\NetBeansProjects\\OpenCV\\cameras.xml",cameras);
            ArrayList< DronBBoxComputer> comp = new  ArrayList<>();

            DronBBoxComputer c1 = new DronBBoxComputer(drone1,true,new Point3(0,60,-100));
            DronBBoxComputer c2 = new DronBBoxComputer(drone2,true,new Point3(0,50,-100));
            comp.add(c1);
            comp.add(c2);
            
//            Point3 [] tab = new Point3[] {new Point3(196.18166, 443.6729, 214.52107),new Point3(325.879, 568.2177, 216.47696),new Point3(238.23164, 428.5011, 214.68085),new Point3(155.42084, 515.49713, 213.95518)};
//            Point3 [] tab = new Point3[] {new Point3(712.4803,-31.73764,1453.3044),new Point3(532.9377,-26.948772,1452.8812),new Point3(693.87976,8.895761,1453.4669),new Point3(689.9685,-111.25641,1452.0688)};
            List<List<String>> records = new ArrayList<List<String>>();
            List<String> headers = new ArrayList<>();
            Integer[] index = new Integer[]{0,3,6,9};
//            Point3[] drone1 = new Point3[]{new Point3(80.52,0,0),new Point3(0,159.738,0),new Point3(-39.62,0,0),new Point3(0,-18.60,0.0)};

            
            List<Point3> globalDrone2 = new ArrayList<>();
            int offset = 2;
            List<Point3> four_points_list = new ArrayList<>();
            List<Double> vals = new ArrayList<>();
            try ( CSVWriter csvWriter = new CSVWriter(new FileWriter("C:\\Users\\Piotrek\\Documents\\NetBeansProjects\\OpenCV\\Dron1Dron2LotdowolnyBBox.csv"));) {
                try ( CSVReader csvReader = new CSVReader(new FileReader("C:\\Users\\Piotrek\\Documents\\NetBeansProjects\\OpenCV\\Dron1Dron2LotdowolnyTest.csv"));) {
                     for(int d =0;d<drones.size();d++)
                     {
                         for(int value =0;value<drones.get(d).length;value +=1)
                            {
                                headers.add("Drone"+Integer.toString(d)+":Marker"+Integer.toString(value)+"_X");
                                headers.add("Drone"+Integer.toString(d)+":Marker"+Integer.toString(value)+"_Y");
                                headers.add("Drone"+Integer.toString(d)+":Marker"+Integer.toString(value)+"_Z");
                            }
                         headers.add("Drone"+Integer.toString(d)+":Center_X");
                         headers.add("Drone"+Integer.toString(d)+":Center_Y");
                         headers.add("Drone"+Integer.toString(d)+":Center_Z");
                         for(int value =0;value<bbox.length;value +=1)
                            {
                                headers.add("Drone"+Integer.toString(d)+":Bbox"+Integer.toString(value)+"_X");
                                headers.add("Drone"+Integer.toString(d)+":Bbox"+Integer.toString(value)+"_Y");
                                headers.add("Drone"+Integer.toString(d)+":Bbox"+Integer.toString(value)+"_Z");
                            }
                     }
                         
                    
                    csvWriter.writeNext(headers.toArray(new String[0]));
                    String[] values = null;
                    for(int i = 0;i<offset;i++)csvReader.readNext();
                    while ((values = csvReader.readNext()) != null) {
                        int value_iterator=0;
                        ArrayList <String> toSave = new ArrayList<>();
                        for(int d =0;d<drones.size();d++)
                        {
                            var drone = drones.get(d);
                            ArrayList<Point3> droneMarkers = new ArrayList<>();
                            for(int value =0;value<drone.length;value ++)
                            {
                                droneMarkers.add(new Point3(Double.parseDouble(values[value_iterator]),Double.parseDouble(values[value_iterator+1]),Double.parseDouble(values[value_iterator+2])));
                                value_iterator +=3;
                            }
                            var crosspoint = comp.get(d).fit(droneMarkers.toArray(new Point3[0]));
                            var global_bbox = Arrays.asList(comp.get(d).boundingCuboid(bbox));
                            ArrayList<Point3> points = new ArrayList<>();
                            points.addAll(droneMarkers);
                            points.add(crosspoint);
                            points.addAll(global_bbox);
                            for(var p:points)
                            {
                                toSave.add(Double.toString(p.x));
                                toSave.add(Double.toString(p.y));
                                toSave.add(Double.toString(p.z));
                            }
                        }
                      
                        csvWriter.writeNext(toSave.toArray(new String[0]));
                    }
                }
            }

//            System.out.println(q1.toString());
        } catch (Exception ex) {
            Logger.getLogger(main.class.getName()).log(Level.SEVERE, null, ex);
        }

    }

}
