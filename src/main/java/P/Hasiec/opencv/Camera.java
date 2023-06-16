/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package P.Hasiec.opencv;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.opencv.core.Point;
import org.opencv.core.Point3;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import org.opencv.core.Mat;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;
import org.w3c.dom.Node;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.apache.commons.numbers.quaternion.Quaternion;
import org.opencv.core.CvType;
import org.opencv.calib3d.Calib3d;
//import org.xml.sax.Parser.
//import org.w3c.dom.Document


/**
 *
 * @author Piotrek
 */
public class Camera {
    Mat projectionMatrix;
    Double focalLength;
    Point principalPoint; 
    Mat Position;
    Quaternion orientation; 
    public Camera() {
    }

    public Camera(Mat projectionMatrix) {
        this.projectionMatrix = projectionMatrix;
    }

    public Camera( double focalLength, Point pp, Point3 Position, Quaternion orientation) {
        this.focalLength = focalLength;
        this.principalPoint = pp;
        this.Position = new Mat(3,1, CvType.CV_64FC1);
        this.Position.put(0,0,Position.x);
        this.Position.put(1,0,Position.y);
        this.Position.put(2,0,Position.z);
        this.orientation = orientation;
        createprojectionMatrix();
    }
     
    private void createprojectionMatrix()
    {
        var cameraMatrix = Mat.zeros(3, 3, CvType.CV_64FC1);
        cameraMatrix.put(0,0,focalLength);
        cameraMatrix.put(1,1,focalLength);
        cameraMatrix.put(2,2,1);
        cameraMatrix.put(0,2,principalPoint.x);
        cameraMatrix.put(1,2,principalPoint.y);
        Mat rotationMatrix  = Mat.zeros(3, 3, CvType.CV_64FC1); 
        var rvec = new Mat(3,1,CvType.CV_64FC1);
        rvec.put(0,0,this.orientation.getX()/this.orientation.getW());
        rvec.put(1,0,this.orientation.getY()/this.orientation.getW());
        rvec.put(2,0,this.orientation.getZ()/this.orientation.getW());
        Calib3d.Rodrigues(rvec, rotationMatrix);
        var cameraExtrinsicMatrix =  Mat.zeros(3, 4, CvType.CV_64FC1);
        var tmpPos = rotationMatrix.matMul(this.Position);
       for(var row = 0; row< rotationMatrix.rows(); row++)
        {
            for(var col = 0; col < rotationMatrix.cols();col++)
            {
                cameraExtrinsicMatrix.put(row,col,rotationMatrix.get(row,col)[0]);
            }
   
        }

        cameraExtrinsicMatrix.put(0,3,tmpPos.get(0,0)[0]);
        cameraExtrinsicMatrix.put(1,3,tmpPos.get(1,0)[0]);
        cameraExtrinsicMatrix.put(2,3,tmpPos.get(2,0)[0]);
        
        for(var row = 0; row< cameraExtrinsicMatrix.rows(); row++)
        {
            for(var col = 0; col < cameraExtrinsicMatrix.cols();col++)
            {
                System.out.print(cameraExtrinsicMatrix.get(row,col)[0]);System.out.print(" ");
            }
            System.out.print("\n");
        }
        projectionMatrix = cameraMatrix.matMul(cameraExtrinsicMatrix);
    }
    
    
    
    
    public static  ArrayList<Camera> loadCamera(String path,ArrayList<String> ids) throws ParserConfigurationException
    {
        DocumentBuilder builder = DocumentBuilderFactory.newInstance().newDocumentBuilder();
        ArrayList<Camera> cameras = new ArrayList<Camera>();
        try {
            var file = new File(path);
            Document doc = builder.parse(path);
     
            doc.getDocumentElement().normalize();
            
//            NodeList nodeList = doc.getChildNodes();
//            var first =  nodeList.item(0);
    
            NodeList nList = doc.getElementsByTagName("Camera");
          
             for (int temp = 0; temp < nList.getLength(); temp++) {
            Node nNode = nList.item(temp);
            Element eElement = (Element) nNode;
            if(ids.contains(eElement.getAttribute("DEVICEID")))
                    {
            NodeList cFrames =  eElement.getElementsByTagName("ControlFrames");
            Element cFrame =  (Element)((Element)cFrames.item(0)).getElementsByTagName("ControlFrame").item(0);
            
            double fl = Double.parseDouble(cFrame.getAttribute("FOCAL_LENGTH"));
            var pptabela = ((String)cFrame.getAttribute("PRINCIPAL_POINT")).split(" ");
            int ppx = Integer.parseInt( pptabela[0]);
            int ppy = Integer.parseInt(pptabela[1]);
            Point pp = new Point(ppx,ppy);
            

            var positionTab = ((String)cFrame.getAttribute("POSITION")).split(" ");
            Point3 position = new Point3(Double.parseDouble(positionTab[0]),Double.parseDouble(positionTab[1]),Double.parseDouble(positionTab[2]));
            var quatTab = ((String)cFrame.getAttribute("ORIENTATION")).split(" ");
            Quaternion orietntation = Quaternion.of(Double.parseDouble( quatTab[3]),Double.parseDouble( quatTab[0]),Double.parseDouble( quatTab[1]),Double.parseDouble( quatTab[2]));
            cameras.add(new Camera(fl,pp,position,orietntation));
             }
             }
        } catch (SAXException | IOException ex) {
            Logger.getLogger(Camera.class.getName()).log(Level.SEVERE, null, ex);
        }
        
        return cameras;
    }
}
