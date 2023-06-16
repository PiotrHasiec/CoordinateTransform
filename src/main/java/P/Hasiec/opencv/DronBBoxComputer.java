package P.Hasiec.opencv;

/**
 *
 * @author Piotrek
 */
import static java.lang.Math.sqrt;
import static java.lang.Math.abs;
import java.util.ArrayList;
import org.apache.commons.numbers.quaternion.Quaternion;
import org.opencv.core.Point3;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.CvType;

public class DronBBoxComputer {

    private Point3 versorX;
    private Point3 versorY;
    private Point3 versorZ;
    private Point3 position = new Point3(0, 0, 0);
    private final Point3 centerOffset;
    private Quaternion startOrientation;
    private final boolean correct;
    public final Point3[] localMarkers;

    public DronBBoxComputer(Point3[] localMarkers,boolean normalizeStartOrientation) {
        centerOffset = new Point3(0, 0, 0);
        startOrientation = Quaternion.ONE;
        this.correct = normalizeStartOrientation;
        this.localMarkers = localMarkers;

    }

    public DronBBoxComputer(Point3[] localMarkers,boolean normalizeStartOrientation, Point3 centerOffset) throws Exception {
        if(centerOffset ==null)
            throw new Exception("centerOffset was null");
        if(localMarkers ==null || localMarkers.length <3)
            throw new Exception("localMarkers has wrong size or is null");
        
        this.centerOffset = centerOffset.clone();
        this.localMarkers = localMarkers;
        startOrientation = Quaternion.of(1, 0, 0, 0);
        this.correct = normalizeStartOrientation;
    }

    public DronBBoxComputer(Point3[] localMarkers,Point3 centerOffset) throws Exception {
        if(centerOffset ==null)
            throw new Exception("centerOffset was null");
        
        if(localMarkers ==null || localMarkers.length <3)
            throw new Exception("localMarkers has wrong size or is null");
        
        this.centerOffset = centerOffset.clone();
        startOrientation = Quaternion.of(1, 0, 0, 0);
        this.localMarkers = localMarkers;
        this.correct = false;
    }

    public DronBBoxComputer(Point3[] localMarkers) throws Exception {
        if(localMarkers ==null || localMarkers.length <3)
            throw new Exception("localMarkers has wrong size or is null");
        
        this.localMarkers = localMarkers;
        centerOffset = new Point3(0, 0, 0);
        startOrientation = Quaternion.ONE;
        this.correct = false;
        
    }

    public static Point3 sum(Point3 p1, Point3 p2) {
        return new Point3(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
    }

    public static Point3 mul(Point3 p, double scaler) {
        return new Point3(p.x * scaler, p.y * scaler, p.z * scaler);
    }

    public Point3 toGlobal(Point3[] cross, Point3 point) {
        this.versorX = sum(cross[1], mul(cross[0], -1));//obliczam wektor kierunkowy osi x
        versorX = mul(versorX, 1 / Math.sqrt(versorX.dot(versorX)));
        // normalizacja 
        this.versorY = sum(cross[3], mul(cross[2], -1));//obliczam wektor kierunkowy osi y
        versorY = mul(versorY, 1 / Math.sqrt(versorY.dot(versorY)));// normalizacja 

        this.versorZ = versorX.cross(versorY);//obliczam wersor kierunkowy osi z jako ilczyn wektorowy wersorów x i y - będzie porostopadły
        return sum(cross[4], sum(mul(versorX, point.x + this.centerOffset.x), sum(mul(versorY, point.y + this.centerOffset.y), mul(versorZ, point.z + this.centerOffset.z))));
    }
    
     public Point3 toGlobal( Point3 point) {
        return sum(position, sum(mul(versorX, point.x + this.centerOffset.x), sum(mul(versorY, point.y + this.centerOffset.y), mul(versorZ, point.z + this.centerOffset.z))));
    }

    public Point3[] boundingCuboid(Point3[] cross, Point3[] localCuboid) {

        if (cross.length != 5 || localCuboid.length != 8) {
            return null;
        }
        Point3[] globalCuboid = new Point3[localCuboid.length];
        for (int i = 0; i < localCuboid.length; i++) {
            Point3 p = localCuboid[i];
            globalCuboid[i] = toGlobal(cross, p);
        }
        return globalCuboid;
    }
    
     public Point3[] boundingCuboid( Point3[] localCuboid) {

        Point3[] globalCuboid = new Point3[localCuboid.length];
        for (int i = 0; i < localCuboid.length; i++) {
            Point3 p = localCuboid[i];
            globalCuboid[i] = toGlobal(p);
        }
        return globalCuboid;
    }

    private ArrayList<int[]> generatePermutation(int[] seq, Integer pos, ArrayList<int[]> listOfPerms) {
        if (pos == null) {
            pos = 0;
        }
        if (pos == seq.length) {
            listOfPerms.add(seq.clone());
        } else {
            for (int i = pos; i < seq.length; i++) {
                int temp = seq[i];
                seq[i] = seq[pos];
                seq[pos] = temp;
                generatePermutation(seq, pos + 1, listOfPerms);
                temp = seq[i];
                seq[i] = seq[pos];
                seq[pos] = temp;
            }
        }
        return listOfPerms;
    }

    /**
     *
     * @param globalPoints
     * @return
     * @throws Exception
     */
    public Point3 fit( Point3[] globalPoints) throws Exception {
        if (this.localMarkers == null || globalPoints == null || this.localMarkers.length != globalPoints.length) {
            throw new Exception("global or local points tabla is wrong");
        }

        Mat Transform;
        var Local = new Mat(4, this.localMarkers.length, CvType.CV_64FC1);
        var invLocal = new Mat(this.localMarkers.length, 4, CvType.CV_64FC1);
        var Global = new Mat(3, this.localMarkers.length, CvType.CV_64FC1);
        int[] perm = new int[globalPoints.length];
        for (int i = 0; i < this.localMarkers.length; i++) {
            Local.put(0, i, this.localMarkers[i].x);
            Local.put(1, i, this.localMarkers[i].y);
            Local.put(2, i, this.localMarkers[i].z);
            Local.put(3, i, 1);
            perm[globalPoints.length - 1 - i] = i;
        }
        ArrayList<int[]> listOfPerms = new ArrayList<>();
        Core.invert(Local, invLocal, Core.DECOMP_SVD);

        listOfPerms = generatePermutation(perm, 0, listOfPerms);
        int[] min = listOfPerms.get(0);
        double minValue = 0;
        for (var p : listOfPerms) {
            for (int i = 0; i < p.length; i++) {
                Global.put(0, p[i], globalPoints[i].x);
                Global.put(1, p[i], globalPoints[i].y);
                Global.put(2, p[i], globalPoints[i].z);
            }

            Transform = Global.matMul(invLocal);
            var X = Transform.matMul(Local);
            double value = 0;

            for (int cols = 0; cols < Global.cols(); cols++) {
                for (int rows = 0; rows < Global.rows(); rows++) {

                    value += Math.pow((X.get(rows, cols)[0] - Global.get(rows, cols)[0]), 2);
                }
            }
            if (p == min || minValue > value) {
                minValue = value;
                min = p;
            }

        }

        for (var i : min) {
            Global.put(0, min[i], globalPoints[i].x);
            Global.put(1, min[i], globalPoints[i].y);
            Global.put(2, min[i], globalPoints[i].z);
        }
        Transform = Global.matMul(invLocal);

        this.versorX = new Point3(Transform.get(0, 0)[0], Transform.get(1, 0)[0], Transform.get(2, 0)[0]);
        this.versorX = mul(this.versorX, 1 / Math.sqrt(this.versorX.dot(this.versorX)));
        this.versorY = new Point3(Transform.get(0, 1)[0], Transform.get(1, 1)[0], Transform.get(2, 1)[0]);
        this.versorY = mul(this.versorY, 1 / Math.sqrt(this.versorY.dot(this.versorY)));
//        this.versorZ  = new Point3(Transform.get(0, 2)[0],Transform.get(1, 2)[0],Transform.get(2, 2)[0]);
        this.versorZ = this.versorX.cross(this.versorY);
        this.versorZ = mul(this.versorZ, 1 / Math.sqrt(this.versorZ.dot(this.versorZ)));
        Point3 center = new Point3(Transform.get(0, 3)[0], Transform.get(1, 3)[0], Transform.get(2, 3)[0]);
        position = center.clone();
        return center;
    }

    /**
     * Funkcja oblicza punkt przecięcia dla ramion krzyża w 3d z uwzględnieniem,
     * że jeden z 4 punktów może nie być współpaszczyznowy z pozostałymi trzema
     *
     * @param[in] four_points wektor 4 punktów ramion krzyża w postaci
     * niekoniecznie uporządkowanej
     * @param[in] arms wektor dwóch wartości stanowiących długości ramion krzyża
     * @param[in] err maksymalny dopuszczalny błąd między faktyczną długością
     * ramienia, a długością obliczoną na podstawie punktów (w jednostkach tych
     * samych co pozycje markerów ramion)
     * @return wektor 5 punktów krzyża w postaci [1. punkt ramienia x,2. punkt
     * ramienia x, 1. punkt ramienia y, 2. punkt ramienia y, punkt przecięcia]
     */
    public Point3[] cross3d(Point3[] four_points, double[] arms, double err) {
        //znalezienie które ramiona znajdują sie naprzeciwko siebie poprzez sprawdzenie odległości między każdym możliwym dopasowaniem punktów w pary
        int[] arms_order = null;

        for (int[] j : new int[][]{{1, 2, 3}, {2, 1, 3}, {3, 1, 2}}) {

            Point3 arm1 = sum(four_points[0], mul(four_points[j[0]], -1));
            Point3 arm2 = sum(four_points[j[1]], mul(four_points[j[2]], -1));
            if ((Math.abs(Math.sqrt(arm1.dot(arm1)) - arms[0]) < err && abs(sqrt(arm2.dot(arm2)) - arms[1]) < err) || (abs(sqrt(arm1.dot(arm1)) - arms[1]) < err && abs(sqrt(arm2.dot(arm2)) - arms[0]) < err)) {
                arms_order = j;
                break;
            }
        }

        if (arms_order == null) {
            return null;
        }

        //poniższe obliczenia zostały wyprowadzone z warunku, że iloczyn skalarny między wektorem rozpiętym od punktu środkowego do jednego z ramion, a wektorem od punktu środkowego do końca ramienia prostopadłego powinien wynosić 0
        Point3 delta_collinear = sum(four_points[arms_order[0]], mul(four_points[0], -1));
        Point3 delta_non_collinear = sum(four_points[arms_order[1]], mul(four_points[0], -1));
        double denominator = (delta_collinear.x * delta_collinear.x + delta_collinear.y * delta_collinear.y + delta_collinear.z * delta_collinear.z);
        double t = (delta_collinear.x * delta_non_collinear.x + delta_collinear.y * delta_non_collinear.y + delta_collinear.z * delta_non_collinear.z) / denominator;
        Point3 cross_point = sum(four_points[0], mul(delta_collinear, t));
        return new Point3[]{four_points[0], four_points[arms_order[0]],
            four_points[arms_order[1]], four_points[arms_order[2]], cross_point};
    }

    public Point3[] cross3d(Point3[] four_points) {
        return cross3d(four_points, new double[]{180, 80}, 120);
    }

    /**
     * A method written from the conversion between quaternions and rotation
     * matrix
     *
     * @return rotation quaternion of local system
     * @throws Exception
     */
    public ArrayList<Double> rotation_quaternion() throws Exception {

        if (this.versorX == null || this.versorY == null || this.versorZ == null) {
            throw new Exception("One or more axis versor are null");

        }

        ArrayList<Double> rotation_quat = new ArrayList<>();
        double w = 0.0;// Math.sqrt(1+this.versorX.x + this.versorY.y +this.versorZ.z)/2;
        double qx = 0.0;//  (this.versorZ.y - this.versorY.z)/(4*w);
        double qy = 0.0;//  (this.versorZ.x - this.versorX.z)/(4*w);
        double qz = 0.0;//  (this.versorX.y - this.versorY.x)/(4*w);

        Double[][] m = new Double[][]{
            {this.versorX.x, this.versorY.x, this.versorZ.x},
            {this.versorX.y, this.versorY.y, this.versorZ.y},
            {this.versorX.z, this.versorY.z, this.versorZ.z}
        };
        Double t = m[0][0] + m[1][1] + m[2][2];
        if (t > 0.0) {
            double s = sqrt(t + 1.0) * 2;
            w = (s * 0.25);
            qx = (m[2][1] - m[1][2]) / s;
            qy = (m[0][2] - m[2][0]) / s;
            qz = (m[1][0] - m[0][1]) / s;
        } else if ((m[0][0] > m[1][1]) & (m[0][0] > m[2][2])) {
            double s = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2; // S=4*qx 
            w = (m[2][1] - m[1][2]) / s;
            if (w < 0) {
                w = w * (-1);
                s = s * (-1);
            }
            qx = 0.25 * s;
            qy = (m[0][1] + m[1][0]) / s;
            qz = (m[0][2] + m[2][0]) / s;
        } else if (m[1][1] > m[2][2]) {
            double s = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2; // s=4*qy
            w = (m[0][2] - m[2][0]) / s;
            if (w < 0) {
                w = w * (-1);
                s = s * (-1);
            }
            qx = (m[0][1] + m[1][0]) / s;
            qy = 0.25 * s;
            qz = (m[1][2] + m[2][1]) / s;
        } else {
            double s = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2; // s=4*qz
            w = (m[1][0] - m[0][1]) / s;
            if (w < 0) {
                w = w * (-1);
                s = s * (-1);
            }
            qx = (m[0][2] + m[2][0]) / s;
            qy = (m[1][2] + m[2][1]) / s;
            qz = 0.25 * s;
        }
        Quaternion norm_rotation_quat = Quaternion.of(w, -qx, -qy, -qz);
        if (this.correct && this.startOrientation == Quaternion.ONE) {
            this.startOrientation = norm_rotation_quat.inverse();
        }
        norm_rotation_quat = norm_rotation_quat.multiply(this.startOrientation);
        rotation_quat.add(norm_rotation_quat.getW());
        rotation_quat.add(norm_rotation_quat.getX());
        rotation_quat.add(norm_rotation_quat.getY());
        rotation_quat.add(norm_rotation_quat.getZ());
        return rotation_quat;
    }

    /**
     * A method written on the basis of the geometric interpretation of the
     * quatrmion rotation.
     *
     * @return rotation quaternion of local system
     * @throws Exception
     */
    public ArrayList<Double> rotation_quaternion2() throws Exception {

        if (this.versorX == null || this.versorY == null || this.versorZ == null) {
            throw new Exception("One or more axis versor are null");

        }

        ArrayList<Double> rotation_quat = new ArrayList<>();

        var vx = new Point3(1, 0, 0);
        var vy = new Point3(0, 1, 0);
        Point3 d1 = sum(mul(this.versorX, -1), vx);
        Point3 d2 = sum(mul(this.versorY, -1), vy);

        Double m = Math.sqrt(Math.pow(d1.z * d2.y - d1.y * d2.z, 2) + Math.pow(d1.y * d2.x - d2.y * d1.x, 2) + Math.pow(d2.z * d1.x - d1.z * d2.x, 2));

        double sign = Math.signum(d1.y * d2.x - d1.x * d2.y);
        Double qx = sign * (d1.y * d2.z - d1.z * d2.y) / m;
        Double qy = sign * (d1.x * d2.z - d1.z * d2.x) / m;
        Double qz = Math.abs(d1.y * d2.x - d1.x * d2.y) / m;

        Double t = (this.versorX.x * qx + this.versorX.y * qy + this.versorX.z * qz) / (Math.pow(qx, 2) + Math.pow(qy, 2) + Math.pow(qz, 2));
        Point3 vt = mul(new Point3(qx, qy, qz), -t);
        Point3 v1 = sum(this.versorX, vt);
        Point3 v2 = sum(vx, vt);
        Double cos_a = (v1.dot(v2)) / Math.sqrt(v1.dot(v1) * v2.dot(v2));
        rotation_quat.add(Math.pow((cos_a + 1) / 2, 0.5));
        rotation_quat.add(qx * Math.pow((1 - cos_a) / 2, 0.5));
        rotation_quat.add(qy * Math.pow((1 - cos_a) / 2, 0.5));
        rotation_quat.add(qz * Math.pow((1 - cos_a) / 2, 0.5));
        return rotation_quat;
    }

    public Point3[] compute_versor(Point3[] cross) {
        this.versorX = sum(cross[1], mul(cross[0], -1));//obliczam wektor kierunkowy osi x
        versorX = mul(versorX, 1 / Math.sqrt(versorX.dot(versorX)));
        // normalizacja 
        this.versorY = sum(cross[3], mul(cross[2], -1));//obliczam wektor kierunkowy osi y
        versorY = mul(versorY, 1 / Math.sqrt(versorY.dot(versorY)));// normalizacja 

        this.versorZ = versorX.cross(versorY);//obliczam wersor kierunkowy osi z jako ilczyn wektorowy wersorów x i y - będzie porostopadły
        //versorZ = versorZ / sqrt(versorZ.dot(versorZ));

        return new Point3[]{this.versorX, this.versorY, this.versorZ};
    }

    /**
     *
     * @return Matrix of rotation
     * @throws Exception
     */
    public Double[][] rotation_matrix() throws Exception {

        if (this.versorX == null || this.versorY == null || this.versorZ == null) {
            throw new Exception("One or more axis versor are null");

        }

        Double[][] rotation_matrix = new Double[][]{{versorX.x, versorY.x, versorZ.x}, {versorX.y, versorY.y, versorZ.y}, {versorX.z, versorY.z, versorZ.z}};

        return rotation_matrix;
    }

}
