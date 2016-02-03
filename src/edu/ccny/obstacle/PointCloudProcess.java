package edu.ccny.obstacle;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;

public class PointCloudProcess {
    public static final String TAG = PointCloudProcess.class.getSimpleName();
	// LOG.setLevel(Level.WARNING), LOG.severe/warning/info/finest
    private static Logger LOG = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);
    
    public void PointCloudProcess() {
    	LOG.setLevel(Level.WARNING);
        return;
    }

    public int rotationRollPitchCorrect (float [] buffer, FloatBuffer buf,
                                         double [][] rotaRoll, double [][] rotaPitch) {
        if (null == buffer || null == rotaRoll || null == rotaPitch) {
            return 0;
        }

        double px, py, pz;
        final int X = 0, Y = 1, Z = 2;
        double [][] r = rotaRoll;
        double [][] p = rotaPitch;

        double r00 = p[0][0]*r[0][0] + p[0][1]*r[1][0] + p[0][2]*r[2][0];
        double r01 = p[0][0]*r[0][1] + p[0][1]*r[1][1] + p[0][2]*r[2][1];
        double r02 = p[0][0]*r[0][2] + p[0][1]*r[1][2] + p[0][2]*r[2][2];
        double r10 = p[1][0]*r[0][0] + p[1][1]*r[1][0] + p[1][2]*r[2][0];
        double r11 = p[1][0]*r[0][1] + p[1][1]*r[1][1] + p[1][2]*r[2][1];
        double r12 = p[1][0]*r[0][2] + p[1][1]*r[1][2] + p[1][2]*r[2][2];
        double r20 = p[2][0]*r[0][0] + p[2][1]*r[1][0] + p[2][2]*r[2][0];
        double r21 = p[2][0]*r[0][1] + p[2][1]*r[1][1] + p[2][2]*r[2][1];
        double r22 = p[2][0]*r[0][2] + p[2][1]*r[1][2] + p[2][2]*r[2][2];
        
        if (buf == null) {
            LOG.warning("rotationRollCorrect(): FloatBuffer is null");
            return 0;
        }
        if (buf.capacity() <= 3) {
        	LOG.warning("rotationRollCorrect(): FloatBuffer has too few points");
            return 0;
        }

        int bufferSize = buf.capacity();
        if (bufferSize >= 90000) {
            bufferSize = 90000;
        }

        for(int c = 0; c < bufferSize - 3; c += 3) {
            // raw sensor data: x: go-right, y: go-down,    z: go-forward
            // world frame:     x: go-right, y: go-forward, z: go-up
            px = buf.get(c + X);
            py = buf.get(c + Z);
            pz = buf.get(c + Y) * (-1);

            // new_point = rotation * point
            float new_px = (float) (r00 * px + r01 * py + r02 * pz);
            float new_py = (float) (r10 * px + r11 * py + r12 * pz);
            float new_pz = (float) (r20 * px + r21 * py + r22 * pz);

            buffer[c + X] = new_px;
            buffer[c + Y] = new_py;
            buffer[c + Z] = new_pz;
        }
        return bufferSize;
    }
    
    public int rotationRollCorrect(float [] buffer, FloatBuffer buf, double [][] rota) {
        if (null == buffer || null == rota) {
            return 0;
        }

        double px, py, pz;
        final int X = 0, Y = 1, Z = 2;
        double r00 = rota[0][0];
        double r01 = rota[0][1];
        double r02 = rota[0][2];
        double r10 = rota[1][0];
        double r11 = rota[1][1];
        double r12 = rota[1][2];
        double r20 = rota[2][0];
        double r21 = rota[2][1];
        double r22 = rota[2][2];

        if (buf == null) {
        	LOG.warning("rotationRollCorrect(): FloatBuffer is null");
            return 0;
        }
        if (buf.capacity() <= 3) {
        	LOG.warning("rotationRollCorrect(): FloatBuffer has too few points");
            return 0;
        }

        int bufferSize = buf.capacity();
        if (bufferSize >= 90000) {
            bufferSize = 90000;
        }

        for(int c = 0; c < bufferSize - 3; c += 3) {
            // raw sensor data: x: go-right, y: go-down,    z: go-forward
            // world frame:     x: go-right, y: go-forward, z: go-up
            px = buf.get(c + X);
            py = buf.get(c + Z);
            pz = buf.get(c + Y) * (-1);

            // new_point = rotation * point
            float new_px = (float) (r00 * px + r01 * py + r02 * pz);
            float new_py = (float) (r10 * px + r11 * py + r12 * pz);
            float new_pz = (float) (r20 * px + r21 * py + r22 * pz);

            buffer[c + X] = new_px;
            buffer[c + Y] = new_py;
            buffer[c + Z] = new_pz;
        }
        return bufferSize;
    }

    public float [] pointsToInitialFrame(float [] buffer,
                                         int bufferSize,
                                         float[] translation,
                                         float[] quaternion,
                                         float[] eulerAngle) { // as output
        if (buffer == null) {
        	LOG.info("pointsToInitialFrame(): buffer is null");
            return null;
        }

        // translation vector in tango frame
        float tangox = translation[0];
        float tangoy = translation[1];
        float tangoz = translation[2];
        
        // rotation matrix in tango frame
        double x = quaternion[0];
        double y = quaternion[1];
        double z = quaternion[2];
        double w = quaternion[3];
        MyQuaternion quat = new MyQuaternion(w, x, y, z);
        MyMatrix rotaMatrix = quat.RotationMatrix();
        double [][] rota = rotaMatrix.getValues();
        double r00 = rota[0][0];
        double r01 = rota[0][1];
        double r02 = rota[0][2];
        double r10 = rota[1][0];
        double r11 = rota[1][1];
        double r12 = rota[1][2];
        double r20 = rota[2][0];
        double r21 = rota[2][1];
        double r22 = rota[2][2];
        double [] angle= quat.getEulerAngle(); // bing
        eulerAngle[0] = (float) angle[0];
        eulerAngle[1] = (float) angle[1];
        eulerAngle[2] = (float) angle[2];

        for (int i = 0; i < bufferSize; i += 3) {
            // depth camera frame
            float point_x = buffer[i + 0]; // buffer.get(i + 0);
            float point_y = buffer[i + 1]; // buffer.get(i + 1);
            float point_z = buffer[i + 2]; // buffer.get(i + 2);

            boolean bShowingLocalFrame = false;
            
            // https://developers.google.com/project-tango/overview/coordinate-systems
            if (bShowingLocalFrame == true) {
                // depth camera frame -> local world frame
                // depth camera frame uses a left-handed coordinate system
                // float px = point_x;
                float py = point_z;
                float pz = point_y;

                // buffer[i + 0] = px;
                buffer[i + 1] = py;
                buffer[i + 2] = pz;
            } else {
                // depth camera frame -> tango device frame (DEVICE frame)
                float px = point_x + 0.05f;
                float py = point_y + 0.05f;
                float pz = point_z * (-1);
                
                // tango frame -> global world frame (START_OF_SERVICE frame)
                // new_point = rotation * point + translation
                float new_px = (float) (r00 * px + r01 * py + r02 * pz + tangox);
                float new_py = (float) (r10 * px + r11 * py + r12 * pz + tangoy);
                float new_pz = (float) (r20 * px + r21 * py + r22 * pz + tangoz);

                buffer[i + 0] = new_px;
                buffer[i + 1] = new_py;
                buffer[i + 2] = new_pz;
            }
        }

        return buffer;
    }

    public FloatBuffer arrayToFloatBuffer(float [] array, int cnt) {
        final int FLOAT_SIZE = 4;
        
        if (0 == cnt) {
        	LOG.warning("arrayToFloatBuffer() return 0");
            return null;
        }
        
        FloatBuffer bufferOut = ByteBuffer.allocateDirect(cnt * FLOAT_SIZE)
                .order(ByteOrder.nativeOrder()).asFloatBuffer();
        for (int i = 0; i < cnt; i++) {
            bufferOut.put(i, array[i]);
        }

        return bufferOut;
    }

    public int transformByRoll(float [] array, float rollEuler) {
        return 0;
    }

    public int downsampleByRange(float [] array, int bufferSize,
                                 float widthMin,   float widthMax,
                                 float heightMin,  float heightMax,
                                 float forwardMin, float forwardMax) {
        final int X = 0;
        final int Y = 1;
        final int Z = 2;
        final int FLOAT_SIZE = 4;
        float x = 0;
        float y = 0;
        float z = 0;

        int cnt = 0;
        for(int c = 0; c < bufferSize - 3; c += 3) {
            x = array[c + X];
            y = array[c + Y];
            z = array[c + Z];

            // in world frame: x: go-right, y: go-forward, z: go-up
            if (    (x < widthMin)   || (x > widthMax)
                 || (y < forwardMin) || (y > forwardMax)
                 || (z < heightMin)  || (z > heightMax)) {
                continue;
            }

            /*if (true == filterAroundPoint(array, c)) {
                continue;
            }*/

            array[cnt] = x;
            cnt += 1;
            array[cnt] = y;
            cnt += 1;
            array[cnt] = z;
            cnt += 1;
            if (cnt >= 90000) {
                break;
            }
        }
        return cnt;
    }

    private boolean filterAroundPoint(float [] buffer, int c) {
        int bufferSize = buffer.length;
        final int X = 0;
        final int Y = 1;
        final int Z = 2;
        
        double x = buffer[c + X];
        double y = buffer[c + Y];
        double z = buffer[c + Z];
        
        // filter the alone points
        if ((c >= 6 ) && (c < (bufferSize - 9))) {
            double dis_1 =  (x - buffer[c + X - 6]) *  (x - buffer[c + X - 6])
                          + (y - buffer[c + Y - 6]) *  (y - buffer[c + Y - 6])
                          + (z - buffer[c + Z - 6]) *  (z - buffer[c + Z - 6]);
            dis_1 = Math.sqrt(dis_1);
            double dis_2 =  (x - buffer[c + X - 3]) *  (x - buffer[c + X - 3])
                          + (y - buffer[c + Y - 3]) *  (y - buffer[c + Y - 3])
                          + (z - buffer[c + Z - 3]) *  (z - buffer[c + Z - 3]);
            dis_2 = Math.sqrt(dis_2);
            double dis1 =   (x - buffer[c + X + 3]) *  (x - buffer[c + X + 3])
                          + (y - buffer[c + Y + 3]) *  (y - buffer[c + Y + 3])
                          + (z - buffer[c + Z + 3]) *  (z - buffer[c + Z + 3]);
            dis1 = Math.sqrt(dis1);
            double dis2 =   (x - buffer[c + X + 6]) *  (x - buffer[c + X + 6])
                          + (y - buffer[c + Y + 6]) *  (y - buffer[c + Y + 6])
                          + (z - buffer[c + Z + 6]) *  (z - buffer[c + Z + 6]);
            dis2 = Math.sqrt(dis2);

            double dis0 =  x*x + y*y + z*z;
            dis0 = Math.sqrt(dis0);
            double threshold = (dis0/1) * 0.015;
            
            if ((dis_1 > threshold) && (dis_2 > threshold) && (dis1 > threshold) && (dis2 > threshold)) {
                return true;
            }
        }
        return false;
    }

}

