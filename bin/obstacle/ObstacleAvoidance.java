package edu.ccny.obstacle;

import java.nio.FloatBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;

public class ObstacleAvoidance {
    private float []  mDataAll = null;
    private int       mDataAllLen = 0;
    private int []    mDataPointsInt = null;
    private int []    mObstacleBoxesInt = null;
    private boolean   isGlobalLocalized = true;
    private float []  mDataPointsLocal = null;
    private float []  mObstacleBoxesLocal = null;
    private float []  mObstacleBoxesLocalVertical = null;
    private final int mPointsMinimum = 4;
    private boolean   obstacleDetectFlag = true;
    private boolean   postponeObstacleDetectFlag = false;
    private long      postponeObstacleDetectTimeStart = 0;
    private int       postponeObstacleDetectTime = 0;

    private static final String TAG = ObstacleAvoidance.class.getSimpleName();
	// LOG.setLevel(Level.WARNING), LOG.severe/warning/info/finest
    private static Logger LOG = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);
    
    private float xMin = -0.6f; // from config
    private float xMax =  0.6f;
    private float yMin =  0.5f;
    private float yMax =  2.0f;
    private float zMin = -0.6f;
    private float zMax =  0.5f;
    private MapUpdaterGlobal mMapGlobal = null;
    private MapUpdaterLocal  mMapLocal  = null;
    private MapUpdaterLocal  mMapLocalVertical = null;
    private static final int X = 0;
    private static final int Y = 1;
    private static final int Z = 2;
    private float [] mObstaclePoint;
    
    private static final int mPointsNumMax = 30000;
    private static int mDataFloatLenMax = mPointsNumMax*3;

    private double mTimeStamp = 0;
    private float [] mTranslation = new float[3];
    private float [] mQuaternion  = new float[4];
    private FloatBuffer mPointCloudBuffer;
    private int mDetectingCnt = 0;
    private int mDetectingObstacleCnt = 0;
    private int mCostTimeSumAll = 0;
    private byte [][] mMap;
    private int mFramesCnt = 0;

    private PointCloudProcess mPointCloudProcess = new PointCloudProcess();

    // Two types of transformation
    // (1) use (scale, x_offset, y_offset), then its size should be 3
    // (2) metric2Grid matrix, then its size should be 9
    private final static int TRANSFORM_BY_SCALE_OFFSET = 3;
    private final static int TRANSFORM_BY_MATRIX = 9;

    public float [] mMetric2GridTransform = null;

    public ObstacleAvoidance(byte [][] map, float [] transform, int obstacleWall, float localGranularityMeter, float [] range) {
    	LOG.setLevel(Level.WARNING);
    	if (null != range) {
            if (6 == range.length) {
                xMin = range[0];
                xMax = range[1];
                yMin = range[2];
                yMax = range[3];
                zMin = range[4];
                zMax = range[5];
            } else {
                LOG.warning("ObstacleAvoidance() invalid range, use default");
            }
        }

        init(map, transform, obstacleWall, localGranularityMeter);
    }

    private void init(byte [][] map, float [] metric2Grid, int obstacleWall, float localGranularityMeter) {
        mDataAll = new float[mDataFloatLenMax];

        int len = metric2Grid.length;
        if (TRANSFORM_BY_SCALE_OFFSET != len && TRANSFORM_BY_MATRIX != len) {
        	LOG.severe("ObstacleAvoidance() invalid transform");
            return;
        }

        mMetric2GridTransform = new float[len];
        float [] m = mMetric2GridTransform;
        for (int i = 0; i < len; i++) {
            m[i] = metric2Grid[i];
        }

        String info = "";
        if (TRANSFORM_BY_SCALE_OFFSET == len) {
            // deprecated code
            // 1st element, scale is: Grids per meter
            info = String.format("Transform: [scale, i, j] = [%.2f, %.2f, %.2f]",
                    m[0], m[1], m[2]);
        } else if (TRANSFORM_BY_MATRIX == len) {
            info = String.format("Transform Matrix = [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                    m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
        }
        LOG.info("info");

        mMap = map;
        mMapGlobal = new MapUpdaterGlobal(map, obstacleWall);
        mMapLocal  = new MapUpdaterLocal(xMin, xMax, yMin, yMax, zMin, zMax, localGranularityMeter, Z);
        mMapLocalVertical = new MapUpdaterLocal(xMin, xMax, yMin, yMax, zMin, zMax, localGranularityMeter, Y);
    }

    public boolean updatingData(FloatBuffer buffer, float [] translation, float [] quaternion, double timeStamp) {
        mPointCloudBuffer = buffer;
        if (null == translation || null == quaternion) {
            return false;
        }
        for (int i = 0; i < 3; i++) {
            mTranslation[i] = translation[i];
        }
        for (int i = 0; i < 4; i++) {
            mQuaternion[i]  = quaternion[i];
        }

        mTimeStamp = timeStamp;
        return true;
    }

    public float [] localMetric2GlobalMetric(float [] buffer, int bufferSize) {
        float px, py, pz;
        final int X = 0, Y = 1;

        float tran_x = mTranslation[0];
        float tran_y = mTranslation[1];
        float [] eulerAngle = MyQuaternion.getEulerAngle(mQuaternion);
        float yawEuler = eulerAngle[2];

        double [][] rota = MyQuaternion.get2DTransformModel(tran_x, tran_y, yawEuler);
        double r00 = rota[0][0];
        double r01 = rota[0][1];
        double r02 = rota[0][2];
        double r10 = rota[1][0];
        double r11 = rota[1][1];
        double r12 = rota[1][2];

        for (int i = 0; i < bufferSize; i += 2) {
            px = buffer[i + X];
            py = buffer[i + Y];
            pz = 1;

            // new_point = rotation * point + translation
            float new_px = (float) (r00 * px + r01 * py + r02 * pz);
            float new_py = (float) (r10 * px + r11 * py + r12 * pz);
            buffer[i + X] = new_px;
            buffer[i + Y] = new_py;
        }
        return buffer;
    }

    public float [] globalMetric2Pixel(float [] buffer, int bufferSize) {
        final int X = 0, Y = 1;
        if (null == buffer) {
            return null;
        }
        if (null == mMetric2GridTransform) {
            return null;
        }
        
        int modelLen = mMetric2GridTransform.length;
        if (TRANSFORM_BY_SCALE_OFFSET == modelLen) {
            float gridPerMeter = mMetric2GridTransform[0];
            float i_offset     = mMetric2GridTransform[1];
            float j_offset     = mMetric2GridTransform[2];

            for (int c = 0; c < bufferSize; c += 2) {
                float x = buffer[c + X];
                float y = buffer[c + Y];

                // similar with transform4Display()
                float i = (-1) * x * gridPerMeter + i_offset;
                float j = y * gridPerMeter + j_offset;
                
                buffer[c + X] = i;
                buffer[c + Y] = j;
            }
        } else if (TRANSFORM_BY_MATRIX == modelLen) {
            float m00 = mMetric2GridTransform[0];
            float m01 = mMetric2GridTransform[1];
            float m02 = mMetric2GridTransform[2];
            float m10 = mMetric2GridTransform[3];
            float m11 = mMetric2GridTransform[4];
            float m12 = mMetric2GridTransform[5];

            for (int c = 0; c < bufferSize; c += 2) {
                float x = buffer[c + X];
                float y = buffer[c + Y];
                float z = 1;

                float i = m00 * x + m01 * y + m02 * z;
                float j = m10 * x + m11 * y + m12 * z;
                j = mMap[0].length - j; // it's height!!!

                buffer[c + X] = i;
                buffer[c + Y] = j;
            }
        }

        return buffer;
    }

    public boolean getIsGlobalLocalized () {
        return isGlobalLocalized;
    }

    public void setIsGlobalLocalized (boolean flag) {
        isGlobalLocalized = flag;
    }

    public void disable() {
        obstacleDetectFlag = false;
    }

    public void enable() {
        obstacleDetectFlag = true;
    }
    
    public void postpone(int ms) {
        if (0 != ms) {
            postponeObstacleDetectFlag = true;
            postponeObstacleDetectTimeStart = System.currentTimeMillis();
            postponeObstacleDetectTime = ms;
        }
    }
    
    public float [] getObstacleCloestPoint () {
    	return mObstaclePoint;
    }
    
    public int [] getObstaclePoints() {
        // get obstacle points [i1, j1, i2, j2, ...] in Image Frame from Global World Metric Frame Transformation
        return mDataPointsInt;
    }

    public float [] getObstaclePointsLocal() {
        // get obstacle points [x1, y1, x2, y2, ...] in Local Metric World Frame
        return mDataPointsLocal;
    }
    
    public int [] getObstacleBoxes() {
        // get obstacle boxes [i1, j1, i2, j2, ...] in Image Frame from Global World Metric Frame Transformation
        return mObstacleBoxesInt;
    }

    public float [] getObstacleBoxesLocal() {
        // get obstacle points [x1, y1, x2, y2, ...] in Local Metric World Frame
        return mObstacleBoxesLocal;
    }
    
    public boolean detecting() {
        String info;

        if (false == obstacleDetectFlag) {
            return false;
        }

        long time = System.currentTimeMillis();
        if (true == postponeObstacleDetectFlag
                && time - postponeObstacleDetectTimeStart < postponeObstacleDetectTime) {
            return false;
        } else {
            postponeObstacleDetectFlag = false;
        }

        if (null == mMetric2GridTransform) {
            return false;
        }
        boolean findObstacle = false;
        mDataPointsLocal = null;
        mObstacleBoxesInt = null;
        long startTime = System.currentTimeMillis();

        float [] eulerAngle = MyQuaternion.getEulerAngle(mQuaternion);
        // When the device vertical, roll is 90 degree
        // If device looks down, roll becomes like: 60 degree
        // In this case, we should let all points to rotate about x-axis (60 - 90) degree
        // plus 14.5 which is from the calibration angle between RRB-D sensor & Vertical
        // vise versa for looks up.
        double angleDegree = eulerAngle[0] - 90 + 14.5;
        double [][] rotaMatrixRoll = MyQuaternion.getRotationMatrixByRoll(angleDegree);

        // The device might has pitch angle
        double [][] rotaMatrixPitch = MyQuaternion.getRotationMatrixByPitch(eulerAngle[1]);

        // update mDataAll
        // raw sensor data: x: go-right, y: go-down,    z: go-forward
        // world frame:     x: go-right, y: go-forward, z: go-up
        mDataAllLen = mPointCloudProcess.rotationRollPitchCorrect(mDataAll, mPointCloudBuffer,
                rotaMatrixRoll, rotaMatrixPitch);
        if (0 == mDataAllLen) {
            mObstaclePoint = null;
            return false;
        }

        mDataAllLen = mPointCloudProcess.downsampleByRange(
                mDataAll, mDataAllLen, xMin, xMax, zMin, zMax, yMin, yMax);
        if (0 == mDataAllLen) {
        	mObstaclePoint = null;
        	return false;
        }

        // Projection to z, project to floor plane
        // use the Array with len, so don't need to re-allocate the array
        mMapLocal.updating(mDataAll, mDataAllLen);
        // each point: x, y, z (height)
        float [] obstaclePoints = mMapLocal.getObstaclePoints();
        int dataPointsLen = obstaclePoints.length;
        
        // local obstacle points process
        mDataPointsLocal = new float[dataPointsLen];
        System.arraycopy(obstaclePoints, 0, mDataPointsLocal, 0, dataPointsLen);
        mObstacleBoxesLocal = mMapLocal.getObstacleBoxes();

        float [] obs = mMapLocal.getCloestPoint();
        
        // Projection to y, like: get in front vertical plane
        mMapLocalVertical.updating(mDataAll, mDataAllLen);
        float [] pV = mMapLocalVertical.getObstaclePoints();
        float [] mDataPointsLocalVertical = new float[pV.length];
        System.arraycopy(pV, 0, mDataPointsLocalVertical, 0, pV.length);
        mObstacleBoxesLocalVertical = mMapLocalVertical.getObstacleBoxes();

        float [] obsY = mMapLocalVertical.getPointByXExtra(obs[0]);

        mObstaclePoint = new float[4];
        mObstaclePoint [0] = obs[0];  // x
        mObstaclePoint [1] = obs[1];  // y
        if (null != obsY) {		      // z: height
        	mObstaclePoint [2] = obsY[1]; 
        } else {
        	mObstaclePoint [2] = -2f;
        }
        mObstaclePoint [3] = obs[2];  // angle (left -> right: 0 -> 180)

        int obstacleBoxNum = 0;
        if (false == isGlobalLocalized) {
            // get region boxes (2 corner points) of these points in local 2d metric
            // mObstacleBoxesLocal = mMapLocal.getObstacleBoxes(); // move to above, out of if()
            /*if (null != mObstacleBoxesLocal) {
                for (int i = 0; i < mObstacleBoxesLocal.length; i += 4) {
                    info = String.format("region %d: corner points [%.2f, %.2f] [%.2f, %.2f]",
                            (int)(i/4)+1,
                            mObstacleBoxesLocal[i+0], mObstacleBoxesLocal[i+1],
                            mObstacleBoxesLocal[i+2], mObstacleBoxesLocal[i+3]);
                    Log.d(TAG, info);
                }
            }*/
            if (null != mObstacleBoxesLocal) {
                if (mObstacleBoxesLocal.length > 0) {
                    findObstacle = true;
                }
            }
        } else {
            // local metric -> global metric
            obstaclePoints = localMetric2GlobalMetric(obstaclePoints, dataPointsLen);
            // global metric -> global byte grid
            obstaclePoints = globalMetric2Pixel(obstaclePoints, dataPointsLen);
            mDataPointsInt = mMapGlobal.getObstaclePoints(obstaclePoints, dataPointsLen);
            
            /*mUtils.saveOneFrameData(obstaclePoints, obstaclePoints.length,
            		2, String.valueOf(mFramesCnt) + "_prjZ.txt");
            mUtils.saveOneFrameData(mObstacleBoxesInt, mObstacleBoxesInt.length,
            		2, String.valueOf(mFramesCnt) + "_prjZObj.txt");*/
            
            if (null != mDataPointsInt) {
                if (mDataPointsInt.length > mPointsMinimum) {
                    findObstacle = true;
                }
            }
            
            mObstacleBoxesInt = mMapGlobal.getObstacleBoxes();
            if (null != mObstacleBoxesInt) {
                if (mObstacleBoxesInt.length > 0) {
                    obstacleBoxNum = mObstacleBoxesInt.length / 4;
                }
            }
        }

        /*// TODO comment out
        mDetectingCnt += 1;
        long costTimeMs = 0;
        if (0 != obstacleBoxNum) {
            mDetectingObstacleCnt += 1;
            costTimeMs = System.currentTimeMillis() - startTime;
            mCostTimeSumAll += costTimeMs;
        } else {
            costTimeMs = System.currentTimeMillis() - startTime;
        }

        if (1 == mDetectingCnt%10) {
            if (0 != mDetectingObstacleCnt) {
                info = String.format("[%d]: detect %d obstacle boxes, costs %d ms, average %d ms",
                        mDetectingCnt, obstacleBoxNum, costTimeMs, (int) (mCostTimeSumAll / mDetectingObstacleCnt));
                LOG.info(info);
            }
        }*/

        return findObstacle;
    }

    public int localObstacleAvoidance() {
        int obstacleFreeDir = -1;
        if (null == mDataPointsLocal) {
            return -1;
        }
        int len = mDataPointsLocal.length;
        float xWide = xMax - xMin;
        final int sectionNum = 3;
        float x1 = xMin + xWide/sectionNum;
        float x2 = xMin + xWide/sectionNum*2;
        int leftPointCnt   = 0;
        int middlePointCnt = 0;
        int rightPointCnt  = 0;

        // histogram statistic of all sections
        for (int i = 0; i < len; i+=2) {
            float x = mDataPointsLocal[i + 0];
            if (x >= xMin && x < x1) {
                leftPointCnt++;
            } else if (x >= x1 && x < x2) {
                middlePointCnt++;
            } else if (x >= x2 && x <= xMax) {
                rightPointCnt++;
            }
        }

        int minValue = Math.min(leftPointCnt, middlePointCnt);
        minValue = Math.min(minValue, rightPointCnt);

        len = len/2;
        if ((rightPointCnt > len/6)
            && (rightPointCnt > len/6)
            && (middlePointCnt > len/6)) {
            obstacleFreeDir = 0; // obstacle in front
        }  else if (minValue == leftPointCnt) {
            obstacleFreeDir = 135; // obstacle-free is left
        } else if (minValue == rightPointCnt) {
            obstacleFreeDir = 45; // obstacle-free is right
        } else if (minValue == middlePointCnt) {
            obstacleFreeDir = -1; // obstacle-free is middle, safe
        }
        /*String info = String.format("left %d, middle %d, right %d", leftPointCnt, middlePointCnt, rightPointCnt);
        Log.w(TAG, info);*/

        mDataPointsLocal = null;
        return obstacleFreeDir;
    }

    private float [] localMetric2GlobalMetric(double [][] rotaMat, float [] trans, float [] local, int len) {
        double r00 = rotaMat[0][0];
        double r01 = rotaMat[0][1];
        double r02 = rotaMat[0][2];
        double r10 = rotaMat[1][0];
        double r11 = rotaMat[1][1];
        double r12 = rotaMat[1][2];

        // translation vector in tango frame
        float posex = trans[0];
        float posey = trans[1];

        float [] global = new float[len];

        for (int c = 0; c < len - 2; c += 2) {
            float px = local[c*2 + 0];
            float py = local[c*2 + 1];
            float pz = 0;

            global[c*2 + 0] = (float) (r00 * px + r01 * py + r02 * pz + posex);
            global[c*2 + 1] = (float) (r10 * px + r11 * py + r12 * pz + posey);
        }
        return global;
    }

    private int [] globalMetric2GlobalGrid(float [] global) {
        if (null == global) {
            return null;
        }
        int len = global.length;
        float [] m = mMetric2GridTransform;

        int [] grid = new int[len];
        
        if (TRANSFORM_BY_SCALE_OFFSET == m.length) {
            for (int c = 0; c < len - 2; c++) {
                grid[c * 2 + 0] = (int) (global[c * 2 + 0] * m[0] + m[1]);
                grid[c * 2 + 1] = (int) (global[c * 2 + 1] * m[0] + m[2]);
            }
        } else if (TRANSFORM_BY_MATRIX == m.length) {
            float m00 = m[0];
            float m01 = m[1];
            float m02 = m[2];
            float m10 = m[3];
            float m11 = m[4];
            float m12 = m[5];
            for (int c = 0; c < len - 2; c++) {
                float x = global[c * 2 + 0];
                float y = global[c * 2 + 1];
                float z = 1;
                grid[c * 2 + 0] = (int)(m00 * x + m01 * y + m02 * z);
                grid[c * 2 + 1] = (int)(m10 * x + m11 * y + m12 * z);
            }
        }

        return grid;
    }
    
}

