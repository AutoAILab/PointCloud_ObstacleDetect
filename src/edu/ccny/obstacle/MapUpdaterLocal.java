package edu.ccny.obstacle;

import java.util.logging.Level;
import java.util.logging.Logger;

public class MapUpdaterLocal {
    private static final String TAG = MapUpdaterLocal.class.getSimpleName();
    // our mMap is in world frame:  x: go-right, y: go-forward, z: go-up
    private float xMin = -0.4f;
    private float xMax =  0.4f;
    private float yMin =  0.5f;
    private float yMax =  1.5f;
    private float zMin = -0.8f;
    private float zMax =  0.5f;
    private float mMetricPerPixel = 0.02f; // 0.2f;

    // TODO: figure out the best threshold
    private final byte mThresholdCnt   = 12;
    private final byte mThresholdCnt3d = 2;

    private final byte UNOCCUPIED = 0;
    private final byte OCCUPIED   = 1;
    private final boolean mSupport3dFlag = true;

    private int   mMapOccupiedCnt = 0;
    private  byte mMap [][] = null;
    private int   mMapCnt [][] = null;
    private  byte mMap3d [][][] = null;
    private int   mMap3dCnt [][][] = null;
    private int   iSize = 0;
    private int   jSize = 0;
    private int   kSize = 0;
    private float mMetricPerPixelHalf = 0;
    private float [] mObjectsPoints;
    private ConnectComponent mConnectComponent = new ConnectComponent();
	// LOG.setLevel(Level.WARNING), LOG.severe/warning/info/finest
    private static Logger LOG = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);
    
    public MapUpdaterLocal() {
    	LOG.setLevel(Level.INFO);
        init();
    }

    public MapUpdaterLocal(float x1, float x2, float y1, float y2,
                           float z1, float z2, float metricPerPixel_para) {
        xMin = x1;
        xMax = x2;
        yMin = y1;
        yMax = y2;
        zMin = z1;
        zMax = z2;
        mMetricPerPixel = metricPerPixel_para;

        init();
    }

    private void init() {
        iSize = (int)((xMax - xMin) / mMetricPerPixel);
        jSize = (int)((yMax - yMin) / mMetricPerPixel);
        kSize = (int)((zMax - zMin) / mMetricPerPixel);
        mMetricPerPixelHalf = mMetricPerPixel/2.0f;
        mMap = new byte[jSize][iSize];
        mMapCnt = new int[jSize][iSize];
        if (true == mSupport3dFlag) {
            mMap3d = new byte[kSize][jSize][iSize];
            mMap3dCnt = new int[kSize][jSize][iSize];
            mObjectsPoints = new float[kSize*jSize*iSize*3];
        } else {
            mObjectsPoints = new float[jSize*iSize*2];
        }

        String info = String.format("MapUpdaterLocal(): localGranularityMeter = %.2f", mMetricPerPixel);
        LOG.info(info);
        info = String.format("MapUpdaterLocal(): left ~ right: %.2f ~ %.2f, forward: %.2f ~ %.2f, down ~ up: %.2f ~ %.2f",
                xMin, xMax, yMin, yMax, zMin, zMax);
        LOG.info(info);
        info = String.format("MapUpdaterLocal(): local 3D grid x-y-z %d-%d-%d", iSize, jSize, kSize);
        LOG.info(info);
    }

    public float [] getObstacleBoxes() {
        boolean zeroAsBg = true;

        int [][] boxes = mConnectComponent.findRegionsBoxij(mMap, zeroAsBg);
        int regionNum = boxes.length;
        if (0 == regionNum) {
            return null;
        }
        float [] boxesMetric = new float[regionNum * 4];

        for (int region = 0; region < regionNum; region++) {
            for (int m = 0; m < 4; m += 2) {
                int i = boxes[region][m + 0];
                int j = boxes[region][m + 1];
                float[] xy = getGridLocalMetric(i, j);
                boxesMetric[region*4 + m + 0] = xy[0];
                boxesMetric[region*4 + m + 1] = xy[1];
            }
        }

        return boxesMetric;
    }
    
    public float [] getObstaclePoints() {
        float x, y, z;
        int i, j, k;

        float [] obstaclePointsWithHeight = new float[mMapOccupiedCnt*3];

        int Dim = 3;
        int cnt = 0;
        for (i = 0; i < iSize; i++)
            for (j = 0; j < jSize; j++) {
                if (OCCUPIED == mMap[j][i]) {
                    z = 0;
                    for (k = kSize - 1; k >= 0; k--) {
                        if (OCCUPIED == mMap3d[k][j][i]) {
                            z = k * mMetricPerPixel + zMin + mMetricPerPixelHalf;
                            break;
                        }
                    }
                    x = i * mMetricPerPixel + xMin + mMetricPerPixelHalf;
                    y = j * mMetricPerPixel + yMin + mMetricPerPixelHalf;
                    obstaclePointsWithHeight[cnt + 0] = x;
                    obstaclePointsWithHeight[cnt + 1] = y;
                    obstaclePointsWithHeight[cnt + 2] = z;
                    cnt += Dim;
                    if (cnt >= mMapOccupiedCnt * 3) {
                        return obstaclePointsWithHeight;
                    }
                }
            }

        return obstaclePointsWithHeight;
    }

    public float [] getObstaclePoints3d(int len []) {
        float x, y, z;
        int i, j, k;

        if (false == mSupport3dFlag) {
            return null;
        }
        
        int cnt = 0;
        for (i = 0; i < iSize; i++)
            for (j = 0; j < jSize; j++)
                for (k = 0; k < kSize; k++) {
                    if (OCCUPIED == mMap3d[k][j][i]) {
                        x = i * mMetricPerPixel + xMin + mMetricPerPixelHalf;
                        y = j * mMetricPerPixel + yMin + mMetricPerPixelHalf;
                        z = k * mMetricPerPixel + zMin + mMetricPerPixelHalf;
                        mObjectsPoints[cnt*3 + 0] = x;
                        mObjectsPoints[cnt*3 + 1] = y;
                        mObjectsPoints[cnt*3 + 2] = z;
                        cnt += 1;
                    }
            }
        len[0] = cnt*3;
        return mObjectsPoints;
    }

    public boolean getHeightValue(float x, float y, float [] z) {
        int i, j, k;
        if (null == z) {
            return false;
        }
        if (1 != z.length) {
            return false;
        }

        if (false == mSupport3dFlag) {
            return false;
        }
        
        i = (int)((x - xMin) / mMetricPerPixel);
        j = (int)((y - yMin) / mMetricPerPixel);
        if (i < 0 || i >= iSize || j < 0 || j >= jSize) {
            return false;
        }
        for (k = kSize - 1; k >= 0; k--) {
            if (OCCUPIED == mMap3d[k][j][i]) {
                z[0] = k * mMetricPerPixel + zMin + mMetricPerPixelHalf;
                return true;
            }
        }

        return false;
    }

    public float [] getGridLocalMetric(int i, int j) {
        float [] pointLocalMetric = new float[2];
        pointLocalMetric[0] = i * mMetricPerPixel + xMin + mMetricPerPixelHalf;
        pointLocalMetric[1] = j * mMetricPerPixel + yMin + mMetricPerPixelHalf;
        return pointLocalMetric;
    }

    public boolean getNearestPoint(float [] xyz) {
        int i, j, k;
        if (null == xyz) {
            return false;
        }
        if (3 != xyz.length) {
            return false;
        }

        if (false == mSupport3dFlag) {
            return false;
        }
        
        for (j = 0; j < jSize; j++)
            for (i = 0; i < iSize; i++)
                for (k = kSize - 1; k >= 0; k--) {
                    if (OCCUPIED == mMap3d[k][j][i]) {
                        xyz[0] = i * mMetricPerPixel + xMin + mMetricPerPixelHalf;
                        xyz[1] = j * mMetricPerPixel + yMin + mMetricPerPixelHalf;
                        xyz[2] = k * mMetricPerPixel + zMin + mMetricPerPixelHalf;
                        return true;
                    }
                }

        return false;
    }

    public void mapResetClear() {
        int i, j, k;

        // reset local mMap
        for (j = 0; j < jSize; j++)
            for (i = 0; i < iSize; i++) {
                mMap[j][i] = UNOCCUPIED;
                mMapCnt[j][i] = 0;
                if (true == mSupport3dFlag) {
                    for (k = 0; k < kSize; k++) {
                        mMap3d[k][j][i] = UNOCCUPIED;
                        mMap3dCnt[k][j][i] = 0;
                    }
                }
            }
    }

    public void updating(float [] points, int len) {
        final int X = 0;
        final int Y = 1;
        final int Z = 2;
        float x, y, z;
        int i, j, k;

        if (len < 3) {
            // Log.d(TAG, "updating() len < 3");
            return;
        } else if (points.length < len) {
        	LOG.warning("updating() points.length < len");
            return;
        }
        if (len % 3 != 0) {
            len = ((int) (len / 3)) * 3;
        }

        mapResetClear();

        for (int c = 0; c < len - 3; c +=3) {
            x = points[c + X];
            y = points[c + Y];
            z = points[c + Z];

            if (x < xMin || x > xMax || y < yMin || y > yMax || z < zMin || z > zMax) {
                continue;
            }

            i = (int)((x - xMin) / mMetricPerPixel);
            j = (int)((y - yMin) / mMetricPerPixel);
            k = (int)((z - zMin) / mMetricPerPixel);
            if (i < 0 || i >= iSize || j < 0 || j >= jSize || k < 0 || k >= kSize) {
                // Log.w(TAG, "i, j, k is invalid");
                continue;
            }
            if (true == mSupport3dFlag) {
                mMap3dCnt[k][j][i] += 1;
            }
            mMapCnt[j][i] += 1;
        }

        updatingPixel();
    }

    private void updatingPixel() {
        int i, j, k;
        mMapOccupiedCnt = 0;
        for (i = 0; i < iSize; i++)
            for (j = 0; j < jSize; j++) {
                if (mMapCnt[j][i] >= mThresholdCnt) {
                    mMap[j][i] = OCCUPIED;
                    mMapOccupiedCnt += 1;
                } else {
                    mMap[j][i] = UNOCCUPIED;
                }
                if (true == mSupport3dFlag) {
                    for (k = 0; k < kSize; k++) {
                        if (mMap3dCnt[k][j][i] >= mThresholdCnt3d) {
                            mMap3d[k][j][i] = OCCUPIED;
                        } else {
                            mMap3d[k][j][i] = UNOCCUPIED;
                        }
                    }
                }
            }
    }

}

