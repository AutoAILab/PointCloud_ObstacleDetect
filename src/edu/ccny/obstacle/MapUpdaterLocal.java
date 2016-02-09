package edu.ccny.obstacle;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.logging.Level;
import java.util.logging.Logger;

public class MapUpdaterLocal {
    // TODO: figure out the best threshold
    private final byte mThresholdCnt = 6;

    private static final int X = 0;
    private static final int Y = 1;
    private static final int Z = 2;

    // 3D -> 2D
    private float xMin = 0;
    private float xMax = 0;
    private float yMin = 0;
    private float yMax = 0;
    private float zMin = 0;
    private float zMax = 0;
    private float mMetricPerPixel = 0;
    private int   mProjection = Z; // default, project to z;

    private final byte UNOCCUPIED = 0;
    private final byte OCCUPIED   = 1;

    private int   mMapOccupiedCnt = 0;
    private  byte mMap [][] = null;
    private int   mMapCnt [][] = null;
    private int   xSize = 0;
    private int   ySize = 0;
    private int   zSize = 0;
    private ConnectComponent mConnectComponent = new ConnectComponent();
	// LOG.setLevel(Level.WARNING), LOG.severe/warning/info/finest
    private static Logger LOG = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);

    private int   iSize = 0;
    private float iMin  = 0;
    private float iMax  = 0;
    
    private int   jSize = 0;
    private float jMin  = 0;
    private float jMax  = 0;
 
    public MapUpdaterLocal(float x1, float x2, float y1, float y2,
                           float z1, float z2, float metricPerPixel_para, int projection) {
        xMin = x1;
        xMax = x2;
        yMin = y1;
        yMax = y2;
        zMin = z1;
        zMax = z2;
        mMetricPerPixel = metricPerPixel_para;

    	LOG.setLevel(Level.INFO);

        mProjection = projection;
        if (Z != mProjection && Y != mProjection) {
        	LOG.severe("Project not supoort yet for: " + mProjection);
        	return;
        }
        
        init();
    }

    private void init() {
        xSize = (int)((xMax - xMin) / mMetricPerPixel);
        ySize = (int)((yMax - yMin) / mMetricPerPixel);
        zSize = (int)((zMax - zMin) / mMetricPerPixel);

        iSize = xSize;
        iMin  = xMin;
        iMax  = xMax;
        
        // default, project to z;
        jSize = ySize;
        jMin  = yMin;
        jMax  = yMax;
        
        if (Z == mProjection) {
        	LOG.info("Project to: Z");
        } else if (Y == mProjection) {
        	LOG.info("Project to: Y");
        	jMin  = zMin;
        	jMax  = zMax;
        	jSize = zSize;
        }
        
        mMap = new byte[jSize][iSize];
        mMapCnt = new int[jSize][iSize];
       
        String info = String.format("MapUpdaterLocal(): localGranularityMeter = %.2f", mMetricPerPixel);
        LOG.info(info);
        info = String.format("MapUpdaterLocal(): i: %.2f ~ %.2f, j: %.2f ~ %.2f", iMin, iMax, jMin, jMax);
        LOG.info(info);
        info = String.format("MapUpdaterLocal(): local 2D grid i-j: %d-%d", iSize, jSize);
        LOG.info(info);
    }

    public float [] getObstacleBoxes() {
        boolean zeroAsBg = true;

        if (Z != mProjection && Y != mProjection) {
        	LOG.severe("Project not supoort yet for: " + mProjection);
        	return null;
        }
        
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
    
    private float [] getPolar (float p[], float jMin) {
    	float [] pPolar = new float[2];
    	
    	float jOffset = jMin - 1.0f;
    	p[1] -= jOffset;
    	double degree = Math.atan2((p[1]), p[0]) * 180 / Math.PI;
    	if (degree < 0) {
    		degree += 180;
    	}
    	double dis = Math.sqrt(p[0]*p[0]+p[1]*p[1]);
    	
    	pPolar[0] = (float)degree;
    	pPolar[1] = (float)dis;
    	return pPolar;
    }
    
    public float [] getObstaclePolarHis() {
    	final int WIDTH = 121;
    	float [] obstacleFront = new float [WIDTH];
    	
    	for (int i = 0; i < WIDTH; i++) {
    		obstacleFront[i] = 0;
    	}
    	
    	float jMin = 1000;
    	for (int i = 0; i < iSize; i++)
            for (int j = 0; j < jSize; j++) {
                if (OCCUPIED == mMap[j][i]) {
                	float p[] = getGridLocalMetric(i, j);
                	if (p[1] < jMin) {
                		jMin = p[1];
                	}
                }
            }
    	
    	for (int i = 0; i < iSize; i++)
            for (int j = 0; j < jSize; j++) {
                if (OCCUPIED == mMap[j][i]) {
                	float p[] = getGridLocalMetric(i, j);
                	
                	float pPolar[] = getPolar(p, jMin);
                	
                	int thetaIndex = (int)(pPolar[0] - 30);
                	if (thetaIndex < 0 || thetaIndex > 120) {
                		continue;
                	}
                	// the far, the smaller;
                	float dis = pPolar[1];
                	if (dis < 0.5f) {
                		dis = 0.5f;
                	}
                	obstacleFront[thetaIndex] += 1/(pPolar[1]*pPolar[1]*pPolar[1]);
                }
            }
    	
    	return obstacleFront;
    }

    public float [] getObstaclePointsPolarFilter(float [] points) {
        ArrayList<Float> pointsList = new ArrayList<Float>();

        int len = points.length;
        if (0 == len || 0 != len % 2) {
            return null;
        }

        for (int i = 0; i < len; i+=2) {
            float p0 = points[i + 0];
            float p1 = points[i + 1];

            double degree = Math.atan2(p1, p0) * 180 / Math.PI;
            if (degree < 0) {
                degree += 180;
            }
            if (degree >= 70 && degree <= 130) {
                pointsList.add(p0);
                pointsList.add(p1);
            }
        }

        len = pointsList.size();
        float [] pointsFilter = new float[len];
        for (int i = 0; i < len; i++) {
            pointsFilter[i] = pointsList.get(i);
        }

        return pointsFilter;
    }

    public float [] getObstaclePoints() {
        int cnt = 0;
        int dim = 2;
        float [] obstaclePoints = new float[mMapOccupiedCnt * dim];

        if (Z != mProjection && Y != mProjection) {
        	LOG.severe("Project not supoort yet for: " + mProjection);
        	return null;
        }
        
        for (int i = 0; i < iSize; i++)
            for (int j = 0; j < jSize; j++) {
                if (OCCUPIED == mMap[j][i]) {
                	float p[] = getGridLocalMetric(i, j);
                	obstaclePoints[cnt + 0] = p[0];
                	obstaclePoints[cnt + 1] = p[1];
                    cnt += dim;
                    if (cnt >= mMapOccupiedCnt * 3) {
                        return obstaclePoints;
                    }
                }
            }

        return obstaclePoints;
    }

    private float [] getGridLocalMetric(int i, int j) {
        float [] pointLocalMetric = new float[2];
        pointLocalMetric[0] = iMin + i * mMetricPerPixel + (mMetricPerPixel/2.0f);
        pointLocalMetric[1] = jMin + j * mMetricPerPixel + (mMetricPerPixel/2.0f);
        return pointLocalMetric;
    }

    private void mapResetClear() {
        // reset local mMap
        for (int i = 0; i < iSize; i++)
            for (int j = 0; j < jSize; j++) {
                mMap[j][i] = UNOCCUPIED;
                mMapCnt[j][i] = 0;
            }
    }

    public boolean updating(float [] points, int len) {
        float x, y, z;
        int i, j, k;

        if (Z != mProjection && Y != mProjection) {
        	LOG.severe("Project not supoort yet for: " + mProjection);
        	return false;
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
            if (i < 0 || i >= xSize || j < 0 || j >= ySize || k < 0 || k >= zSize) {
            	// LOG.warning("i, j, or k is invalid");
                continue;
            }
            
            if (Z == mProjection) {
            	// j still is j
            } else if (Y == mProjection) {
            	j = k;
            }
            
            mMapCnt[j][i] += 1;
        }

        for (i = 0; i < iSize; i++)
            for (j = 0; j < jSize; j++) {
                if (mMapCnt[j][i] >= mThresholdCnt) {
                    mMap[j][i] = OCCUPIED;
                } else {
                    mMap[j][i] = UNOCCUPIED;
                }
            }

        // mMap = morphologyDiat(mMap);
        
        mMapOccupiedCnt = 0;
        for (i = 0; i < iSize; i++)
            for (j = 0; j < jSize; j++) {
                if (OCCUPIED == mMap[j][i]) {
                    mMapOccupiedCnt += 1;
                }
            }
        return true;
    }

    private byte [][] morphologyDiat(byte [][] m) {
    	int i, j;
    	
    	byte[][] mOut = new byte[m.length][];
    	    for (i = 0; i < m.length; i++) {
    	    	mOut[i] = Arrays.copyOf(m[i], m[i].length);
    	    }

    	for (i = 1; i < iSize - 1; i++)
            for (j = 1; j < jSize - 1; j++) {
            	if (OCCUPIED == m[j][i]) {
            		mOut[j][i - 1]     = OCCUPIED;
            		mOut[j][i + 1]     = OCCUPIED;
            		mOut[j - 1][i]     = OCCUPIED;
            		mOut[j + 1][i] 	   = OCCUPIED;
            		/*mOut[j - 1][i - 1] = OCCUPIED;
            		mOut[j + 1][i - 1] = OCCUPIED;
            		mOut[j - 1][i + 1] = OCCUPIED;
            		mOut[j + 1][i + 1] = OCCUPIED;*/
            	}
            }
        
    	return mOut;
    }

    public float [] getClosestPoint(float [] points) {
        int i, j;
        double disMin = 1000000;
        float [] thePoint = null;

        if (null == points) {
            return null;
        }

        int len = points.length;

        for (i = 0; i < len; i+=2) {
            float [] p = new float[2];
            p[0] = points[i+0];
            p[1] = points[i+1];
            double dis = Math.sqrt(p[0]*p[0] + p[1]*p[1]);
            if (dis < disMin) {
                thePoint = p;
                disMin = dis;
            }
        }

        if (null == thePoint) {
            return null;
        }

        double degree = Math.atan2(thePoint[1], thePoint[0]) * 180 / Math.PI;
        if (degree < 0) {
            degree += 180;
        }

        float [] xytheta = new float [3];
        xytheta[0] = thePoint[0];
        xytheta[1] = thePoint[1];
        xytheta[2] = (float) degree;
        return xytheta;
    }

    public float [] getClosestPoint() {
    	int i, j;
    	double disMin = 1000000;
    	float [] thePoint = null;
    	
    	for (i = 0; i < iSize; i++)
            for (j = 0; j < jSize; j++) {
                if (OCCUPIED == mMap[j][i]) {
                	float [] p = getGridLocalMetric(i, j);
                    double dis = Math.sqrt(p[0]*p[0] + p[1]*p[1]);
                    if (dis < disMin) {
                    	thePoint = p;
                    	disMin = dis;
                    }
                }
            }
    	if (null == thePoint) {
    		return null;
    	}
    	
    	double degree = Math.atan2(thePoint[1], thePoint[0]) * 180 / Math.PI;
    	if (degree < 0) {
    		degree += 180;
    	}
    	
    	float [] xytheta = new float [3];
    	xytheta[0] = thePoint[0];
    	xytheta[1] = thePoint[1];
    	xytheta[2] = (float) degree;
    	return xytheta;
    }

    public float [] getPointByXExtra(float x) {
        int i, j;
        float [] pOut = new float [2];

        float [] pL = getPointByX(x - mMetricPerPixel);
        float yL;
        if (null == pL) {
        	yL = -1000;
        } else {
        	yL = pL[1];
        }
        
        float [] p  = getPointByX(x);
        float y;
        if (null == p) {
        	y = -1000;
        } else {
        	y = p[1];
        }
        
        float [] pR = getPointByX(x + mMetricPerPixel);
        float yR;
        if (null == pR) {
        	yR = -1000;
        } else {
        	yR = pR[1];
        }
        
        float height = Math.max(yL, y);
        height = Math.max(height, yR);
        
        if (height > -100) {
        	pOut[0] = x;
        	pOut[1] = height;
        	return pOut;
        }

        return null;
    }
    
    private float [] getPointByX(float x) {
        int i, j;

        i = (int)((x - iMin) / mMetricPerPixel);
        if (i < 0 || i >= iSize) {
            return null;
        }
        for (j = jSize - 1; j >= 0; j--) {
            if (OCCUPIED == mMap[j][i]) {
            	return getGridLocalMetric(i, j);
            }
        }

        return null;
    }
    
    /*public float [] getObstaclePoints3d(int len []) {
        float x, y, z;
        int i, j, k;

        if (false == mSupport3dFlag) {
            return null;
        }
        
        int cnt = 0;
        for (i = 0; i < xSize; i++)
            for (j = 0; j < ySize; j++)
                for (k = 0; k < zSize; k++) {
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
        
        for (j = 0; j < ySize; j++)
            for (i = 0; i < xSize; i++)
                for (k = zSize - 1; k >= 0; k--) {
                    if (OCCUPIED == mMap3d[k][j][i]) {
                        xyz[0] = i * mMetricPerPixel + xMin + mMetricPerPixelHalf;
                        xyz[1] = j * mMetricPerPixel + yMin + mMetricPerPixelHalf;
                        xyz[2] = k * mMetricPerPixel + zMin + mMetricPerPixelHalf;
                        return true;
                    }
                }

        return false;
    }*/

}

