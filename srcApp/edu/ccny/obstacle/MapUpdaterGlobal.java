package edu.ccny.obstacle;

import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

public class MapUpdaterGlobal {
    private static final String TAG = MapUpdaterGlobal.class.getSimpleName();
    private final byte BITMAP_OCCUPIED = (byte) 0x00;
    private final byte BITMAP_FREE     = (byte) 0xFF;
    private final byte GRID_OCCUPIED   = (byte) 0x00;
    private final byte GRID_FREE       = (byte) 0xFF;
    private int [] mObstacles = null;
    private ConnectComponent mConnectComponent = new ConnectComponent();
    public byte [][] mKnownGridMap;
    private int mWallThresholdPixel = -1; // need config from constructor
	// LOG.setLevel(Level.WARNING), LOG.severe/warning/info/finest
    private static Logger LOG = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);
    
    public MapUpdaterGlobal(byte [][] map, int threshold) {
        int gridHeight = map.length;
        int gridWidth  = map[0].length;
        LOG.setLevel(Level.WARNING);
        
        byte [][] mapXYSwitch = new byte[gridWidth][gridHeight];
        for (int j = 0; j < gridHeight; j++)
            for (int i = 0; i < gridWidth; i++) {
                mapXYSwitch[i][j] = map[j][i];
            }
        map = mapXYSwitch;
        gridHeight = map.length;
        gridWidth  = map[0].length;

        mKnownGridMap = map;
        mWallThresholdPixel = threshold;
        int cnt_BITMAP_OCCUPIED = 0;
        int cnt_BITMAP_FREE     = 0;
        int cnt_BITMAP_OTHERS   = 0;
        for (int j = 0; j < gridHeight; j++)
            for (int i = 0; i < gridWidth; i++) {
                if (BITMAP_OCCUPIED == mKnownGridMap[j][i]) {
                    cnt_BITMAP_OCCUPIED += 1;
                    mKnownGridMap[j][i] = GRID_OCCUPIED;
                } else if (BITMAP_FREE == mKnownGridMap[j][i]) {
                    cnt_BITMAP_FREE += 1;
                    mKnownGridMap[j][i] = GRID_FREE;
                } else {
                    cnt_BITMAP_OTHERS += 1;
                    mKnownGridMap[j][i] = GRID_FREE;
                }
            }

        /*String info = String.format("knowGridMap [%d x %d] = %d: All %d = Occupied %d, Free %d, Others %d",
                gridHeight, gridWidth, gridHeight * gridWidth,
                cnt_BITMAP_OCCUPIED + cnt_BITMAP_FREE + cnt_BITMAP_OTHERS,
                cnt_BITMAP_OCCUPIED, cnt_BITMAP_FREE, cnt_BITMAP_OTHERS);
        LOG.info(info);*/
    }

    public int [] getObstaclePoints(float [] points, int size) {
        int [] objectGrids = new int[size];
        if (null == points) {
            return null;
        }
        for (int c = 0; c < size;  c++) {
            objectGrids[c] = (int) points[c];
        }

        // mObstacles = objectGrids;
        mObstacles = disregardPointsInWalls(objectGrids);
        return mObstacles;
    }

    public int [] getObstacleBoxes() {
        if (null == mObstacles) {
            return null;
        }
        if (0 == mObstacles.length) {
            return null;
        }

        int iMin = 10000000;
        int iMax = -1000;
        int jMin = 10000000;
        int jMax = -1000;
        for (int c = 0; c < mObstacles.length; c+=2) {
            int i = mObstacles[c + 0];
            int j = mObstacles[c + 1];
            if (i < iMin) iMin = i;
            if (i > iMax) iMax = i;
            if (j < jMin) jMin = j;
            if (j > jMax) jMax = j;
        }

        int width  = (iMax - iMin) + 1;
        int height = (jMax - jMin) + 1;
        byte [][] map = new byte[height][width];
        for (int j = 0; j < height; j++)
            for (int i = 0; i < width; i++) {
                map[j][i] = 0;
            }

        int obstacleCnt = 0;
        for (int c = 0; c < mObstacles.length; c+=2) {
            int i = mObstacles[c + 0];
            int j = mObstacles[c + 1];
            i = i - iMin;
            j = j - jMin;
            map[j][i] = 1;
            obstacleCnt += 1;
        }
        boolean zeroAsBg = true;
        /*String info = String.format("Range [%d, %d] [%d, %d] Buffer len %d, Point count %d",
                iMin, jMin, iMax, jMax, mObstacles.length, obstacleCnt);
        Log.w(TAG, info);*/

        int [][] boxes = mConnectComponent.findRegionsBoxij(map, zeroAsBg);
        int regionNum = boxes.length;
        if (0 == regionNum) {
            return null;
        }
        int [] boxesPixel = new int[regionNum * 4];

        for (int region = 0; region < regionNum; region++) {
            for (int m = 0; m < 4; m += 2) {
                int i = boxes[region][m + 0];
                int j = boxes[region][m + 1];
                boxesPixel[region*4 + m + 0] = i + iMin;
                boxesPixel[region*4 + m + 1] = j + jMin;
            }
        }

        return boxesPixel;
    }

    private int [] disregardPointsInWalls(int [] objects) {
        int len = objects.length;
        int i;
        int j;
        int c;

        if (null == objects) {
            return null;
        }
        ArrayList<Integer> grids = new ArrayList<Integer>();

        int totalPoints   = len / 2;
        int wallPoints    = 0;
        int nonwallPoints = 0;
        for (c = 0; c < len; c += 2) {
            i = objects[c + 0];
            j = objects[c + 1];
            if (true != isWall(i, j)) {
                grids.add(i);
                grids.add(j);
                nonwallPoints += 1;
            } else {
                wallPoints += 1;
            }
        }
        /*String info = String.format("total points %d, wall %d, non wall %d",
                totalPoints, wallPoints, nonwallPoints);
        Log.d(TAG, info);*/

        len = grids.size();
        if (0 == len) {
            return null;
        }

        int [] obstacleGrids = new int[len];
        for (c = 0; c < len; c += 2) {
            obstacleGrids[c + 0] = grids.get(c + 0);
            obstacleGrids[c + 1] = grids.get(c + 1);
        }

        return obstacleGrids;
    }

    private boolean isWall(int i, int j) {
        int gridHeight = mKnownGridMap.length;
        int gridWidth  = mKnownGridMap[0].length;

        int iLeft  = i - mWallThresholdPixel;
        int iRight = i + mWallThresholdPixel;
        int jLeft  = j - mWallThresholdPixel;
        int jRight = j + mWallThresholdPixel;
        int iMin = Math.max(iLeft, 0);
        int iMax = Math.min(iRight, gridWidth);
        int jMin = Math.max(jLeft, 0);
        int jMax = Math.min(jRight, gridHeight);

        for (int n = jMin; n < jMax; n++)
            for (int m = iMin; m < iMax; m++) {
                // n --- j, m --- i
                if (GRID_OCCUPIED == mKnownGridMap[n][m]) {
                    return true;
                }
            }

        return false;
    }

}

