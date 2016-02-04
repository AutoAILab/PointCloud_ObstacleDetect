package edu.ccny.obstacle;

import java.util.ArrayList;

public class ConnectComponent
{
    private static final String TAG = ConnectComponent.class.getSimpleName();
    final int MAX_LABEL_NUM= 1000;
    int mNextLabel = 1;

    public int getMaxLabelNumber() {
        return mNextLabel;
    }

    public int [][] findRegionsBoxij(byte [][] map, boolean zeroAsBg) {
        ArrayList<Integer>[] regionPoints = findRegionsPoints(map, zeroAsBg);
        int regionNum = regionPoints.length;
        int [][] boxes = new int[regionNum][4];

        for (int region = 0; region < regionNum; region++) {
            int iMin = 10000000;
            int iMax = -1;
            int jMin = 10000000;
            int jMax = -1;
            for (int m = 0; m < regionPoints[region].size(); m += 2) {
                int i = regionPoints[region].get(m + 0);
                int j = regionPoints[region].get(m + 1);
                if (i < iMin) iMin = i;
                if (i > iMax) iMax = i;
                if (j < jMin) jMin = j;
                if (j > jMax) jMax = j;
            }
            boxes[region][0] = iMin;
            boxes[region][1] = jMin;
            boxes[region][2] = iMax;
            boxes[region][3] = jMax;
        }

        return boxes;
    }

    private int [][] findLabelMatrix(byte [][] map, boolean zeroAsBg) {
        int allPointsCnt = 0;
        String info;

        int h = map.length;
        int w = map[0].length;

        int [] labelArray = getLabels(map, zeroAsBg);
        int componentsNum = getMaxLabelNumber();
        int componentsPoints [] = new int[componentsNum];
        int [][] labelMatrix = new int[h][w];
        for (int i = 0; i < w; i++)
            for (int j = 0; j < h; j++) {
                labelMatrix[j][i] = labelArray[j*w+i];
            }

        int mapNoneZeroCnt = 0;
        for (int i = 0; i < w; i++)
            for (int j = 0; j < h; j++) {
                if (0 != map[j][i]) {
                    mapNoneZeroCnt += 1;
                }
            }

        int labelMatNoneZeroCnt = 0;
        for (int i = 0; i < labelArray.length; i++) {
            if (0 != labelArray[i]) {
                labelMatNoneZeroCnt += 1;
            }
        }

        /*info = String.format("Local Grid Map [%d x %d], occupied grid number = %d",
                h, w, mapNoneZeroCnt);
        Log.d(TAG, info);
        info = String.format("LabelArray of local Grid Map [%d], occupied label number = %d",
                labelArray.length, labelMatNoneZeroCnt);
        Log.d(TAG, info);*/

        for (int c = 1; c <= componentsNum; c++) {
            for (int i = 0; i < labelArray.length; i++) {
                if (c == labelArray[i]) {
                    componentsPoints[c-1] += 1;
                    allPointsCnt += 1;
                }
            }
        }

        /*for (int c = 1; c <= componentsNum; c++) {
            int num = componentsPoints[c-1];
            if (0 != num) {
                info = String.format("region %d: points number = %d", c, num);
                Log.d(TAG, info);
            }
        }

        info = String.format("findLabelMatrix() find %d regions, occupied label number = %d",
                componentsNum, allPointsCnt);
        Log.d(TAG, info);*/

        return labelMatrix;
    }

    private ArrayList<Integer>[] findRegionsPoints(byte [][] map, boolean zeroAsBg) {
        // from updated map, get the region components
        int [][] labelMatrix = findLabelMatrix(map, true);
        int regionNum = getMaxLabelNumber();
        ArrayList<Integer>[] regionPoints = new ArrayList[regionNum];

        int h = labelMatrix.length;
        int w = labelMatrix[0].length;

        for (int region = 0; region < regionNum; region++) {
            regionPoints[region] = new ArrayList<Integer>();
            for (int j = 0; j < h; j++)
                for (int i = 0; i < w; i++) {
                    if ((region + 1) == labelMatrix[j][i]) {
                        regionPoints[region].add(i);
                        regionPoints[region].add(j);
                    }
                }
        }

        for (int region = 0; region < regionNum; region++) {
            if (0 == regionPoints[region].size()) {
                /*String info = String.format("the region %d have 0 number point, error", region);
                Log.w(TAG, info);*/
            }
        }
        return regionPoints;
    }

    private int[] findLabelArray(int[] myMap, int w, int h, boolean zeroIsBackground)
    {
        mNextLabel = 1;

        // label first
        int[] label= labelingMap(myMap, w, h,zeroIsBackground);
        int[] stat= new int[mNextLabel+1];
        String info = "";

        for (int i = 0; i < myMap.length; i++) {
            /*if (label[i] > mNextLabel) {
                Log.e(TAG, "bigger label than mNextLabel found!");
            }*/
            stat[label[i]]++;
        }

        stat[0] = 0;            // label 0 will be mapped to 0
                                // whether 0 is background or not
        int j = 1;
        for (int i = 1; i < stat.length; i++) {
            if (stat[i]!=0) {
                stat[i]=j++;
            }
        }

        /*info = String.format("From %d to %d regions", mNextLabel, (j-1));
        Log.i(TAG, info);*/

        mNextLabel = j-1;
        for (int i = 0; i < myMap.length; i++) {
            label[i] = stat[label[i]];
        }
        return label;
    }

    private int[] getLabels(byte[][] myMap, boolean zeroIsBackground) {
        int h = myMap.length;
        int w = myMap[0].length;
        int [] imageArray = new int [w*h];
        for (int i = 0; i < w; i++)
            for (int j = 0; j < h; j++) {
                imageArray[j*w+i] = myMap[j][i];
            }
        return findLabelArray(imageArray, w, h, zeroIsBackground);
    }

    private int[] labelingMap(int[] myMap, int w, int h, boolean zeroIsBackground)
    {
        int[] labelMat = new int[w*h];
        int[] parent = new int[MAX_LABEL_NUM];
        int[] labels = new int[MAX_LABEL_NUM];
        
        // region label starts from 1;
        // this is required as union-find data structure
        int next_region = 1;
        for (int y = 0; y < h; ++y){
            for (int x = 0; x < w; ++x){
                if (myMap[y*w+x] == 0 && zeroIsBackground) {
                    continue;
                }
                int k = 0;
                boolean connected = false;
                
                // if connected to the left
                if (x > 0 && myMap[y*w+x-1] == myMap[y*w+x]) {
                   k = labelMat[y*w+x-1];
                   connected = true;
                }
                
                // if connected to the up
                if (y > 0 && myMap[(y-1)*w+x] == myMap[y*w+x] &&
                    (connected = false || myMap[(y-1)*w+x] < k)) {
                    k = labelMat[(y-1)*w+x];
                    connected = true;
                }
                if (!connected) {
                    k = next_region;
                    next_region++;
                }

                /*if (k >= MAX_LABEL_NUM) {
                    Log.e(TAG, "maximum number of labels reached, increase MAX_LABEL_NUM and recompile.");
                }*/
                labelMat[y*w+x] = k;
                // if connected, but with different label, then do union
                if (x > 0 && myMap[y*w+x-1] == myMap[y*w+x] && labelMat[y*w+x-1] != k) {
                    uf_union(k, labelMat[y * w + x - 1], parent);
                }
                if (y > 0 && myMap[(y-1)*w+x] == myMap[y*w+x] && labelMat[(y-1)*w+x] != k) {
                    uf_union(k, labelMat[(y - 1) * w + x], parent);
                }
            }
        }

        // Begin the second pass. Assign the new labels
        // if 0 is reserved for background, then the first available label is 1
        mNextLabel = 1;
        for (int i = 0; i < w*h; i++) {
            if (myMap[i]!=0 || !zeroIsBackground) {
                labelMat[i] = uf_find(labelMat[i], parent, labels);
                // The labels are from 1, if label 0 should be considered, then
                // all the label should minus 1
                if (!zeroIsBackground) {
                    labelMat[i]--;
                }
            }
        }
        mNextLabel--;   // mNextLabel records the max label
        if (!zeroIsBackground) {
            mNextLabel--;
        }

        /*String info = String.format("%d regions", mNextLabel);
        Log.i(TAG, info);*/

        return labelMat;
    }

    private void uf_union(int x, int y, int[] parent)
    {
        while (parent[x] > 0) {
            x = parent[x];
        }
        while (parent[y] > 0) {
            y = parent[y];
        }
        if (x != y) {
            if (x < y) {
                parent[x] = y;
            } else {
                parent[y] = x;
            }
        }
    }

    private int uf_find(int x, int[] parent, int[] label)

     {
        while (parent[x] > 0) {
            x = parent[x];
        }
        if (label[x] == 0) {
            label[x] = mNextLabel++;
        }
        return label[x];
    }

}

