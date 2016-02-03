package edu.ccny.obstacle;

// import android.os.Environment;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;

public class Utils {
	// LOG.setLevel(Level.WARNING), LOG.severe/warning/info/finest
    private static Logger LOG = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);

    final static String pathSeparator = "/";
	private String topPath = "";
	private String mDir = "";
	private String mPointCloudDir = "";
	
    /*
    12_quaternion.txt
	12_raw.txt
	12_timestamp.txt
	12_translation.txt
	localGranularityMeter.txt
	model.txt
	obstacleWall.txt
	range.txt
    */

	public Utils (String topPath, String mDir, String mPointCloudDir) {
		this.topPath = topPath;
		this.mDir = mDir;
		this.mPointCloudDir = mPointCloudDir;
    	LOG.setLevel(Level.WARNING);
	}
	
    private File createDataDir() {
    	File file;

    	file = new File(topPath, mDir);
        file = new File(file, mPointCloudDir);
        return file;
    }
    
    public byte[][] getMapImage(String mapName) {
    	BufferedImage img = null;
    	
    	File file = new File(topPath, mDir);
        file = new File(file, mapName);
    	
        try {
            img = ImageIO.read(file);
        } catch (IOException e) {
        	e.printStackTrace();
	        return null;
        }

        int height = img.getHeight();
        int width = img.getWidth();
        int occupiedCnt = 0;
        int unOccupiedCnt = 0;
        int othersCnt = 0;

        byte[][] buffer = new byte[height][width];

        for (int h = 0; h < height; h++) {
            for (int w = 0; w < width; w++) {
                int rgb = img.getRGB(w, h);
                
                if (-16777216 == rgb) {
                	buffer[h][w] = 0x00;
                	occupiedCnt++;
                } else if (-1 == rgb) {
                	buffer[h][w] = (byte)0xFF;
                	unOccupiedCnt++;
                } else {
                	othersCnt++;
                }
            }
        }
        
        return buffer;
    }
    
    public float [] getTransform() {
    	File file = createDataDir();
    	
    	String line;
        String filePathName = file.getPath() + 
        		pathSeparator + "model.txt";
		try {
			InputStream fIn = new FileInputStream(filePathName);
			BufferedReader reader = new BufferedReader(new InputStreamReader(fIn));

			int len = 9;
			float [] array = new float[len];
			for (int i = 0; i < len; i++) {
				line = reader.readLine();
				array[i] = Float.parseFloat(line.trim());
			}
			
			reader.close();
	        fIn.close();
	        
	        return array;
		} catch (Exception e) {
            e.printStackTrace();
	        return null;
        }
    }
    
    public float [] getQuaternion(int id) {
    	File file = createDataDir();
    	
    	String line;
        String filePathName = file.getPath() + 
        		pathSeparator + String.valueOf(id) + "_quaternion.txt";
		try {
			InputStream fIn = new FileInputStream(filePathName);
			BufferedReader reader = new BufferedReader(new InputStreamReader(fIn));

			int len = 4;
			float [] array = new float[len];
			for (int i = 0; i < len; i++) {
				line = reader.readLine();
				array[i] = Float.parseFloat(line.trim());
			}
			
			reader.close();
	        fIn.close();
	        
	        return array;
		} catch (Exception e) {
            // e.printStackTrace();
	        return null;
        }
    }
    
    public float [] getTranslation(int id) {
    	File file = createDataDir();
    	
    	String line;
        String filePathName = file.getPath() + 
        		pathSeparator + String.valueOf(id) + "_translation.txt";
		try {
			InputStream fIn = new FileInputStream(filePathName);
			BufferedReader reader = new BufferedReader(new InputStreamReader(fIn));

			int len = 3;
			float [] array = new float[len];
			for (int i = 0; i < len; i++) {
				line = reader.readLine();
				array[i] = Float.parseFloat(line.trim());
			}

			reader.close();
	        fIn.close();
	        
	        return array;
		} catch (Exception e) {
            e.printStackTrace();
	        return null;
        }
    }
    
    public FloatBuffer getRawPC(int id) {
    	File file = createDataDir();
    	final int FLOAT_SIZE = 4;
    	
    	String line;
    	int lineCnt = 0;
        String filePathName = file.getPath() + 
        		pathSeparator + String.valueOf(id) + "_raw.txt";
		try {
			InputStream fIn = new FileInputStream(filePathName);
			BufferedReader reader = new BufferedReader(new InputStreamReader(fIn));
	        ArrayList<String> lineStrs = new ArrayList<String>();

			while ((line = reader.readLine()) != null) {
				lineCnt += 1;
				lineStrs.add(line);
			}
			
			FloatBuffer buf = ByteBuffer.allocateDirect(FLOAT_SIZE * lineCnt * 3).order(ByteOrder.nativeOrder()).asFloatBuffer();
			
			for (int i = 0; i < lineCnt; i++) {
				String[] sp = lineStrs.get(i).split(" ");
				float x = Float.parseFloat(sp[0].trim());
				float y = Float.parseFloat(sp[1].trim());
				float z = Float.parseFloat(sp[2].trim());
				buf.put(i * 3 + 0, x);
				buf.put(i * 3 + 1, y);
				buf.put(i * 3 + 2, z);
			}
			reader.close();
	        fIn.close();
	        
	        return buf;
		} catch (Exception e) {
            e.printStackTrace();
	        return null;
        }
    }
    
    // file  =  new File(Environment.getExternalStorageDirectory(), mDir);
    // file = new File(\", mDir);
    public void removeFramesData() {
        File file = null;
        try {
            file = createDataDir();
            if(file.exists()) {
                file.delete();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void saveOneFrameData(float [] dataAll, int dim, String fileName) {
        saveOneFrameData(dataAll, dataAll.length, dim, fileName);
    }

    public void saveStr2File(String str, String fileName) {
        File file = null;
        try {
        	file = createDataDir();
            if (file.mkdirs()) {
            	LOG.warning("Directory created");
            }

            String filePathName = file.getPath() + pathSeparator + fileName;
            File myFile = new File(filePathName);
            if(myFile.exists()) {
                myFile.delete();
            } else {
                myFile.createNewFile();
            }
            FileOutputStream fOut = new FileOutputStream(myFile);
            OutputStreamWriter myOutWriter = new OutputStreamWriter(fOut);

            myOutWriter.append(str);

            myOutWriter.close();
            fOut.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void saveOneFrameData(float [] dataAll, int dataAllLen, int dim, String fileName) {
        File file = null;
        try {
        	file = createDataDir();
            if (file.mkdirs()) {
            	LOG.warning("Directory created");
            }

            String filePathName = file.getPath() + pathSeparator + fileName;
            File myFile = new File(filePathName);
            if(myFile.exists()) {
                myFile.delete();
            } else {
                myFile.createNewFile();
            }
            FileOutputStream fOut = new FileOutputStream(myFile);
            OutputStreamWriter myOutWriter = new OutputStreamWriter(fOut);

            if (dataAllLen % dim != 0) {
            	String info = String.format("dataAllLen % %d != 0", dim);
            	LOG.warning(info);
            }
            dataAllLen = dim * (dataAllLen/dim);
            for (int i = 0; i < dataAllLen; i += dim) {
            	float x = dataAll[i + 0];
                float y = dataAll[i + 1];
                myOutWriter.append(String.valueOf(x) + " ");
            	if (2 == dim) {
                    myOutWriter.append(String.valueOf(y) + "\n");
            	} else if (3 == dim) {
                    float z = dataAll[i + 2];
                    myOutWriter.append(String.valueOf(y) + " ");
                    myOutWriter.append(String.valueOf(z) + "\n");
            	}
            }
            myOutWriter.close();
            fOut.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void saveOneFrameData(int [] dataAll, int dim, String fileName) {
        saveOneFrameData(dataAll, dataAll.length, dim, fileName);
    }

    public void saveOneFrameData(int [] dataAll, int dataAllLen, int dim, String fileName) {
        File file = null;
        try {
        	file = createDataDir();
            if (file.mkdirs()) {
                LOG.warning("Directory created");
            }

            String filePathName = file.getPath() + pathSeparator + fileName;
            File myFile = new File(filePathName);
            if(myFile.exists()) {
                myFile.delete();
            } else {
                myFile.createNewFile();
            }
            FileOutputStream fOut = new FileOutputStream(myFile);
            OutputStreamWriter myOutWriter = new OutputStreamWriter(fOut);

            if (dataAllLen % dim != 0) {
            	String info = String.format("dataAllLen % %d != 0", dim);
            	LOG.warning(info);
            }
            dataAllLen = dim * (dataAllLen/dim);
            for (int i = 0; i < dataAllLen; i += dim) {
            	int x = dataAll[i + 0];
            	int y = dataAll[i + 1];
                myOutWriter.append(String.valueOf(x) + " ");
            	if (2 == dim) {
                    myOutWriter.append(String.valueOf(y) + "\n");
            	} else if (3 == dim) {
            		int z = dataAll[i + 2];
                    myOutWriter.append(String.valueOf(y) + " ");
                    myOutWriter.append(String.valueOf(z) + "\n");
            	}
            }
            myOutWriter.close();
            fOut.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void saveOneFrameData(FloatBuffer dataAll, String fileName) {
        File file = null;
        try {
        	file = createDataDir();
            if (file.mkdirs()) {
                LOG.warning("Directory created");
            }

            String filePathName = file.getPath() + pathSeparator + fileName;
            File myFile = new File(filePathName);
            if(myFile.exists()) {
                myFile.delete();
            } else {
                myFile.createNewFile();
            }
            FileOutputStream fOut = new FileOutputStream(myFile);
            OutputStreamWriter myOutWriter = new OutputStreamWriter(fOut);

            int dataAllLen = dataAll.capacity();
            if (dataAllLen % 3 != 0) {
                LOG.warning("dataAllLen % 3 != 0");
            }
            dataAllLen = 3 * (dataAllLen/3);
            for (int i = 0; i < dataAllLen; i += 3) {
                float x = dataAll.get(i + 0);
                float y = dataAll.get(i + 1);
                float z = dataAll.get(i + 2);
                myOutWriter.append(String.valueOf(x) + " ");
                myOutWriter.append(String.valueOf(y) + " ");
                myOutWriter.append(String.valueOf(z) + "\n");
            }
            myOutWriter.close();
            fOut.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
