/**
 * 
 */
package edu.ccny;

import java.nio.FloatBuffer;
import java.util.logging.Level;
import java.util.logging.Logger;

import edu.ccny.obstacle.*;

public class MainTest {

	public MainTest() { /* TODO Auto-generated constructor stub */ }

	public static void main(String[] args) { // TODO Auto-generated method stub	
		// LOG.setLevel(Level.WARNING), LOG.severe/warning/info/finest
	    Logger LOG = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);
	    LOG.setLevel(Level.FINEST);
	    String info;
	    
	    // final String topPath = Environment.getExternalStorageDirectory();
		final String topPath = "C:/Users/df/Documents/code/bing/indoormapseditor/";
		final String mDir = "obstacleTango";
		// pointCloud3D_20160202_2007_data_moving2person
		//    1:10 works fine {-2, 2, 0, 3, -0.8f, 0.8f}
		//    3:10 works fine {-1, 1, 0, 2, -0.8f, 0.5f}, {-0.8f, 0.8f, 0, 2, -0.8f, 0.5f}
		// pointCloud3D_20160202_2009_data_moving2person_slow
		// pointCloud3D_20160202_2010_data_movingBoth
		final String mPointCloudDir = "pointCloud3D_20160202_2007_data_moving2person";
	    Utils mUtils = new Utils(topPath, mDir, mPointCloudDir);

		byte [][] buffer = mUtils.getMapImage("st6b_grid.png");
		float [] obstacleRange = new float[]{-2, 2, 0, 3, -0.8f, 0.8f};
		float [] mat = mUtils.getTransform();
		
	    ObstacleAvoidance mObstacleAvoidances = null;
	    mObstacleAvoidances = new ObstacleAvoidance(buffer, mat, 8, 0.05f, obstacleRange);

	    for (int id = 1; id < 2000; id++) {
	    	LOG.warning("main() loop .................. id = " + id);
			float [] q = mUtils.getQuaternion(id);
			if (null == q) {
				return;
			}
			float [] t = mUtils.getTranslation(id);
			FloatBuffer buf = mUtils.getRawPC(id);
			LOG.warning("FloatBuffer size = " + buf.capacity());
			double timeStamp = 0;
			
		    mObstacleAvoidances.updatingData(buf, t.clone(), q.clone(), timeStamp);
		    mObstacleAvoidances.setIsGlobalLocalized(true);
		    mObstacleAvoidances.detecting(id);
		    
		    int [] mObstaclePoints = null;
		    int [] mObstacleBoxesInt = null;
		    if (true == mObstacleAvoidances.detecting(id)) {
	            mObstaclePoints   = mObstacleAvoidances.getObstaclePoints();
	            mObstacleBoxesInt = mObstacleAvoidances.getObstacleBoxes();
	            
	            info = String.format("obstale points = %d, boxes = %d",
	    	    		mObstaclePoints.length/2, mObstacleBoxesInt.length/4);
	    	    LOG.warning(info);
		    } else {
	    	    LOG.warning("no obstacle");
		    }
	    }
	    
	    return;
	}
	
}
