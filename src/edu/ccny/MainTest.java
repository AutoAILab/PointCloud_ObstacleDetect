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
	    
	    ObstacleAvoidance mObstacleAvoidances = null;

		int id = 3;
		float [] q = Utils.getQuaternion(id);
		float [] t = Utils.getTranslation(id);
		FloatBuffer buf = Utils.getRawPC(id);
		LOG.warning("FloatBuffer size = " + buf.capacity());
		double timeStamp = 0;
		
		byte [][] buffer = Utils.getMapImage("st6b_grid.png");
		
		float [] obstacleRange = new float[]{-2, 2, 0, 4, -1, 1};
		
		float [] mat = Utils.getTransform();
		mObstacleAvoidances = new ObstacleAvoidance(buffer, mat, 8, 0.05f, obstacleRange);
	    mObstacleAvoidances.updatingData(buf, t.clone(), q.clone(), timeStamp);
	    mObstacleAvoidances.setIsGlobalLocalized(true);
	    
	    int [] mObstaclePoints = null;
	    int [] mObstacleBoxesInt = null;
	    if (true == mObstacleAvoidances.detecting(id)) {
            mObstaclePoints   = mObstacleAvoidances.getObstaclePoints();
            mObstacleBoxesInt = mObstacleAvoidances.getObstacleBoxes();
            
            info = String.format("obstale points length = %d, box length = %d",
    	    		mObstaclePoints.length, mObstacleBoxesInt.length);
    	    LOG.warning(info);
	    } else {
    	    LOG.warning("no obstacle");
	    }
	    
	    return;
	}
	
}
