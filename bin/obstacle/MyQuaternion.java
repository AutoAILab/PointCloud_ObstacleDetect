package edu.ccny.obstacle;

public class MyQuaternion {
    public double w, x, y, z;  // private final
	
    // create a new object with the given components
    public MyQuaternion(double w, double x, double y, double z) {
        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public MyQuaternion() {
        this.w = 1;
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }
    
    public MyQuaternion clone() {
    	MyQuaternion out = new MyQuaternion(w, x, y, z);
        return out;
    }
    
    // return a string representation of the invoking object
    public String toString() {
        return w + " + " + x + "i + " + y + "j + " + z + "k";
    }

    // return the quaternion norm
    public double norm() {
        return Math.sqrt(w * w + x * x + y * y + z * z);
    }

    public static double [][] getRotationMatrixByPitch(double pitchEuler) {
        double [][] m = new double[3][3];
        double rad = Math.toRadians(pitchEuler);
        double cos = Math.cos(rad);
        double sin = Math.sin(rad);
        m[0][0] = cos;
        m[0][2] = sin;
        m[1][1] = 1;
        m[2][0] = sin * (-1);
        m[2][2] = cos;
        return m;
    }

    public static double [][] getRotationMatrixByRoll(double rollEuler) {
        double [][] m = new double[3][3];
        double rad = Math.toRadians(rollEuler);
        double cos = Math.cos(rad);
        double sin = Math.sin(rad);
        m[0][0] = 1;
        m[1][1] = cos;
        m[1][2] = sin * (-1);
        m[2][1] = sin;
        m[2][2] = cos;
        return m;
    }

    public static double [][] get2DTransformModel(float tran_x, float tran_y, float yawEuler) {
        double [][] m = new double[3][3];
        double rad = Math.toRadians(yawEuler);
        double cos = Math.cos(rad);
        double sin = Math.sin(rad);
        m[0][0] = cos;
        m[0][1] = sin * (-1);
        m[0][2] = tran_x;

        m[1][0] = sin;
        m[1][1] = cos;
        m[1][2] = tran_y;

        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
        return m;
    }

    public static float [] getEulerAngle(float [] quaternion) {
        double roll, pitch, yaw;
        // rotation matrix in tango frame
        double x = quaternion[0];
        double y = quaternion[1];
        double z = quaternion[2];
        double w = quaternion[3];
        double q0 = w;
        double q1 = x;
        double q2 = y;
        double q3 = z;

        roll = Math.atan2(2 * (q1 * q0 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
        pitch = Math.asin(2 * (q0 * q2 - q1 * q3));
        yaw = Math.atan2(2 * (q1 * q2 + q0 * q3), 1 - 2 * (q2 * q2 + q3 * q3));
        roll = Math.toDegrees(roll);
        pitch = Math.toDegrees(pitch);
        yaw = Math.toDegrees(yaw);
        float[] eulerAngle = {(float)roll, (float)pitch, (float)yaw};
        return eulerAngle;
    }

    public double[] getEulerAngle() {
		double r, p, y;
		double q0 = this.w;
		double q1 = this.x;
		double q2 = this.y;
		double q3 = this.z;
        r = Math.atan2(2 * (q1 * q0 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
        p = Math.asin(2 * (q0 * q2 - q1 * q3));
        y = Math.atan2(2 * (q1 * q2 + q0 * q3), 1 - 2 * (q2 * q2 + q3 * q3));
        r = Math.toDegrees(r);
        p = Math.toDegrees(p);
        y = Math.toDegrees(y);
		double[] euler = {r, p, y};
		return euler;
	}
    
    // return the quaternion conjugate
    public MyQuaternion conjugate() {
        return new MyQuaternion(w, -x, -y, -z);
    }

    // return a new MyQuaternion whose value is (this + b)
    public MyQuaternion plus(MyQuaternion b) {
        MyQuaternion a = this;
        return new MyQuaternion(a.w+b.w, a.x+b.x, a.y+b.y, a.z+b.z);
    }

    // return a new MyQuaternion whose value is (this * b)
    public MyQuaternion times(MyQuaternion b) {
        MyQuaternion a = this;
        double y0 = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
        double y1 = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
        double y2 = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
        double y3 = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
        return new MyQuaternion(y0, y1, y2, y3);
    }

    // return a new MyQuaternion whose value is the inverse of this
    public MyQuaternion inverse() {
        double d = w*w + x*x + y*y + z*z;
        return new MyQuaternion(w/d, -x/d, -y/d, -z/d);
    }

    // return a / b
    // we use the definition a * b^-1 (as opposed to b^-1 a)
    public MyQuaternion divides(MyQuaternion b) {
         MyQuaternion a = this;
        return a.times(b.inverse());
    }
    
    public MyMatrix RotationMatrix() {
    	double[][] m = new double[3][];
    	m[0] = new double[3];
    	m[1] = new double[3];
    	m[2] = new double[3];
    	MyQuaternion q = this;
    	
    	double sqw = q.w*q.w;
        double sqx = q.x*q.x;
        double sqy = q.y*q.y;
        double sqz = q.z*q.z;

        // invs (inverse square length) is only required if quaternion is not already normalised
        double invs = 1 / (sqx + sqy + sqz + sqw);
        m[0][0] = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
        m[1][1] = (-sqx + sqy - sqz + sqw)*invs ;
        m[2][2] = (-sqx - sqy + sqz + sqw)*invs ;
        
        double tmp1 = q.x*q.y;
        double tmp2 = q.z*q.w;
        m[1][0] = 2.0 * (tmp1 + tmp2)*invs ;
        m[0][1] = 2.0 * (tmp1 - tmp2)*invs ;
        
        tmp1 = q.x*q.z;
        tmp2 = q.y*q.w;
        m[2][0] = 2.0 * (tmp1 - tmp2)*invs ;
        m[0][2] = 2.0 * (tmp1 + tmp2)*invs ;
        tmp1 = q.y*q.z;
        tmp2 = q.x*q.w;
        m[2][1] = 2.0 * (tmp1 + tmp2)*invs ;
        m[1][2] = 2.0 * (tmp1 - tmp2)*invs ;
        
        return new MyMatrix(m);
    }
    
    // sample client for testing
    public static void main(String[] args) {
        MyQuaternion a = new MyQuaternion(3.0, 1.0, 0.0, 0.0);
        System.out.println("a = " + a);

        MyQuaternion b = new MyQuaternion(0.0, 5.0, 1.0, -2.0);
        System.out.println("b = " + b);

        System.out.println("norm(a)  = " + a.norm());
        System.out.println("conj(a)  = " + a.conjugate());
        System.out.println("a + b    = " + a.plus(b));
        System.out.println("a * b    = " + a.times(b));
        System.out.println("b * a    = " + b.times(a));
        System.out.println("a / b    = " + a.divides(b));
        System.out.println("a^-1     = " + a.inverse());
        System.out.println("a^-1 * a = " + a.inverse().times(a));
        System.out.println("a * a^-1 = " + a.times(a.inverse()));
    }

}

