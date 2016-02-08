package edu.ccny.obstacle;

public class MyMatrix {

	private int nrows;
	private int ncols;
	private double[][] data;

	public MyMatrix(double[][] dat) {
		this.data = dat;
		this.nrows = dat.length;
		this.ncols = dat[0].length;
	}

	public MyMatrix(int nrow, int ncol) {
		this.nrows = nrow;
		this.ncols = ncol;
		data = new double[nrow][ncol];
	}

	public int getNrows() {
		return nrows;
	}

	public void setNrows(int nrows) {
		this.nrows = nrows;
	}

	public int getNcols() {
		return ncols;
	}

	public void setNcols(int ncols) {
		this.ncols = ncols;
	}

	public double[][] getValues() {
		return data;
	}

	public void setValues(double[][] values) {
		this.data = values;
	}

	public void setValueAt(int row, int col, double value) {
		data[row][col] = value;
	}

	public double getValueAt(int row, int col) {
		return data[row][col];
	}

	public boolean isSquare() {
		return nrows == ncols;
	}

	public int size() {
		if (isSquare())
			return nrows;
		return -1;
	}

	public MyMatrix multiplyByConstant(double constant) {
		MyMatrix mat = new MyMatrix(nrows, ncols);
		for (int i = 0; i < nrows; i++) {
			for (int j = 0; j < ncols; j++) {
				mat.setValueAt(i, j, data[i][j] * constant);
			}
		}
		return mat;
	}
	public MyMatrix insertColumnWithValue1() {
		MyMatrix X_ = new MyMatrix(this.getNrows(), this.getNcols()+1);
		for (int i=0;i<X_.getNrows();i++) {
			for (int j=0;j<X_.getNcols();j++) {
				if (j==0)
					X_.setValueAt(i, j, 1.0);
				else 
					X_.setValueAt(i, j, this.getValueAt(i, j-1));
				
			}
		}
		return X_;
	}
	
	public MyQuaternion ToQuaternion() {
		double m00 = data[0][0];
		double m01 = data[0][1];
		double m02 = data[0][2];
		double m10 = data[1][0];
		double m11 = data[1][1];
		double m12 = data[1][2];
		double m20 = data[2][0];
		double m21 = data[2][1];
		double m22 = data[2][2];
		MyQuaternion q = new MyQuaternion();
		
		q.w = Math.sqrt(1.0 + m00 + m11 + m22) / 2.0;
		double w4 = (4.0 * q.w);
		q.x = (m21 - m12) / w4 ;
		q.y = (m02 - m20) / w4 ;
		q.z = (m10 - m01) / w4 ;
		return q;
	}
}
