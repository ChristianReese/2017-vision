package first.frc.team2077.season2017.vision.trackers;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class LineSegment 
{
	private Point pt1;
	private Point pt2;
	
	public LineSegment(Point pt1, Point pt2) 
	{
		this.pt1 = new Point( pt1.x, pt1.y );
		this.pt2 = new Point( pt2.x, pt2.y );
	}
	
	public LineSegment( LineSegment copyFrom )
	{
		if ( copyFrom != null )
		{
			pt1 = new Point( copyFrom.pt1.x, copyFrom.pt1.y );
			pt2 = new Point( copyFrom.pt2.x, copyFrom.pt2.y );
		}
	}

	/**
	 * @return the pt1
	 */
	public Point getPt1() {
		return pt1;
	}

	/**
	 * @return the pt2
	 */
	public Point getPt2() {
		return pt2;
	}
	
	public void set( LineSegment other ) 
	{
		if ( other != null )
		{
			if ( ( other.pt1 != null ) && ( other.pt2 != null ) )
			{
				this.pt1 = new Point( other.pt1.x, other.pt1.y );
				this.pt2 = new Point( other.pt2.x, other.pt2.y );
			}
		}
	}
	
	public void set( Point pt1, Point pt2 ) 
	{
		if ( ( pt1 != null ) && ( pt2 != null ) )
		{
			this.pt1 = new Point ( pt1.x, pt1.y );
			this.pt2 = new Point ( pt2.x, pt2.y );
		}
	}

	public void draw( int thickness, Mat output, double[] color )
	{
		Imgproc.line( output, pt1, pt2, new Scalar( color ), thickness );
	}

	public void draw( int thickness, Mat output )
	{
		draw( thickness, output, Utility.red );
	}
	
	public double calculateLength()
	{
		return Utility.getPointsDistance( pt1, pt2 );
	}
	
	public double calculateAngle( boolean fast )
	{
		double dx = pt2.x - pt1.x;
		double dy = pt2.y - pt1.y;		
		double arcCosInput = dx / calculateLength();		
		double toAdd = 0.0;
		
		if ( dy < 0.0 )
		{
			arcCosInput *= -1.0;
			toAdd = 180.0;
		}
		
		return Utility.mod( ( fast ? Utility.fastArcCosine( arcCosInput ) : Math.toDegrees( Math.acos( arcCosInput ) ) ) + toAdd, 360.0 );
	}
	
	/**
	 * @return True if this line intersects with other, false otherwise.
	 */
	public boolean isIntersectingWith( LineSegment other )
	{
		return Utility.linesIntersect( this.pt1.x, this.pt1.y, this.pt2.x, this.pt2.y, 
									other.pt1.x, other.pt1.y, other.pt2.x, other.pt2.y );
	}
	
	public boolean isCollinearWith( LineSegment other )
	{
		final double MAX_ERROR = 20.0;//10.0;
		
		LineSegment baseLine = this;

		double angle1 = Math.abs( Utility.getLowestAngleBetween( baseLine, new LineSegment( pt1, other.pt1 ), true ) );
		double angle2 = Math.abs( Utility.getLowestAngleBetween( baseLine, new LineSegment( pt1, other.pt2 ), true ) );
		double angle3 = Math.abs( Utility.getLowestAngleBetween( baseLine, other, true ) );
		
		return ( ( angle1 < MAX_ERROR ) && ( angle2 < MAX_ERROR ) && ( angle3 < MAX_ERROR ) );
	}
	
	public Point getNormalVect()
	{
		double calculatedLength = calculateLength();
		
		if ( calculatedLength > 0.0 )
		{
			return new Point( ( pt2.x - pt1.x ) / calculatedLength, ( pt2.y - pt1.y ) / calculatedLength );
		}
		
		return new Point();
	}

	/* (non-Javadoc)
	 * @see java.lang.Object#hashCode()
	 */
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((pt1 == null) ? 0 : pt1.hashCode());
		result = prime * result + ((pt2 == null) ? 0 : pt2.hashCode());
		return result;
	}

	/* (non-Javadoc)
	 * @see java.lang.Object#equals(java.lang.Object)
	 */
	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		LineSegment other = (LineSegment) obj;
		if (pt1 == null) {
			if (other.pt1 != null)
				return false;
		} else if (!pt1.equals(other.pt1))
			return false;
		if (pt2 == null) {
			if (other.pt2 != null)
				return false;
		} else if (!pt2.equals(other.pt2))
			return false;
		return true;
	}
	
}
