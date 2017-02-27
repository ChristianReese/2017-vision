package first.frc.team2077.season2017.vision.trackers;

import java.util.ArrayList;

import org.opencv.core.Core;
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

	public void draw( Mat output )
	{
		double dx = pt2.x - pt1.x;
		double dy = pt2.y - pt1.y;
		
		double arcCosInput = dx / calculateLength();
		
		if ( dy < 0.0 )
		{
			arcCosInput *= -1.0;
		}
		
		double[] rotationColor = Utility.hueToRGB( Utility.fastArcCosine( arcCosInput ) * 2.0 );
		
		Imgproc.line( output, pt1, pt2, new Scalar( rotationColor ) );
		//Utility.drawPoint( pt1, red, 1, output );
		//Utility.drawPoint( pt2, blue, 1, output );
	}
	
	public double calculateLength()
	{
		return Utility.getPointsDistance( pt1, pt2 );
	}
	
	public double calculateAngle()
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
		
		return Utility.mod( Utility.fastArcCosine( arcCosInput ) + toAdd, 360.0 );
	}
	
	/**
	 * @return True if this line intersects with other, false otherwise.
	 */
	public boolean isIntersectingWith( LineSegment other )
	{
		return Utility.linesIntersect( this.pt1.x, this.pt1.y, this.pt2.x, this.pt2.y, 
									other.pt1.x, other.pt1.y, other.pt2.x, other.pt2.y );
	}
	
	/*public boolean isCollinearWith( LineSegment other, double dividendErrorDelta, double divisorErrorDelta )
	{
		Utility.MutableDouble dividend = new Utility.MutableDouble(0.0);
		Utility.MutableDouble divisor = new Utility.MutableDouble(0.0);
		
		if ( Utility.getUScalarDivisionComponents( this, other, dividend, divisor ) == false )
		{
			return false;
		}

		dividendErrorDelta = Math.abs( dividendErrorDelta );
		divisorErrorDelta = Math.abs( divisorErrorDelta );
		
		//System.out.println( dividend.getValue() + " ; " + divisor.getValue() );
		
		return ( ( Math.abs( divisor.getValue() ) <= divisorErrorDelta ) && Math.abs( dividend.getValue() ) <= dividendErrorDelta );
	}*/
	
	public boolean isCollinearWith( LineSegment other )
	{
		final double MAX_ERROR = 20.0;//10.0;
		
		LineSegment baseLine = this;

		double angle1 = Utility.getLowestAngleBetween( baseLine, new LineSegment( pt1, other.pt1 ) );
		double angle2 = Utility.getLowestAngleBetween( baseLine, new LineSegment( pt1, other.pt2 ) );
		double angle3 = Utility.getLowestAngleBetween( baseLine, other );
		
		return ( ( angle1 < MAX_ERROR ) && ( angle2 < MAX_ERROR ) && ( angle3 < MAX_ERROR ) );
	}
	
	public boolean isParallelWith( LineSegment other, double divisorErrorDelta )
	{
		Utility.MutableDouble dividend = new Utility.MutableDouble(0.0);
		Utility.MutableDouble divisor = new Utility.MutableDouble(0.0);
		
		if ( Utility.getUScalarDivisionComponents( this, other, dividend, divisor ) == false )
		{
			return false;
		}
		
		divisorErrorDelta = Math.abs( divisorErrorDelta );
		
		return ( Math.abs( divisor.getValue() ) <= divisorErrorDelta );
	}
	
	/**
	 * Adds extensionAmount in length to both ends of this line segment.
	 * @param extensionAmount The amount to extend by.
	 */
	public void extend( double extensionAmount )
	{
		double calculatedLength = calculateLength();
		
		if ( calculatedLength > 0.01 )
		{
			double normalizedSignedX = ( pt2.x - pt1.x ) / calculatedLength;
			double normalizedSignedY = ( pt2.y - pt1.y ) / calculatedLength;
	
			pt2.x += ( normalizedSignedX * extensionAmount );
			pt2.y += ( normalizedSignedY * extensionAmount );
			pt1.x -= ( normalizedSignedX * extensionAmount );
			pt1.y -= ( normalizedSignedY * extensionAmount );
		}
	}
	
	/**
	 * Provides a new instance of this line with extensionAmount in length added to both ends of the segment.
	 * @param extensionAmount The amount to extend by.
	 * @return A new instance of this line with the same center point and length = currentLength + (2 * extensionAmount).
	 */
	public LineSegment getThisExtended( double extensionAmount )
	{
		LineSegment result = new LineSegment( this );
		result.extend( extensionAmount );
		
		return result;
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
	
}
