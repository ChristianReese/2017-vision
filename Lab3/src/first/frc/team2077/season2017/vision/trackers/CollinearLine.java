package first.frc.team2077.season2017.vision.trackers;

import org.opencv.core.Mat;
import org.opencv.core.Point;

public class CollinearLine 
{
	private LineSegment projectedSegment1; // Pt1 of this segment is Pt1 of this collinear line
	private LineSegment projectedSegment2; // Pt2 of this segment is Pt2 of this collinear line
	private LineSegment bridgeLine;

	private double segment1Fraction;
	private double segment2Fraction;

	private Polygon segment1ParentPolygon;
	private Polygon segment2ParentPolygon;
	
	private double totalLength;
	
	public static void correctIntersectingCLPair( CollinearLine cl1, CollinearLine cl2 )
	{
    	Point pt1 = cl1.projectedSegment1.getPt1();
    	Point pt2 = cl1.projectedSegment2.getPt2();
    	Point pt3 = cl2.projectedSegment1.getPt1();
    	Point pt4 = cl2.projectedSegment2.getPt2();
    	
    	if ( Utility.linesIntersect( pt1.x, pt1.y, pt2.x, pt2.y, pt3.x, pt3.y, pt4.x, pt4.y ) )
    	{
    		cl1.reMapToLine( new LineSegment( pt1, pt4 ) );
    		cl2.reMapToLine( new LineSegment( pt3, pt2 ) );
    	}
	}
	
	public static CollinearLine createCollinearLine( LineSegment segment, Polygon parent )
	{
		if ( segment == null )
		{
			return null;
		}
		
		CollinearLine result = new CollinearLine( null );		

		result.projectedSegment1 = new LineSegment( segment.getPt1(), segment.getPt1() );
		result.bridgeLine = new LineSegment( segment.getPt1(), segment.getPt2() );
		result.projectedSegment2 = new LineSegment( segment.getPt2(), segment.getPt2() );

		result.segment1Fraction = 0.0;
		result.segment2Fraction = 0.0;

		result.segment1ParentPolygon = parent;
		result.segment2ParentPolygon = parent;
		
		result.totalLength = segment.calculateLength();
		
		return result;
	}
	
	public static CollinearLine createCollinearLine( LineSegment segment1, LineSegment segment2,
			Polygon segment1ParentPolygon, Polygon segment2ParentPolygon )
	{
		if ( ( segment1 == null ) || ( segment2 == null )
				|| ( segment1ParentPolygon == null ) || ( segment2ParentPolygon == null ) )
		{
			return null;
		}
		
		CollinearLine result = new CollinearLine( null );		

		Point averagePointA = Utility.getAveragePoint( segment1.getPt1(), segment1.getPt2() );
		Point averagePointB = Utility.getAveragePoint( segment2.getPt1(), segment2.getPt2() );
		Point averagePointAB = Utility.getAveragePoint( averagePointA, averagePointB );
		
		LineSegment projectionLine = new LineSegment( averagePointAB, 
				new Point( averagePointAB.x + averagePointB.x - averagePointA.x, 
						   averagePointAB.y + averagePointB.y - averagePointA.y ) );
		
		LineSegment totalLine;
		double totalLength;
		
		LineSegment bridgeLine;

		result.segment1ParentPolygon = segment1ParentPolygon;
		result.segment2ParentPolygon = segment2ParentPolygon;
		
		result.projectedSegment1 = Utility.projectLineSegmentOnLine( segment1, projectionLine );
		result.projectedSegment2 = Utility.projectLineSegmentOnLine( segment2, projectionLine );
		
		totalLine = new LineSegment( result.projectedSegment1.getPt1(), result.projectedSegment2.getPt2() );
		totalLength = totalLine.calculateLength();

		result.segment1Fraction = result.projectedSegment1.calculateLength() / totalLength;
		result.segment2Fraction = result.projectedSegment2.calculateLength() / totalLength;
		
		bridgeLine = new LineSegment( result.projectedSegment1.getPt1(), result.projectedSegment2.getPt2() );
		
		result.bridgeLine = bridgeLine;
		result.totalLength = bridgeLine.calculateLength();
		
		return result;
	}
	
	public CollinearLine( CollinearLine copyFrom )
	{
		if ( copyFrom != null )
		{
			this.projectedSegment1 = new LineSegment( copyFrom.projectedSegment1 );
			this.projectedSegment2 = new LineSegment( copyFrom.projectedSegment2 );

			this.segment1Fraction = copyFrom.segment1Fraction;
			this.segment2Fraction = copyFrom.segment2Fraction;

			this.segment1ParentPolygon = copyFrom.segment1ParentPolygon;
			this.segment2ParentPolygon = copyFrom.segment2ParentPolygon;

			this.bridgeLine = copyFrom.bridgeLine;
			this.totalLength = copyFrom.totalLength;
		}
	}
	
	public void draw( Mat output )
	{
		final int PT_SIZE = 5;
		
		if ( ( projectedSegment1 != null ) && ( projectedSegment2 != null ) )
		{
			LineSegment bridgeLine = new LineSegment( projectedSegment1.getPt1(), projectedSegment2.getPt2() );
			bridgeLine.draw( 5, output );

			Utility.drawPoint( projectedSegment1.getPt1(), Utility.white, PT_SIZE, output );
			Utility.drawPoint( projectedSegment1.getPt2(), Utility.white, PT_SIZE, output );
			Utility.drawPoint( projectedSegment2.getPt1(), Utility.white, PT_SIZE, output );
			Utility.drawPoint( projectedSegment2.getPt2(), Utility.white, PT_SIZE, output );
		}
	}
	
	public boolean isInverselyPairedWith( CollinearLine other )
	{
		LineSegment line1 = new LineSegment( projectedSegment1.getPt1(), other.projectedSegment1.getPt1() );
		LineSegment line2 = new LineSegment( projectedSegment2.getPt2(), other.projectedSegment2.getPt2() );
		
		return line1.isIntersectingWith( line2 );
	}
	
	public CollinearLine getThisInverted()
	{
		CollinearLine result = new CollinearLine( null );

		result.projectedSegment1 = new LineSegment( this.projectedSegment2.getPt2(), this.projectedSegment2.getPt1() );
		result.projectedSegment2 = new LineSegment( this.projectedSegment1.getPt2(), this.projectedSegment1.getPt1() );

		result.segment1Fraction = this.segment2Fraction;
		result.segment2Fraction = this.segment1Fraction;

		result.segment1ParentPolygon = this.segment2ParentPolygon;
		result.segment2ParentPolygon = this.segment1ParentPolygon;

		result.bridgeLine = this.bridgeLine;
		result.totalLength = this.totalLength;
		
		return result;
	}
	
	public void reMapToLine( LineSegment mapTo )
	{
		bridgeLine = new LineSegment( mapTo.getPointAlongLine( segment1Fraction ), 
				mapTo.getPointAlongLine( 1.0 - segment2Fraction ) );

		projectedSegment1 = new LineSegment( mapTo.getPt1(), bridgeLine.getPt1() );
		projectedSegment2 = new LineSegment( bridgeLine.getPt2(), mapTo.getPt2() );
	}
	
	public Point getPt1()
	{
		return projectedSegment1.getPt1();
	}
	
	public Point getPt2()
	{
		return projectedSegment1.getPt2();
	}
	
	public Point getPt3()
	{
		return projectedSegment2.getPt1();
	}
	
	public Point getPt4()
	{
		return projectedSegment2.getPt2();
	}

	public double getSegment1Fraction() 
	{
		return segment1Fraction;
	}

	public double getSegment2Fraction() 
	{
		return segment2Fraction;
	}

	public Polygon getSegment1ParentPolygon() 
	{
		return segment1ParentPolygon;
	}

	public Polygon getSegment2ParentPolygon() 
	{
		return segment2ParentPolygon;
	}

	public double getTotalLength() 
	{
		return totalLength;
	}

	public LineSegment getBridgeLine() 
	{
		return bridgeLine;
	}
	
}
