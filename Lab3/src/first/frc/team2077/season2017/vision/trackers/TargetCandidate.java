package first.frc.team2077.season2017.vision.trackers;

import org.opencv.core.Mat;

public class TargetCandidate 
{
	public static final double MIN_SCORE = 70.0;
	
	private CollinearLine cl1 = null;
	private CollinearLine cl2 = null;
	
	private double score = 0.0;
	
	public static TargetCandidate generateTargetCandidate( CollinearLine cl1, CollinearLine cl2,
			double cameraWidth, double cameraHeight )
	{
		final double CAMERA_DIAGONAL = Math.sqrt( cameraWidth*cameraWidth + cameraHeight*cameraHeight );
		final double MIN_TOTAL_SEGMENT_FRACTION = 0.2;
		final double MAX_TOTAL_SEGMENT_FRACTION = 0.5;
		final double MAX_TOTAL_SEGMENT_FRACTION_DIFFERENCE = 0.1;
		final double MIN_SIDE_LENGTH = CAMERA_DIAGONAL / 16.0;//50.0;

		final double CL_LENGTH_DIFFERENCE_SCORE_WEIGHT = 40.0;
		final double SIDE_LENGTH_DIFFERENCE_SCORE_WEIGHT = 40.0;
		final double SEGMENT_FRACTION_DIFFERENCE_SCORE_WEIGHT = 20.0;
		
		TargetCandidate result = new TargetCandidate();
		double cl1TotalSegmentFraction;
		double cl2TotalSegmentFraction;
		double totalSegmentFractionDifference;

		double smallestCLLength;
		double largestCLLength;
		double smallestSideLength;
		double largestSideLength;

		LineSegment side1;
		LineSegment side2;
		double side1Length;
		double side2Length;
		
		result.cl1 = new CollinearLine( cl1 );
		
		if ( cl1.isInverselyPairedWith( cl2 ) )
		{
			result.cl2 = cl2.getThisInverted();
		}
		else
		{
			result.cl2 = new CollinearLine( cl2 );
		}
		
		if ( ( result.cl1.getSegment1ParentPolygon() != result.cl2.getSegment1ParentPolygon() ) 
				&& ( result.cl1.getSegment2ParentPolygon() != result.cl2.getSegment2ParentPolygon() ) )
		{
			return null;
		}

		cl1TotalSegmentFraction = result.cl1.getSegment1Fraction() + result.cl1.getSegment2Fraction();
		cl2TotalSegmentFraction = result.cl2.getSegment1Fraction() + result.cl2.getSegment2Fraction();
		
		if ( ( cl1TotalSegmentFraction > MAX_TOTAL_SEGMENT_FRACTION )
				|| ( cl2TotalSegmentFraction > MAX_TOTAL_SEGMENT_FRACTION ) 
				|| ( cl1TotalSegmentFraction < MIN_TOTAL_SEGMENT_FRACTION )
				|| ( cl2TotalSegmentFraction < MIN_TOTAL_SEGMENT_FRACTION ) )
		{
			return null;
		}
		
		totalSegmentFractionDifference = Math.abs( cl1TotalSegmentFraction - cl2TotalSegmentFraction );
		
		if ( Double.isNaN( totalSegmentFractionDifference ) )
		{
			return null;
		}
		
		if ( totalSegmentFractionDifference > MAX_TOTAL_SEGMENT_FRACTION_DIFFERENCE )
		{
			//System.out.println( totalSegmentFractionDifference );
			return null;
		}
		
		if ( result.cl1.getTotalLength() > result.cl2.getTotalLength() )
		{
			largestCLLength = result.cl1.getTotalLength();
			smallestCLLength = result.cl2.getTotalLength();
		}
		else
		{
			largestCLLength = result.cl2.getTotalLength();
			smallestCLLength = result.cl1.getTotalLength();
		}
		
		if ( Double.isNaN( smallestCLLength ) || Double.isNaN( largestCLLength ) )
		{
			return null;
		}

		side1 = new LineSegment( result.cl1.getPt1(), result.cl2.getPt1() );
		side2 = new LineSegment( result.cl1.getPt4(), result.cl2.getPt4() );
		side1Length = side1.calculateLength();
		side2Length = side2.calculateLength();
		
		if ( side1Length > side2Length )
		{
			largestSideLength = side1Length;
			smallestSideLength = side2Length;
		}
		else
		{
			largestSideLength = side2Length;
			smallestSideLength = side1Length;
		}
		
		if ( Double.isNaN( smallestSideLength ) || Double.isNaN( largestSideLength ) )
		{
			return null;
		}
		
		if ( ( smallestSideLength < MIN_SIDE_LENGTH ) || ( smallestCLLength < MIN_SIDE_LENGTH )
				|| ( largestSideLength < MIN_SIDE_LENGTH ) || ( largestCLLength < MIN_SIDE_LENGTH ) )
		{
			//System.out.println( smallestSideLength + " " + smallestCLLength );
			return null;
		}

		result.score += ( smallestCLLength / largestCLLength ) * CL_LENGTH_DIFFERENCE_SCORE_WEIGHT;
		result.score += ( smallestSideLength / largestSideLength ) * SIDE_LENGTH_DIFFERENCE_SCORE_WEIGHT;
		result.score += ( MAX_TOTAL_SEGMENT_FRACTION_DIFFERENCE
				- Math.min( totalSegmentFractionDifference, MAX_TOTAL_SEGMENT_FRACTION_DIFFERENCE ) )
				* ( 1.0 / MAX_TOTAL_SEGMENT_FRACTION_DIFFERENCE ) * SEGMENT_FRACTION_DIFFERENCE_SCORE_WEIGHT;
		
		return result;
	}
	
	public void draw( Mat output )
	{
		LineSegment bridgeSegment1 = new LineSegment( cl1.getPt1(), cl2.getPt1() );
		LineSegment bridgeSegment2 = new LineSegment( cl1.getPt2(), cl2.getPt2() );
		LineSegment bridgeSegment3 = new LineSegment( cl1.getPt3(), cl2.getPt3() );
		LineSegment bridgeSegment4 = new LineSegment( cl1.getPt4(), cl2.getPt4() );

		bridgeSegment1.draw( output );
		bridgeSegment2.draw( output );
		bridgeSegment3.draw( output );
		bridgeSegment4.draw( output );

		cl1.draw( output );
		cl2.draw( output );
	}
	
	public double getScore()
	{
		return score;
	}
	
	public double calculateAngleDifference()
	{
		// Determine highest and lowest
		return Utility.getLowestAngleBetween( cl1.getBridgeLine(), cl2.getBridgeLine(), false );
	}
}
