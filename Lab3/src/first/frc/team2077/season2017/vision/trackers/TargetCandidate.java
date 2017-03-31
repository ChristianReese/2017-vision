package first.frc.team2077.season2017.vision.trackers;

import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.Point;

public class TargetCandidate 
{
	public static final double GEAR_TARGET_HEIGHT_INCHES = 5.0;
	public static final double GEAR_TARGET_TOTAL_WIDTH_INCHES = 10.0;
	public static final double MIN_SCORE = 70.0;
	
	private CollinearLine cl1 = null;
	private CollinearLine cl2 = null;
	
	private double score = 0.0;

	private double largestSideLength = -1.0;
	private double smallestSideLength = -1.0;
	
	private Point centerPoint = null;
	
	private boolean hasBothRectangles_;
	
	public static TargetCandidate generateTargetCandidate( CollinearLine cl1, 
			CollinearLine cl2, double cameraDiagonal, boolean usingHorizontalAxis )
	{
		final double MIN_TOTAL_SEGMENT_FRACTION = 0.2;
		final double MAX_TOTAL_SEGMENT_FRACTION = 0.5;
		final double MAX_TOTAL_SEGMENT_FRACTION_DIFFERENCE = 0.1;
		final double MIN_SIDE_LENGTH = cameraDiagonal / 23.2;//16.0;//50.0;

		final double CL_LENGTH_DIFFERENCE_SCORE_WEIGHT = 40.0;
		final double SIDE_LENGTH_DIFFERENCE_SCORE_WEIGHT = 40.0;
		final double SEGMENT_FRACTION_DIFFERENCE_SCORE_WEIGHT = 20.0;
		
		TargetCandidate result = new TargetCandidate();
		double cl1TotalSegmentFraction;
		double cl2TotalSegmentFraction;
		double totalSegmentFractionDifference;

		double smallestCLLength;
		double largestCLLength;

		LineSegment side1;
		LineSegment side2;
		double side1Length;
		double side2Length;

		Point avgCL1Point = null;
		Point avgCL2Point = null;
		
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
		
		CollinearLine.correctIntersectingCLPair( result.cl1, result.cl2 );

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
			result.largestSideLength = side1Length;
			result.smallestSideLength = side2Length;
		}
		else
		{
			result.largestSideLength = side2Length;
			result.smallestSideLength = side1Length;
		}
		
		if ( Double.isNaN( result.smallestSideLength ) || Double.isNaN( result.largestSideLength ) )
		{
			return null;
		}
		
		if ( ( result.smallestSideLength < MIN_SIDE_LENGTH ) || ( smallestCLLength < MIN_SIDE_LENGTH )
				|| ( result.largestSideLength < MIN_SIDE_LENGTH ) || ( largestCLLength < MIN_SIDE_LENGTH ) )
		{
			//System.out.println( result.smallestSideLength + " or " + smallestCLLength + " < " + MIN_SIDE_LENGTH );
			return null;
		}

		result.score += ( smallestCLLength / largestCLLength ) * CL_LENGTH_DIFFERENCE_SCORE_WEIGHT;
		result.score += ( result.smallestSideLength / result.largestSideLength )
				* SIDE_LENGTH_DIFFERENCE_SCORE_WEIGHT;
		result.score += ( MAX_TOTAL_SEGMENT_FRACTION_DIFFERENCE
				- Math.min( totalSegmentFractionDifference, MAX_TOTAL_SEGMENT_FRACTION_DIFFERENCE ) )
				* ( 1.0 / MAX_TOTAL_SEGMENT_FRACTION_DIFFERENCE ) * SEGMENT_FRACTION_DIFFERENCE_SCORE_WEIGHT;
		
		result.centerPoint = new Point( ( side1.getPt1().x + side1.getPt2().x
							+ side2.getPt1().x + side2.getPt2().x ) / 4.0, 
				 ( side1.getPt1().y + side1.getPt2().y
							+ side2.getPt1().y + side2.getPt2().y ) / 4.0);

		avgCL1Point = new Point( ( result.cl1.getPt1().x + result.cl1.getPt4().x ) / 2.0,
				( result.cl1.getPt1().y + result.cl1.getPt4().y ) / 2.0);
		avgCL2Point = new Point( ( result.cl2.getPt1().x + result.cl2.getPt4().x ) / 2.0,
				( result.cl2.getPt1().y + result.cl2.getPt4().y ) / 2.0);
		
		// Make collinear line order consistent:
		if ( ( usingHorizontalAxis && ( avgCL1Point.x > avgCL2Point.x ) )
				|| ( ( !usingHorizontalAxis ) && ( avgCL1Point.y > avgCL2Point.y ) ))
		{
			// Swap CL1 and CL2
			CollinearLine tmp = result.cl1;
			result.cl1 = result.cl2;
			result.cl2 = tmp;
		}
		
		result.hasBothRectangles_ = true;
		
		return result;
	}
	
	public static TargetCandidate generateTargetCandidate( Polygon quad, boolean cameraRunningSideways )
	{
		List<Point> quadPoints = quad.getPoints();
		
		Point pt1, pt2, pt3, pt4, pointsAverage;
		LineSegment end1, side1, end2, side2;
		
		Point end1Normal, end2Normal, endLineNormalsAverage;
		
		double side1Length, side2Length, end1Length, end2Length, 
			   sideLengthsAverage, endLengthsAverage, normScore, 
			   targetRatio, centerShiftDirection;
		
		TargetCandidate result = new TargetCandidate();		
		
		if ( quadPoints == null )
		{
			return null;
		}
		
		if ( quadPoints.size() != 4 )
		{
			return null;
		}

		pt1 = quadPoints.get(0);
		pt2 = quadPoints.get(1);
		pt3 = quadPoints.get(2);
		pt4 = quadPoints.get(3);
		
		if ( cameraRunningSideways )
		{
			if ( ( pt4.x < pt1.x ) && ( pt3.x < pt2.x ) )
			{
				if ( pt4.y > pt3.y )
				{
					end1 = new LineSegment( pt4, pt3 );
				}
				else
				{
					end1 = new LineSegment( pt3, pt4 );
				}
				
				if ( pt1.y > pt2.y )
				{
					end2 = new LineSegment( pt1, pt2 );
				}
				else
				{
					end2 = new LineSegment( pt2, pt1 );
				}
				
				side1 = new LineSegment( pt1, pt4 );
				side2 = new LineSegment( pt2, pt3 );
			}
			else
			{
				if ( pt1.y > pt2.y )
				{
					end1 = new LineSegment( pt1, pt2 );
				}
				else
				{
					end1 = new LineSegment( pt2, pt1 );
				}
				
				if ( pt4.y > pt3.y )
				{
					end2 = new LineSegment( pt4, pt3 );
				}
				else
				{
					end2 = new LineSegment( pt3, pt4 );
				}
				
				side1 = new LineSegment( pt1, pt4 );
				side2 = new LineSegment( pt2, pt3 );
			}
		}
		else
		{
			end1 = new LineSegment( pt4, pt3 );
			end2 = new LineSegment( pt1, pt2 );
			
			side1 = new LineSegment( pt1, pt4 );
			side2 = new LineSegment( pt2, pt3 );
		}
		
		// Make ends point in the same direction
		if ( Utility.dot( end1.getNormalVect(), end2.getNormalVect() ) < 0.0 )
		{
			if ( ( pt4.x < pt1.x ) && ( pt3.x < pt2.x ) )
			{
				LineSegment tmp = new LineSegment( end1 );			
				end1.set( tmp.getPt2(), tmp.getPt1() ); // Swap
			}
			else
			{
				LineSegment tmp = new LineSegment( end2 );			
				end2.set( tmp.getPt2(), tmp.getPt1() ); // Swap
			}
		}

		result.cl1 = CollinearLine.createCollinearLine( end1, quad );
		result.cl2 = CollinearLine.createCollinearLine( end2, quad );

		side1Length = side1.calculateLength();
		side2Length = side2.calculateLength();
		end1Length = end1.calculateLength();
		end2Length = end2.calculateLength();

		sideLengthsAverage = ( side1Length + side2Length ) / 2.0;
		endLengthsAverage = ( end1Length + end2Length ) / 2.0;
		
		if ( side1Length > side2Length )
		{
			result.largestSideLength = side1Length;
			result.smallestSideLength = side2Length;
		}
		else
		{
			result.largestSideLength = side2Length;
			result.smallestSideLength = side1Length;
		}

		end1Normal = end1.getNormalVect();
		end2Normal = end1.getNormalVect();
		
		endLineNormalsAverage = new Point( ( end1Normal.x + end2Normal.x ) / 2.0, ( end1Normal.y + end2Normal.y ) / 2.0 );
		pointsAverage = new Point( ( pt1.x + pt2.x + pt3.x + pt4.x ) / 4.0, ( pt1.y + pt2.y + pt3.y + pt4.y ) / 4.0 );
		
		centerShiftDirection = ( result.calculateAngleDifference() > 0.0 ) ? 1.0 : -1.0;
		
		result.centerPoint = new Point( pointsAverage.x + endLineNormalsAverage.x * endLengthsAverage * 2.0 * centerShiftDirection, 
				                        pointsAverage.y + endLineNormalsAverage.y * endLengthsAverage * 2.0 * centerShiftDirection );
		
		normScore = 1.0;
		targetRatio = sideLengthsAverage / endLengthsAverage;
		
		normScore *= ( 1.0 - Math.max(0.0, Math.min( 1.0, ( targetRatio - Polygon.PERFECT_TARGET_RECTANGLE_SIDE_RATIO )
												  / ( Polygon.MAX_TARGET_RECTANGLE_SIDE_RATIO
														  - Polygon.PERFECT_TARGET_RECTANGLE_SIDE_RATIO ) ) ) );
		
		normScore *= ( 1.0 - Math.max(0.0, Math.min( 1.0, ( Polygon.PERFECT_TARGET_RECTANGLE_SIDE_RATIO - targetRatio )
												  / ( Polygon.PERFECT_TARGET_RECTANGLE_SIDE_RATIO
														  - Polygon.MIN_TARGET_RECTANGLE_SIDE_RATIO ) ) ) );
		
		result.score = normScore * 100.0;
		
		result.hasBothRectangles_ = false;
		
		return result;
	}
	
	public void draw( Mat output )
	{
		LineSegment bridgeSegment1 = new LineSegment( cl1.getPt1(), cl2.getPt1() );
		LineSegment bridgeSegment2 = new LineSegment( cl1.getPt2(), cl2.getPt2() );
		LineSegment bridgeSegment3 = new LineSegment( cl1.getPt3(), cl2.getPt3() );
		LineSegment bridgeSegment4 = new LineSegment( cl1.getPt4(), cl2.getPt4() );

		bridgeSegment1.draw( 5, output );
		bridgeSegment2.draw( 5, output );
		bridgeSegment3.draw( 5, output );
		bridgeSegment4.draw( 5, output );

		cl1.draw( output );
		cl2.draw( output );
		
		Utility.drawPoint( getCenterPoint(), Utility.white, 5, output );
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

	public double getLargestSideLength() 
	{
		return largestSideLength;
	}

	public double getSmallestSideLength() 
	{
		return smallestSideLength;
	}

	public Point getCenterPoint() 
	{
		return centerPoint;
	}
	
	public double getNormalizedScale()
	{
		if ( hasBothRectangles_ )
		{
			return ( ( cl1.getTotalLength() + cl2.getTotalLength() ) / ( 2.0 * GEAR_TARGET_TOTAL_WIDTH_INCHES) );
		}
		
		return ( ( largestSideLength + smallestSideLength ) / ( 2.0 * GEAR_TARGET_HEIGHT_INCHES ) );
	}

	/**
	 * @return the hasBothRectangles
	 */
	public boolean hasBothRectangles() {
		return hasBothRectangles_;
	}
	
}
