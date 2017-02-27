package first.frc.team2077.season2017.vision.trackers;

import org.opencv.core.Mat;
import org.opencv.core.Point;

public class Utility 
{
	// For acos(x): Index = floor((x+1)*arcCosTable.length/2)
	private static double[] arcCosTable = 
		{
				180, 171.891, 168.522, 165.93, 163.74, 161.805, 160.052, 
				158.435, 156.926, 155.505, 154.158, 152.873, 151.642, 150.459, 
				149.317, 148.212, 147.14, 146.099, 145.085, 144.096, 143.13, 
				142.186, 141.261, 140.354, 139.464, 138.59, 137.732, 136.887, 
				136.055, 135.235, 134.427, 133.63, 132.844, 132.067, 131.3, 
				130.542, 129.792, 129.05, 128.316, 127.59, 126.87, 126.157, 
				125.451, 124.75, 124.056, 123.367, 122.684, 122.006, 121.332, 
				120.664, 120, 119.341, 118.686, 118.034, 117.387, 116.744, 
				116.104, 115.468, 114.835, 114.205, 113.578, 112.955, 112.334, 
				111.716, 111.1, 110.487, 109.877, 109.269, 108.663, 108.059, 
				107.458, 106.858, 106.26, 105.664, 105.07, 104.478, 103.887, 
				103.297, 102.709, 102.122, 101.537, 100.953, 100.37, 99.7879, 
				99.207, 98.627, 98.0479, 97.4697, 96.8922, 96.3154, 95.7393, 
				95.1637, 94.5886, 94.0141, 93.4399, 92.8661, 92.2925, 91.7192, 
				91.1461, 90.573, 90.0001, 89.4271, 88.8541, 88.2809, 87.7076, 
				87.1341, 86.5603, 85.9861, 85.4115, 84.8365, 84.2609, 83.6848, 
				83.108, 82.5305, 81.9522, 81.3731, 80.7932, 80.2122, 79.6303, 
				79.0473, 78.4631, 77.8777, 77.291, 76.703, 76.1135, 75.5226, 
				74.93, 74.3358, 73.7399, 73.1421, 72.5425, 71.9408, 71.3371, 
				70.7313, 70.1232, 69.5127, 68.8999, 68.2844, 67.6664, 67.0456, 
				66.4219, 65.7952, 65.1655, 64.5325, 63.8962, 63.2564, 62.6129, 
				61.9658, 61.3146, 60.6595, 60.0001, 59.3362, 58.6678, 57.9946, 
				57.3164, 56.633, 55.9442, 55.2498, 54.5495, 53.843, 53.1301, 
				52.4105, 51.6839, 50.9499, 50.2082, 49.4584, 48.7002, 47.933, 
				47.1564, 46.3699, 45.573, 44.7651, 43.9456, 43.1136, 42.2686, 
				41.4097, 40.5358, 39.6461, 38.7395, 37.8145, 36.8699, 35.9041, 
				34.9152, 33.9013, 32.8599, 31.7884, 30.6834, 29.5414, 28.3577, 
				27.1268, 25.842, 24.4947, 23.0739, 21.5652, 19.9485, 18.1949, 
				16.2602, 14.0699, 11.4784, 8.10962, 0
		};

	public static double[] white = { 255.0, 255.0, 255.0 };
	public static double[] red = { 0.0, 0.0, 255.0 };
	public static double[] green = { 0.0, 255.0, 0.0 };
	public static double[] blue = { 255.0, 0.0, 0.0 };
	public static double[] yellow = { 0.0, 255.0, 255.0 };
	
	public static class MutableDouble 
	{
		private double value;
		
		public MutableDouble( double value ) 
		{
			this.value = value;
		}
		
		public double getValue() 
		{
			return this.value;
		}
		
		public void setValue( double value ) 
		{
			this.value = value;
		}
	}
	
	public static double fastArcCosine( double x )
	{
		int index = ( int )( ( x + 1.0 ) * ( ( arcCosTable.length - 1.0 ) / 2.0 ) );
		
		return arcCosTable[ Math.max( 0, Math.min( index, arcCosTable.length - 1 ) ) ];
	}
	
	public static double[] hueToRGB( double hue )
	{
		double rgb[] = new double[3];
		double wrappedHue = mod( hue, 360 );
		
		double colorVal = ( 1.0 - Math.abs( mod( wrappedHue / 60.0, 2 ) - 1.0 ) );
		
		if ( wrappedHue >= 300.0 )
		{
			rgb[2] = 255.0;
			rgb[1] = 0.0;
			rgb[0] = colorVal * 255.0;
		}
		else if ( wrappedHue >= 240.0 )
		{
			rgb[2] = colorVal * 255.0;
			rgb[1] = 0.0;
			rgb[0] = 255.0;
		}
		else if ( wrappedHue >= 180.0 )
		{
			rgb[2] = 0.0;
			rgb[1] = colorVal * 255.0;
			rgb[0] = 255.0;
		}
		else if ( wrappedHue >= 120.0 )
		{
			rgb[2] = 0.0;
			rgb[1] = 255.0;
			rgb[0] = colorVal * 255.0;
		}
		else if ( wrappedHue >= 60.0 )
		{
			rgb[2] = colorVal * 255.0;
			rgb[1] = 255.0;
			rgb[0] = 0.0;
		}
		else
		{
			rgb[2] = 255.0;
			rgb[1] = colorVal * 255.0;
			rgb[0] = 0.0;
		}
		
		return rgb;
	}
	
	public static double average( double[] vals )
	{
		double sum = 0.0;
		for ( double val : vals )
		{
			sum += val;
		}
		
		return ( sum / vals.length );
	}
	
	public static boolean approxCompare( double d1, double d2 )
	{
		return ( Math.abs( d1 - d2 ) < 0.01 );
	}
	
	public static int mod(int x, int n)
	{
		return (int)mod((double)x, (double)n);
	}
	
	public static double mod(double x, double n)
	{
		if ( n == 0 )
		{
			return 0.0;
		}
		
		double r = x % n;
		if (r < 0.0)
		{
		    r += n;
		}
		
		return r;
	}
	
	public static double dot( double x1, double y1, double x2, double y2 )
	{
		return ( ( x1 * x2 ) + ( y1 * y2 ) );
	}
	
	public static double normDot( double x1, double y1, double x2, double y2 )
	{
		double vect1Length = Math.sqrt( ( x1 * x1 ) + ( y1 * y1 ) );
		double vect2Length = Math.sqrt( ( x2 * x2 ) + ( y2 * y2 ) );
		
		if ( ( vect1Length < 0.001 ) || ( vect2Length < 0.001 )
				|| ( vect1Length == Double.NaN ) || ( vect2Length == Double.NaN ) )
		{
			return 0.0;
		}

		double newX1 = x1 / vect1Length;
		double newY1 = y1 / vect1Length;
		double newX2 = x2 / vect2Length;
		double newY2 = y2 / vect2Length;
		
		return dot( newX1, newY1, newX2, newY2 );
	}
	
	public static double crossProductMagnitude( double x1, double y1, double x2, double y2 )
	{
		return ((x1*y2)-(y1*x2));
	}

	public static double getTurn( Point pt1, Point pt2, Point pt3 )
	{
		return getTurn( pt2.x - pt1.x, pt2.y - pt1.y, pt3.x - pt2.x, pt3.y - pt2.y );
	}
	
	public static double getTurn( double x1, double y1, double x2, double y2 )
	{
		return ( 1.0 - normDot( x1, y1, x2, y2 ) );
	}
	
	/**
	 * The U scalar is a scalar along ls2 that locates the intersect point with ls1.
	 * Its division components can help to determine collinearity and parallelism.
	 * @param ls1 First line segment.
	 * @param ls2 Second line segment.
	 * @param dividend The dividend in ( U = dividend / divisor ).
	 * @param divisor The divisor in ( U = dividend / divisor ).
	 * @return True if dividend and divisor were set. False due to invalid parameters.
	 */
	public static boolean getUScalarDivisionComponents( LineSegment ls1, LineSegment ls2, 
			MutableDouble dividend, MutableDouble divisor )
	{
		if ( ( ls1 == null ) || ( ls2 == null )
				|| ( dividend == null ) || ( divisor == null ) )
		{
			return false;
		}
		
		if ( ( ls1.getPt1() == null ) || ( ls1.getPt2() == null )
				|| ( ls2.getPt1() == null ) || ( ls2.getPt2() == null ) )
		{
			return false;
		}
		
		Point p = ls1.getPt1();
		Point q = ls2.getPt1();
		Point r = new Point( ls1.getPt2().x - ls1.getPt1().x, ls1.getPt2().y - ls1.getPt1().y );
		Point s = new Point( ls2.getPt2().x - ls2.getPt1().x, ls2.getPt2().y - ls2.getPt1().y );
		
		dividend.setValue( Utility.crossProductMagnitude( 
				q.x - p.x, q.y - p.y, r.x, r.y ) ); // ( q - p ) x r
		divisor.setValue( Utility.crossProductMagnitude( 
				r.x, r.y, s.x, s.y ) ); // r x s
		
		return true;
	}
	
	public static double getPointsDistance( Point pt1, Point pt2 )
	{
		double dx = pt2.x - pt1.x;
		double dy = pt2.y - pt1.y;
		
		return Math.sqrt( dx*dx + dy*dy );
	}
	
	public static Point getAveragePoint( Point[] points )
	{
		if ( points == null )
		{
			return null;
		}
		
		if ( points.length > 0 )
		{
			Point result = new Point();
			
			for ( int i = 0; i < points.length; ++i )
			{
				result.x += points[ i ].x;
				result.y += points[ i ].y;
			}

			result.x /= points.length;
			result.y /= points.length;
			
			return result;
		}
		
		return null;
	}
	
	public static Point getAveragePoint( Point pt1, Point pt2 )
	{
		Point points[] = { pt1, pt2 };
		
		return getAveragePoint( points );
	}
	
	public static Point projectPointOnLine( Point point, LineSegment line, MutableDouble projectionValue )
	{
		Point normalVector = line.getNormalVect();
		double dotValue = dot( point.x - line.getPt1().x, point.y - line.getPt1().y,
				line.getPt2().x - line.getPt1().x, line.getPt2().y - line.getPt1().y ) / line.calculateLength();
		
		projectionValue.setValue( dotValue );
		
		return new Point( line.getPt1().x + ( normalVector.x * dotValue ), 
						  line.getPt1().y + ( normalVector.y * dotValue ) );
	}
	
	public static LineSegment projectLineSegmentOnLine( LineSegment toProject, LineSegment toThis )
	{
		LineSegment result = null;
		MutableDouble pt1ProjectValue = new MutableDouble( 0.0 );
		MutableDouble pt2ProjectValue = new MutableDouble( 0.0 );

		Point pt1 = Utility.projectPointOnLine( toProject.getPt1(), toThis, pt1ProjectValue );
		Point pt2 = Utility.projectPointOnLine( toProject.getPt2(), toThis, pt2ProjectValue );
		
		if ( pt1ProjectValue.getValue() < pt2ProjectValue.getValue() )
		{
			return new LineSegment( pt1, pt2 );
		}
		
		return new LineSegment( pt2, pt1 );
	}
	
	public static double getLowestAngleBetween( LineSegment lineSegment1, LineSegment lineSegment2 )
	{
		double angle1 = lineSegment1.calculateAngle();
		double angle2 = lineSegment2.calculateAngle();
		
		double rawDifference = Math.abs( angle1 - angle2 );
		
		if ( rawDifference > 270 ) // >270 & <=360
		{
			return 90.0 - ( rawDifference - 270.0 );
		}
		else if ( rawDifference > 180 ) // >180 & <=270
		{
			return rawDifference - 180.0;
		}
		else if ( rawDifference > 90 ) // >90 & <=180
		{
			return 90.0 - ( rawDifference - 90.0 );
		}
		
		return rawDifference; // 0 .. 90
	}
	
	public static double determinant( Point pt1, Point pt2, Point pt3 )
	{
		double result =  pt1.x*pt2.y + pt2.x*pt3.y + pt3.x*pt1.y - pt1.x*pt3.y - pt2.x*pt1.y - pt3.x*pt2.y;
		
		//System.out.println( result );
		
		return result;
	}
	
    /**
     * (Copied from java.awt.geom.Line2D)
     * 
     * Computes twice the (signed) area of the triangle defined by the three
     * points.  This method is used for intersection testing.
     * 
     * @param x1  the x-coordinate of the first point.
     * @param y1  the y-coordinate of the first point.
     * @param x2  the x-coordinate of the second point.
     * @param y2  the y-coordinate of the second point.
     * @param x3  the x-coordinate of the third point.
     * @param y3  the y-coordinate of the third point.
     * 
     * @return Twice the area.
     */
    private static double area2(double x1, double y1,
                               double x2, double y2,
                               double x3, double y3) 
    {
      return (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);    
    }
    
    /**
     * (Copied from java.awt.geom.Line2D)
     * 
     * Returns <code>true</code> if (x3, y3) lies between (x1, y1) and (x2, y2),
     * and false otherwise,  This test assumes that the three points are 
     * collinear, and is used for intersection testing.
     * 
     * @param x1  the x-coordinate of the first point.
     * @param y1  the y-coordinate of the first point.
     * @param x2  the x-coordinate of the second point.
     * @param y2  the y-coordinate of the second point.
     * @param x3  the x-coordinate of the third point.
     * @param y3  the y-coordinate of the third point.
     * 
     * @return A boolean.
     */
    private static boolean between(double x1, double y1, 
                                  double x2, double y2, 
                                  double x3, double y3) 
    {
      if (x1 != x2) {
        return (x1 <= x3 && x3 <= x2) || (x1 >= x3 && x3 >= x2);   
      }
      else {
        return (y1 <= y3 && y3 <= y2) || (y1 >= y3 && y3 >= y2);   
      }
    }
	
    /**
     * (Copied from java.awt.geom.Line2D)
     * 
     * Test if the line segment (x1,y1)-&gt;(x2,y2) intersects the line segment 
     * (x3,y3)-&gt;(x4,y4).
     *
     * @param x1 the first x coordinate of the first segment
     * @param y1 the first y coordinate of the first segment 
     * @param x2 the second x coordinate of the first segment
     * @param y2 the second y coordinate of the first segment
     * @param x3 the first x coordinate of the second segment
     * @param y3 the first y coordinate of the second segment
     * @param x4 the second x coordinate of the second segment
     * @param y4 the second y coordinate of the second segment
     * @return true if the segments intersect
     */
    public static boolean linesIntersect(double x1, double y1,
                                        double x2, double y2,
                                        double x3, double y3,
                                        double x4, double y4)
    {
      double a1, a2, a3, a4;
    
      // deal with special cases
      if ((a1 = area2(x1, y1, x2, y2, x3, y3)) == 0.0) 
      {
        // check if p3 is between p1 and p2 OR
        // p4 is collinear also AND either between p1 and p2 OR at opposite ends
        if (between(x1, y1, x2, y2, x3, y3)) 
        {
          return true;
        }
        else 
        {
          if (area2(x1, y1, x2, y2, x4, y4) == 0.0) 
          {
            return between(x3, y3, x4, y4, x1, y1) 
                   || between (x3, y3, x4, y4, x2, y2);
          }
          else {
            return false;
          }
        }
      }
      else if ((a2 = area2(x1, y1, x2, y2, x4, y4)) == 0.0) 
      {
        // check if p4 is between p1 and p2 (we already know p3 is not
        // collinear)
        return between(x1, y1, x2, y2, x4, y4);
      }
    
      if ((a3 = area2(x3, y3, x4, y4, x1, y1)) == 0.0) {
        // check if p1 is between p3 and p4 OR
        // p2 is collinear also AND either between p1 and p2 OR at opposite ends
        if (between(x3, y3, x4, y4, x1, y1)) {
          return true;
        }
        else {
          if (area2(x3, y3, x4, y4, x2, y2) == 0.0) {
            return between(x1, y1, x2, y2, x3, y3) 
                   || between (x1, y1, x2, y2, x4, y4);
          }
          else {
            return false;
          }
        }
      }
      else if ((a4 = area2(x3, y3, x4, y4, x2, y2)) == 0.0) {
        // check if p2 is between p3 and p4 (we already know p1 is not
        // collinear)
        return between(x3, y3, x4, y4, x2, y2);
      }
      else {  // test for regular intersection
        return ((a1 > 0.0) ^ (a2 > 0.0)) && ((a3 > 0.0) ^ (a4 > 0.0));
      } 
    }

	public static void drawVertex( Vertex vertex, double[] color, int size, Mat camFrame )
	{
		drawPoint( vertex.getPoint(), color, size, camFrame );
	}
	
	public static void drawPoint( Point point, double[] color, int size, Mat camFrame )
	{
		for ( int x = (int) (point.x - size); x <= point.x + size; ++x )
		{
			for ( int y = (int) (point.y - size); y <= point.y + size; ++y )
			{
				if ( ( y >= 0 ) && ( y < camFrame.height() ) && ( x >= 0 ) && ( x < camFrame.width() ) )
				camFrame.put( y, x, color );
			}
		}
	}
}
