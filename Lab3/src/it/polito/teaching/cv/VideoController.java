package it.polito.teaching.cv;

import java.io.ByteArrayInputStream;
import java.util.AbstractList;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

import first.frc.team2077.season2017.vision.trackers.GearLiftTargetTracking;
import javafx.application.Platform;
import javafx.beans.property.ObjectProperty;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Slider;
import javafx.scene.control.TextField;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.text.Text;

/**
 * The controller associated with the only view of our application. The
 * application logic is implemented here. It handles the button for
 * starting/stopping the camera, the acquired video stream, the relative
 * controls and the histogram creation.
 * 
 * @author <a href="mailto:luigi.derussis@polito.it">Luigi De Russis</a>
 * @author <a href="mailto:christian.g.reese96@gmail.com">Christian Reese</a>
 * @version 1.1 (2015-10-20)
 * @since 1.0 (2013-11-20)
 * 		
 */
public class VideoController
{	
	// the FXML button
	@FXML
	private Button button;
	//// the FXML grayscale checkbox
	//@FXML
	//private CheckBox grayscale;
	// the FXML logo checkbox
	@FXML
	private CheckBox logoCheckBox;
	@FXML
	private CheckBox thresholdCheckBox;
	// the FXML grayscale checkbox
	@FXML
	private ImageView histogram;
	// the FXML area for showing the current frame
	@FXML
	private ImageView currentFrame;

	@FXML
	private Slider hueMinSldr;
	@FXML
	private Slider hueMaxSldr;
	@FXML
	private Slider satMinSldr;
	@FXML
	private Slider satMaxSldr;
	@FXML
	private Slider lumMinSldr;
	@FXML
	private Slider lumMaxSldr;

	@FXML
	private Text hueMinValLbl;
	@FXML
	private Text hueMaxValLbl;
	@FXML
	private Text satMinValLbl;
	@FXML
	private Text satMaxValLbl;
	@FXML
	private Text lumMinValLbl;
	@FXML
	private Text lumMaxValLbl;
	
	@FXML 
	private TextField cameraNumberInput;

	// a timer for acquiring the video stream
	private ScheduledExecutorService frameGrabTimer;
	// a timer for updating the GUI
	private ScheduledExecutorService guiTimer;
	// the OpenCV object that realizes the video capture
	private VideoCapture capture;
	// a flag to change the button behavior
	private boolean cameraActive;
	// the logo to be loaded
	private Mat logo;
	
	/**
	 * Initialize method, automatically called by @{link FXMLLoader}
	 */
	public void initialize()
	{
		Video mainClass = Video.getInstance();
		
		this.capture = new VideoCapture();
		this.cameraActive = false;
		
		// update the GUI on a set interval
		Runnable guiUpdater = new Runnable() {
			
			private boolean initialized = false;
			
			@Override
			public void run()
			{
				Platform.runLater( () -> { 
					
						if ( initialized )
						{
							Video.Config configProxy = Video.getInstance().getConfig();
							
							if ( ( hueMinValLbl != null ) && ( hueMinSldr != null ) ) 
								hueMinValLbl.setText( Integer.toString( (int)hueMinSldr.getValue() ) ); 
							if ( ( hueMaxValLbl != null ) && ( hueMaxSldr != null ) ) 
								hueMaxValLbl.setText( Integer.toString( (int)hueMaxSldr.getValue() ) ); 
							if ( ( satMinValLbl != null ) && ( satMinSldr != null ) ) 
								satMinValLbl.setText( Integer.toString( (int)satMinSldr.getValue() ) ); 
							if ( ( satMaxValLbl != null ) && ( satMaxSldr != null ) ) 
								satMaxValLbl.setText( Integer.toString( (int)satMaxSldr.getValue() ) ); 
							if ( ( lumMinValLbl != null ) && ( lumMinSldr != null ) ) 
								lumMinValLbl.setText( Integer.toString( (int)lumMinSldr.getValue() ) ); 
							if ( ( lumMaxValLbl != null ) && ( lumMaxSldr != null ) ) 
								lumMaxValLbl.setText( Integer.toString( (int)lumMaxSldr.getValue() ) ); 
							
							if ( configProxy != null )
							{
								configProxy.minHueStartVal = hueMinSldr.getValue();
								configProxy.maxHueStartVal = hueMaxSldr.getValue();
								configProxy.minSatStartVal = satMinSldr.getValue();
								configProxy.maxSatStartVal = satMaxSldr.getValue();
								configProxy.minLumStartVal = lumMinSldr.getValue();
								configProxy.maxLumStartVal = lumMaxSldr.getValue();
							}
						}
						else
						{
							Video.Config configProxy = Video.getInstance().getConfig();
							
							if ( configProxy != null )
							{
								hueMinSldr.setValue( configProxy.minHueStartVal );
								hueMaxSldr.setValue( configProxy.maxHueStartVal );
								satMinSldr.setValue( configProxy.minSatStartVal );
								satMaxSldr.setValue( configProxy.maxSatStartVal );
								lumMinSldr.setValue( configProxy.minLumStartVal );
								lumMaxSldr.setValue( configProxy.maxLumStartVal );
								
								initialized = true;
							}
						}
					} );
			}
		};
		
		this.guiTimer = Executors.newSingleThreadScheduledExecutor();
		this.guiTimer.scheduleAtFixedRate(guiUpdater, 0, 33, TimeUnit.MILLISECONDS);
		
		if ( mainClass != null )
		{
			mainClass.setGUIUpdateTimer(guiTimer);
		}
	}
	
	/**
	 * The action triggered by pushing the button on the GUI
	 */
	@FXML
	protected void startCamera()
	{
		Video mainClass = Video.getInstance();
		
		// set a fixed width for the frame
		//this.currentFrame.setFitWidth(600);
		// preserve image ratio
		this.currentFrame.setPreserveRatio(true);
		
		if (!this.cameraActive)
		{
			try 
			{
				int cameraIndex = Integer.parseInt(cameraNumberInput.getText());
				
				// start the video capture
				this.capture.open( cameraIndex );
				capture.set(Videoio.CAP_PROP_EXPOSURE , -8 );
				//capture.set(Videoio.CAP_PROP_WHITE_BALANCE_BLUE_U , 8000 );
				//capture.set(Videoio.CV_CAP_PROP_SETTINGS , 1 );
				
				// is the video stream available?
				if (this.capture.isOpened())
				{
					this.cameraActive = true;
					
					// grab a frame every 33 ms (30 frames/sec)
					Runnable frameGrabber = new Runnable() {
						
						//boolean propertiesSet = false;
						
						@Override
						public void run()
						{
							Image imageToShow = grabFrame();
							//currentFrame.setImage(imageToShow);
							onFXThread(currentFrame.imageProperty(), imageToShow);
							
							//if ( !propertiesSet )
							//{
							//	capture.set(Videoio.CAP_PROP_EXPOSURE , -8 );
							//	propertiesSet = true;
							//}

							//System.out.println("CAP_PROP_WHITE_BALANCE_BLUE_U: " + capture.get(Videoio.CAP_PROP_WHITE_BALANCE_BLUE_U ));
							//System.out.println("CAP_PROP_WHITE_BALANCE_RED_V: " + capture.get(Videoio.CAP_PROP_WHITE_BALANCE_RED_V ));
							//System.out.println("CAP_PROP_EXPOSURE: " + capture.get(Videoio.CAP_PROP_EXPOSURE ));
						}
					};
					
					this.frameGrabTimer = Executors.newSingleThreadScheduledExecutor();
					this.frameGrabTimer.scheduleAtFixedRate(frameGrabber, 0, 33, TimeUnit.MILLISECONDS);
					
					// update the button content
					this.button.setText("Stop Camera");
					
					if ( mainClass != null )
					{
						mainClass.setFrameGrabTimer(frameGrabTimer);
						mainClass.setVideoCapture(capture);
					}
				}
				else
				{
					// log the error
					System.err.println("Impossible to open the camera connection...");
				}
			} 
			catch( NumberFormatException ex )
			{
				
			}
		}
		else
		{
			// the camera is not active at this point
			this.cameraActive = false;
			// update again the button content
			this.button.setText("Start Camera");
			
			// stop the timer
			try
			{
				this.frameGrabTimer.shutdown();
				this.frameGrabTimer.awaitTermination(33, TimeUnit.MILLISECONDS);
			}
			catch (InterruptedException e)
			{
				// log the exception
				System.err.println("Exception in stopping the frame capture, trying to release the camera now... " + e);
			}
			
			// release the camera
			this.capture.release();
			// clean the frame
			this.currentFrame.setImage(null);
		}
	}
	
	/**
	 * The action triggered by selecting/deselecting the logo checkbox
	 */
	@FXML
	protected void loadLogo()
	{
		if (logoCheckBox.isSelected())
		{
			// read the logo only when the checkbox has been selected
			this.logo = Imgcodecs.imread("resources/Poli.png");
		}
	}
	
	/**
	 * Get a frame from the opened video stream (if any)
	 * 
	 * @return the {@link Image} to show
	 */
	private Image grabFrame()
	{
		// init everything
		Image imageToShow = null;
		Mat frame = new Mat();
		
		// check if the capture is open
		if (this.capture.isOpened())
		{
			try
			{
				// read the current frame
				this.capture.read(frame);
				
				// if the frame is not empty, process it
				if (!frame.empty())
				{
					// show the histogram
					this.showHistogram(frame);
										
					Mat output = GearLiftTargetTracking.processMat( frame, 
										capture.get( Videoio.CV_CAP_PROP_FRAME_WIDTH ), 
										capture.get( Videoio.CV_CAP_PROP_FRAME_HEIGHT ), 
										hueMinSldr.getValue(),
										hueMaxSldr.getValue(),
										satMinSldr.getValue(),
										satMaxSldr.getValue(),
										lumMinSldr.getValue(),
										lumMaxSldr.getValue(),
										thresholdCheckBox.isSelected() );
					
					// convert the Mat object (OpenCV) to Image (JavaFX)
					imageToShow = mat2Image( output );
				}
				
			}
			catch (Exception e)
			{
				// log the error
				System.err.println("Exception during the frame elaboration: " + e);
				e.printStackTrace();
			}
		}
		
		return imageToShow;
	}
	
	/**
	 * Compute and show the histogram for the given {@link Mat} image
	 * 
	 * @param frame
	 *            the {@link Mat} image for which compute the histogram
	 */
	private void showHistogram(Mat frame)
	{
		// split the frames in multiple images
		List<Mat> images = new ArrayList<Mat>();
		Core.split(frame, images);
		
		// set the number of bins at 256
		MatOfInt histSize = new MatOfInt(256);
		// only one channel
		MatOfInt channels = new MatOfInt(0);
		// set the ranges
		MatOfFloat histRange = new MatOfFloat(0, 256);
		
		// compute the histograms for the B, G and R components
		Mat hist_b = new Mat();
		Mat hist_g = new Mat();
		Mat hist_r = new Mat();
		
		// B component or gray image
		Imgproc.calcHist(images.subList(0, 1), channels, new Mat(), hist_b, histSize, histRange, false);
		
		// G and R components (if the image is not in gray scale)
		//if (!gray)
		{
			Imgproc.calcHist(images.subList(1, 2), channels, new Mat(), hist_g, histSize, histRange, false);
			Imgproc.calcHist(images.subList(2, 3), channels, new Mat(), hist_r, histSize, histRange, false);
		}
		
		// draw the histogram
		int hist_w = 150; // width of the histogram image
		int hist_h = 150; // height of the histogram image
		int bin_w = (int) Math.round(hist_w / histSize.get(0, 0)[0]);
		
		Mat histImage = new Mat(hist_h, hist_w, CvType.CV_8UC3, new Scalar(0, 0, 0));
		// normalize the result to [0, histImage.rows()]
		Core.normalize(hist_b, hist_b, 0, histImage.rows(), Core.NORM_MINMAX, -1, new Mat());
		
		// for G and R components
		//if (!gray)
		{
			Core.normalize(hist_g, hist_g, 0, histImage.rows(), Core.NORM_MINMAX, -1, new Mat());
			Core.normalize(hist_r, hist_r, 0, histImage.rows(), Core.NORM_MINMAX, -1, new Mat());
		}
		
		// effectively draw the histogram(s)
		for (int i = 1; i < histSize.get(0, 0)[0]; i++)
		{
			// B component or gray image
			Imgproc.line(histImage, new Point(bin_w * (i - 1), hist_h - Math.round(hist_b.get(i - 1, 0)[0])),
					new Point(bin_w * (i), hist_h - Math.round(hist_b.get(i, 0)[0])), new Scalar(255, 0, 0), 2, 8, 0);
			// G and R components (if the image is not in gray scale)
			//if (!gray)
			{
				Imgproc.line(histImage, new Point(bin_w * (i - 1), hist_h - Math.round(hist_g.get(i - 1, 0)[0])),
						new Point(bin_w * (i), hist_h - Math.round(hist_g.get(i, 0)[0])), new Scalar(0, 255, 0), 2, 8,
						0);
				Imgproc.line(histImage, new Point(bin_w * (i - 1), hist_h - Math.round(hist_r.get(i - 1, 0)[0])),
						new Point(bin_w * (i), hist_h - Math.round(hist_r.get(i, 0)[0])), new Scalar(0, 0, 255), 2, 8,
						0);
			}
		}
		
		// display the histogram...
		Image histImg = mat2Image(histImage);
		//this.histogram.setImage(histImg);
		onFXThread(this.histogram.imageProperty(), histImg);
	}
	
	/**
	 * Convert a Mat object (OpenCV) in the corresponding Image for JavaFX
	 * 
	 * @param frame
	 *            the {@link Mat} representing the current frame
	 * @return the {@link Image} to show
	 */
	private Image mat2Image(Mat frame)
	{
		// create a temporary buffer
		MatOfByte buffer = new MatOfByte();
		// encode the frame in the buffer, according to the PNG format
		Imgcodecs.imencode(".png", frame, buffer);
		// build and return an Image created from the image encoded in the
		// buffer
		return new Image(new ByteArrayInputStream(buffer.toArray()));
	}
	
	/**
	 * Generic method for putting element running on a non-JavaFX thread on the
	 * JavaFX thread, to properly update the UI
	 * 
	 * @param property
	 *            a {@link ObjectProperty}
	 * @param value
	 *            the value to set for the given {@link ObjectProperty}
	 */
	private static <T> void onFXThread(final ObjectProperty<T> property, final T value)
	{
		Platform.runLater(() -> {
			property.set(value);
		});
	}

	
}
