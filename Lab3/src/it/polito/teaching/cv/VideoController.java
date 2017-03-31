package it.polito.teaching.cv;

import java.io.ByteArrayInputStream;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
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
	// threshold checkbox
	@FXML
	private CheckBox thresholdCheckBox;
	// the FXML area for showing the current frame
	@FXML
	private ImageView currentFrame;

	@FXML
	private Slider min1Sldr;
	@FXML
	private Slider max1Sldr;
	@FXML
	private Slider min2Sldr;
	@FXML
	private Slider max2Sldr;
	@FXML
	private Slider min3Sldr;
	@FXML
	private Slider max3Sldr;

	@FXML
	private Text min1ValLbl;
	@FXML
	private Text max1ValLbl;
	@FXML
	private Text min2ValLbl;
	@FXML
	private Text max2ValLbl;
	@FXML
	private Text min3ValLbl;
	@FXML
	private Text max3ValLbl;
	
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
							
							if ( ( min1ValLbl != null ) && ( min1Sldr != null ) ) 
								min1ValLbl.setText( Integer.toString( (int)min1Sldr.getValue() ) ); 
							if ( ( max1ValLbl != null ) && ( max1Sldr != null ) ) 
								max1ValLbl.setText( Integer.toString( (int)max1Sldr.getValue() ) ); 
							if ( ( min3ValLbl != null ) && ( min3Sldr != null ) ) 
								min3ValLbl.setText( Integer.toString( (int)min3Sldr.getValue() ) ); 
							if ( ( max3ValLbl != null ) && ( max3Sldr != null ) ) 
								max3ValLbl.setText( Integer.toString( (int)max3Sldr.getValue() ) ); 
							if ( ( min2ValLbl != null ) && ( min2Sldr != null ) ) 
								min2ValLbl.setText( Integer.toString( (int)min2Sldr.getValue() ) ); 
							if ( ( max2ValLbl != null ) && ( max2Sldr != null ) ) 
								max2ValLbl.setText( Integer.toString( (int)max2Sldr.getValue() ) ); 
							
							if ( configProxy != null )
							{
								configProxy.minHueStartVal = min1Sldr.getValue();
								configProxy.maxHueStartVal = max1Sldr.getValue();
								configProxy.minSatStartVal = min3Sldr.getValue();
								configProxy.maxSatStartVal = max3Sldr.getValue();
								configProxy.minLumStartVal = min2Sldr.getValue();
								configProxy.maxLumStartVal = max2Sldr.getValue();
							}
						}
						else
						{
							Video.Config configProxy = Video.getInstance().getConfig();
							
							if ( configProxy != null )
							{
								min1Sldr.setValue( configProxy.minHueStartVal );
								max1Sldr.setValue( configProxy.maxHueStartVal );
								min3Sldr.setValue( configProxy.minSatStartVal );
								max3Sldr.setValue( configProxy.maxSatStartVal );
								min2Sldr.setValue( configProxy.minLumStartVal );
								max2Sldr.setValue( configProxy.maxLumStartVal );
								
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
		
		if (!this.cameraActive)
		{
			int cameraIndex = Integer.parseInt(cameraNumberInput.getText());
			
			// start the video capture
			this.capture.open( cameraIndex );
			capture.set(Videoio.CAP_PROP_EXPOSURE , -7 );
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
						currentFrame.setImage(imageToShow);
						onFXThread(currentFrame.imageProperty(), imageToShow);
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
				//frame = Imgcodecs.imread("resources/img.png");
				
				// if the frame is not empty, process it
				if (!frame.empty())
				{
					Mat output = GearLiftTargetTracking.processMat( frame, 
										capture.get( Videoio.CV_CAP_PROP_FRAME_WIDTH ), 
										capture.get( Videoio.CV_CAP_PROP_FRAME_HEIGHT ), 
										min1Sldr.getValue(),
										max1Sldr.getValue(),
										min2Sldr.getValue(),
										max2Sldr.getValue(),
										min3Sldr.getValue(),
										max3Sldr.getValue(),
										thresholdCheckBox.isSelected() );
					
					// convert the Mat object (OpenCV) to Image (JavaFX)
					imageToShow = mat2Image( output );
					
					output.release();
				}
				
				frame.release();
				
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
		
		Image result = new Image(new ByteArrayInputStream(buffer.toArray()));
		
		buffer.release();
		
		// build and return an Image created from the image encoded in the
		// buffer
		return result;
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
