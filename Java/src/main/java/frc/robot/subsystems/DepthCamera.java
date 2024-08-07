package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.orbbec.obsensor.*;

public class DepthCamera extends SubsystemBase
{
    /**
     * Object Declarations
     */
    private OBContext obContext;
    private final Object pipeLock = new Object();
    private Pipeline pipeline;

    /**
     * Subsystem Constructor
     */
    public DepthCamera()
    {
        initCamera();
    }

    /**
     * Setup the Orbbec SDK and register the device connection status callback
     */
    private void initCamera()
    {
        // Init ObContext and regist the device status listnener
        obContext = new OBContext(new DeviceChangedCallback(){
            @Override
            public void onDeviceAttach(DeviceList deviceList)
            {
                // New devices attached
                // init the pipeline
                initPipeline(deviceList);
                startStreams();

                // Release the device list.
                deviceList.close();
            }

            @Override
            public void onDeviceDetach(DeviceList deviceList)
            {
                // Devices detached
                // Release the pipeline when the device is detached
                stopStreams();
                deInitPipeline();

                // Release the device list.
                deviceList.close();
            }
        });

        DeviceList deviceList = obContext.queryDevices();
        if (null != deviceList)
        {
            if (deviceList.getDeviceCount() > 0)
            {
                initPipeline(deviceList);
                startStreams();
            }
            deviceList.close();
        }
    }

    /**
     * Release the cameara when not used anymore
     */
    private void destroyCamera()
    {
        try 
        {
            if (null != obContext)
            {
                obContext.close();
                obContext = null;
            }
        } 
        catch(Exception e)
        {
            e.printStackTrace();
        }
    }

    /**
     * Init the pipeline
     * 
     * @param deviceList new attached device list
     */
    private void initPipeline(DeviceList deviceList)
    {
        synchronized(pipeLock) 
        {
            if (null != pipeline)
            {
                pipeline.close();
            }

            // Create the new attached device, default for index 0
            try(Device device = deviceList.getDevice(0))
            {
                pipeline = new Pipeline(device);
            }
            catch(Exception e)
            {
                e.printStackTrace();
            }
        }
    }


    /**
     * Release the pipeline
     */
    private void deInitPipeline()
    {
        synchronized(pipeLock)
        {
            try
            {
                if (null != pipeline)
                {
                    pipeline.close();
                    pipeline = null;
                }
            }
            catch (Exception e)
            {
                e.printStackTrace();
            }
        }
    }

    /**
     * Start streams, this function shows for streams config and starting streams by pipeline
     */
    public void startStreams()
    {
        synchronized(pipeLock)
        {
            if (null == pipeline)
            {
                System.out.println("Camera Failed! No device connected!");
                return;
            }

            // Create the config for starting streams
            Config config = new Config();

            // Config for depth stream.
            try (StreamProfileList depthProfileList = pipeline.getStreamProfileList(SensorType.DEPTH)) 
            {
                // For example: You can also get StreamProfile this way.
                // for (int i = 0, N = depthProfileList.getStreamProfileCount(); i < N; i++) {
                //     VideoStreamProfile vsp = depthProfileList.getStreamProfile(i).as(StreamType.VIDEO);
                //     printVideoStreamProfile(vsp);
                // }

                // Filter out the StreamProfile you want based on specified arguments.
                //VideoStreamProfile depthProfile = depthProfileList.getVideoStreamProfile(640, 480, Format.Y16, 30);
                VideoStreamProfile depthProfile = depthProfileList.getStreamProfile(0).as(StreamType.VIDEO);
                if (null == depthProfile) 
                {
                    throw new OBException("get depth stream profile failed, no matched depth stream profile!");
                }
                // enable depth stream.
                config.enableStream(depthProfile);
                // release the depth stream profile.
                depthProfile.close();
            } 
            catch(Exception e) 
            {
                e.printStackTrace();
            }

            // For example: You can enable hardware depth align to color if you want.
            // config.setAlignMode(AlignMode.ALIGN_D2C_HW_ENABLE);
            
            // For example: You can enable frame synchronization if you want.
            // pipeline.enableFrameSync();

            try
            {
                pipeline.start(config, new FrameSetCallback()
                {
                    @Override
                    public void onFrameSet(FrameSet frameSet) 
                    {
                        // process the frame set
                        processFrameSet(frameSet);
                        frameSet.close();
                    }
                });
                config.close();
            }
            catch (Exception e)
            {
                e.printStackTrace();
            }
        }
    }


    /**
     * Stop all the steams.
     */
    public void stopStreams()
    {
        synchronized(pipeLock)
        {
            try
            {
                if (null != pipeline)
                {
                    pipeline.stop();
                }
            }
            catch (Exception e)
            {
                e.printStackTrace();
            }
        }
    }

    /**
     * Process the Depth Frame and printout the XYZ at a specified point.
     * 
     * @param frame DepthFrame
     */
    private void processDepthFrame(DepthFrame frame)
    {
        // Get Height and Width of Frame
        int height = frame.getHeight();
        int width = frame.getWidth();

        // Specific Pixel we are checking
        int x = 355;
        int y = 205;

        // FOV of camera in Depth mode
        double fov_w = 79.0;
        double fov_h = 62.0;

        // Get the data from the depth frame
        byte[] frameData = new byte[frame.getDataSize()];
        frame.getData(frameData);

        // Process the data
        // Depth data is 16 bit, java uses 8 bit bytes so two bytes needed for full depth data
        int depthD1 = frameData[x * width + y]; 
        int depthD2 = frameData[x * width + y + 1];

        // When adding the first byte is the MSB and masking is required
        double pZ = ((depthD1 << 8) & 0xFF00) + (depthD2 & 0xFF);

        // Calculate Theta W and H
        double theta_w = (fov_w / (double)width) * (x - (width/2.0));
        double theta_h = (fov_h / (double)height) * (y - (height/2.0));

        // Calculate X and Y distance in 3d notation
        double pX= pZ * Math.tan(Math.toRadians(theta_w));
        double pY = pZ * Math.tan(Math.toRadians(theta_h));

        // Build demo string to display data on a dashboard
        StringBuilder sb = new StringBuilder()
        .append("x: " + Math.round(pX) + "mm, y: " + Math.round(pY) + "mm, z: " + pZ+ "mm");
        SmartDashboard.putString("Data: ", sb.toString());
    }

    /**
     * Process the frame set
     * 
     * @param frameSet new frame set
     */
    private void processFrameSet(FrameSet frameSet)
    {
        // Get the depth frame in frameSet.
        try (DepthFrame depthFrame = frameSet.getFrame(FrameType.DEPTH)) 
        {
            if (null != depthFrame) 
            {
                processDepthFrame(depthFrame);
            }
        } 
        catch(Exception e) 
        {
            e.printStackTrace();
        }
    }
}