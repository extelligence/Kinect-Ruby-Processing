require 'ruby-processing'
# require File.expand_path(File.dirname(__FILE__)) + "/library/openkinect/openkinect.jar"

class Sketch < Processing::App
  load_library "openkinect"
  include_package 'org.openkinect'
  include_package 'org.openkinect.processing'
  # include_class 'org.openkinect.processing.Kinect'
  # import 'org.openkinect.processing.Kinect'

  attr_accessor :kinect, :depth, :rgb, :ir, :deg
  
  # @kinect = nil
  # @depth = true
  # @rgb = false
  # @ir = false
  
  # @deg = 15  # Start at 15 degrees
  
  def setup()
    # current = java.lang.System.getProperty("java.library.path")
    # p current
    # java.lang.System.setProperty("java.library.path",
    # "#{current}:#{File.expand_path('library/openkinect')}");
    # p current = java.lang.System.getProperty("java.library.path")
    
    @deg = 15
    
    size(1280,520);
    @kinect = Kinect.new(self)
    @kinect.start()
    @kinect.enableDepth(@depth);
    @kinect.enableRGB(@rgb);
    @kinect.enableIR(@ir);
    @kinect.tilt(@deg);
  end
 
  def draw()
    background(0);

    image(@kinect.getVideoImage(),0,0);
    image(@kinect.getDepthImage(),640,0);
    fill(255);
    text("RGB/IR FPS: #{@kinect.getVideoFPS()}        Camera tilt: #{@deg} degrees",10,495);
    text("DEPTH FPS: #{@kinect.getDepthFPS()}",640,495);
    text("Press 'd' to enable/disable depth    Press 'r' to enable/disable rgb image   Press 'i' to enable/disable IR image  UP and DOWN to tilt camera   Framerate: #{@frame_rate}",10,515);
  end
  
  def keyPressed()
    if (key == 'd')
      @depth = !@depth;
      @kinect.enableDepth(@depth);
      
    elsif (key == 'r')
      @rgb = !@rgb;
      @ir = false if @rgb
      @kinect.enableRGB(@rgb);
      
    elsif (key == 'i')
      @ir = !@ir;
      @rgb = false if @ir;
      @kinect.enableIR(@ir);
      
    elsif (key == CODED)
      if (key_code == UP)
        @deg = @deg + 1;
      elsif (key_code == DOWN)
        @deg = @deg - 1;
      end
      @deg = constrain(@deg,0,30);
      p "tilting to: #{@deg}"
      @kinect.tilt(@deg);
    end
  end
  
  def stop()
    @kinect.quit();
    # super.stop();
  end
end