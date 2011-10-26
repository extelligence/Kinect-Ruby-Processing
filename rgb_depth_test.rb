require 'ruby-processing'

class RGBDepthText < Processing::App
  load_library "openkinect"
  include_package 'org.openkinect'
  include_package 'org.openkinect.processing'

  attr_accessor :kinect, :depth, :rgb, :ir, :deg
  alias_method :quit, :stop
  
  def setup()
    @deg = 15
    @depth = true
    @rgb = false
    @ir = false
    
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
    text("Press 'd' to enable/disable depth    Press 'r' to enable/disable rgb image   Press 'i' to enable/disable IR image (crashy!)   Press 'q' to quit   UP and DOWN to tilt camera   Framerate: #{frame_rate}",10,515);
  end
  
  def toggle_rgb
    @rgb = !@rgb;
    @ir = false if @rgb
    @kinect.enableRGB(@rgb);
  end
  
  def toggle_ir
    @ir = !@ir;
    @rgb = false if @ir;
    @kinect.enableIR(@ir);
  end
  
  def tilt_up
    @deg = @deg + 1;
    tilt_now
  end
  
  def tilt_down
    @deg = @deg - 1;
    tilt_now
  end
  
  def tilt_now
    @deg = constrain(@deg,0,30);
    @kinect.tilt(@deg);
  end
  
  def keyPressed()
    if (key == 'd')
      @depth = !@depth;
      @kinect.enableDepth(@depth);
      
    elsif (key == 'r')
      toggle_rgb
      
    elsif (key == 'i')
      toggle_ir
      
    elsif (key == CODED)
      if (key_code == UP)
        tilt_up
      elsif (key_code == DOWN)
        tilt_down
      end
      
      elsif (key == 'q')
        stop
    end
  end
  
  def stop()
    @kinect.quit();
    super.stop
    # exit
  end
end