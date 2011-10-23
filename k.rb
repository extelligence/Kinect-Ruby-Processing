require 'ruby-processing'

class Sketch < Processing::App
  load_library "openkinect"
  include_package 'org.openkinect'
  include_package 'org.openkinect.processing'

  attr_accessor :kinect, :depth, :rgb, :ir, :deg
  
  def setup()
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
      @ir = false if !!@rgb
      @kinect.enableRGB(@rgb);
      
    elsif (key == 'i')
      @ir = !@ir;
      @rgb = false if !!@ir;
      @kinect.enableIR(@ir);
      
    elsif (key == CODED)
      if (key_code == UP)
        @deg = @deg + 1;
      elsif (key_code == DOWN)
        @deg = @deg - 1;
      end
      @deg = constrain(@deg,0,30);
      @kinect.tilt(@deg);
      
      elsif (key == 's')
        stop
    end
  end
  
  def stop()
    @kinect.quit();
    # super.stop();
  end
end