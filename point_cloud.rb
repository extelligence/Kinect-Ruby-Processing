# Port of the pure Processing example by Daniel Shiffman
# Kinect Point Cloud example
# http://www.shiffman.net
# https://github.com/shiffman/libfreenect/tree/master/wrappers/java/processing
require 'ruby-processing'

ROTATION_SLIDER_SCALE = 1.0/100.0
DEFAULT_SKIP = 4
DEFAULT_ROTATION_DELTA = 0.015
# DEFAULT_TRAIL_FRAMES = 0


class PointCloud < Processing::App
  load_libraries :openkinect, :control_panel
  include_package 'org.openkinect'
  include_package 'org.openkinect.processing'

  # Kinect Library object
  attr_accessor :kinect
  
  attr_accessor :a
  attr_reader :skip

  # Size of kinect image
  attr_accessor :w, :h

  # We'll use a lookup table so that we don't have to repeat the math over and over
  attr_accessor :depth_lookup

  def setup
    # control panel defaults
    @skip = DEFAULT_SKIP
    @rotation_delta = DEFAULT_ROTATION_DELTA
    # @trail_frames = DEFAULT_TRAIL_FRAMES
    
    @a = 0.0
    @w = 640
    @h = 480
    @depth_lookup = Array.new 2048
    
    @fx_d = 1.0 / 5.9421434211923247e+02
    @fy_d = 1.0 / 5.9104053696870778e+02
    @cx_d = 3.3930780975300314e+02
    @cy_d = 2.4273913761751615e+02

    size 800, 600, P3D
    setup_control
    @kinect = Kinect.new self
    @kinect.start
    @kinect.enableDepth true
    # We don't need the grayscale image in this example
    # so this makes it more efficient
    @kinect.processDepthImage false

    # Lookup table for all possible depth values (0 - 2047)
    @depth_lookup.each_with_index do |depth, i|
      @depth_lookup[i] = raw_depth_to_meters i
    end
    
    background 0
    stroke 255
    fill 255
    text_mode SCREEN
    
    # Scale up by 200
    @factor = 200.0
  end
 
  def draw
    background 0
    text "Kinect FR: #{@kinect.getDepthFPS}\nProcessing FR: #{frame_rate}\n[Q]uit.",10,16

    # Get the raw depth as array of integers
    depth = @kinect.getRawDepth
  
    # Translate and rotate
    translate width/2, height/2, -50
    rotateY @a
  
    x = 0
    while x < @w
      y = 0
      while y < @h
        offset = x+y*@w
        
        # Convert kinect data to world xyz coordinate
        raw_depth = depth[offset]

        push_matrix
        translate_world x, y, raw_depth

        # Draw a point
        point 0,0
        pop_matrix
        
        # We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
        y += skip
      end
      x += skip
    end
  
    # Rotate
    @a += @rotation_delta
  end
  
  # These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
  def raw_depth_to_meters(depth_value)
    if (depth_value < 2047)
      return 1.0 / Float(Float(depth_value) * -0.0030711016 + 3.3309495161)
    end
    return 0.0
  end
  
  def translate_world(x, y, depth_value)
    depth = @depth_lookup[depth_value]
    translate Float((x - @cx_d) * depth * @fx_d) * @factor,
              Float((y - @cy_d) * depth * @fy_d) * @factor,
              @factor - Float(depth) * @factor
  end
  
  def setup_control
    control_panel do |c|
      c.title = "Point Cloud"
      c.slider(:point_skip, 1..100, @skip) do |v|
        @skip = Integer v
      end
      c.slider(:rotation_delta, -2..2, @rotation_delta/ROTATION_SLIDER_SCALE) do |v|
        # value on slider scaled down by 1x10^-2
        @rotation_delta = Float v*ROTATION_SLIDER_SCALE
      end
      # c.slider(:trail_frames, 0..5, @trail_frames) do |v|
      #   @trail_frames = Integer v
      # end
      # c.slider :opacity
      # c.slider(:app_width, 5..60, 20) { reset! }
      # c.menu(:options, ['one', 'two', 'three'], 'two') {|m| load_menu_item(m) }
      # c.checkbox :paused
      c.button :reset!
    end
  end
  
  def reset!
    @skip = DEFAULT_SKIP
    @rotation_delta = DEFAULT_ROTATION_DELTA
    # @trail_frames = DEFAULT_TRAIL_FRAMES
  end
  
  def keyPressed()
    if (key == 'q' || key == 'Q')
      stop
    end
  end
  
  def stop
    @kinect.quit
    super.stop if super.respond_to? :stop
    exit
  end

end
