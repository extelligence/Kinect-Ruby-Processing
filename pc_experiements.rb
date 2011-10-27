# Port of the pure Processing example by Daniel Shiffman
# Kinect Point Cloud example
# http://www.shiffman.net
# https://github.com/shiffman/libfreenect/tree/master/wrappers/java/processing
require 'ruby-processing'

ROTATION_SLIDER_SCALE = 1.0/100.0
DEFAULT_SKIP = 10
DEFAULT_ROTATION_DELTA = 0.015
DEFAULT_TRAIL_FRAMES_SIZE = 13
DEFAULT_SHAPE = "point"
DEFAULT_SHAPE_WIDTH = 1.0
DEFAULT_SHAPE_HEIGHT = 1.0
DEFAULT_TRAIL_R = 204
DEFAULT_TRAIL_G = 0
DEFAULT_TRAIL_B = 132
DEFAULT_POINT_R = 0
DEFAULT_POINT_G = 255
DEFAULT_POINT_B = 255


class PointCloud < Processing::App
  load_libraries :openkinect, :control_panel
  include_package 'org.openkinect'
  include_package 'org.openkinect.processing'

  # Kinect Library object
  attr_accessor :kinect
  
  attr_accessor :a
  attr_reader :skip
  attr_accessor :trail_r, :trail_g, :trail_b
  
  # Size of kinect image
  attr_accessor :w, :h

  # We'll use a lookup table so that we don't have to repeat the math over and over
  attr_accessor :depth_lookup

  def setup
    # control panel defaults
    @skip = DEFAULT_SKIP
    @rotation_delta = DEFAULT_ROTATION_DELTA
    @trail_frames_size = DEFAULT_TRAIL_FRAMES_SIZE
    @trail_frames = []
    @trail_frame = []
    @shape_width = DEFAULT_SHAPE_WIDTH
    @shape_height = DEFAULT_SHAPE_HEIGHT
    
    @a = 0.0
    @w = 640
    @h = 480
    @depth_lookup = Array.new 2048
    @allow_rotation = true
    @allow_trail = false
    @point_r = DEFAULT_POINT_R
    @point_g = DEFAULT_POINT_G
    @point_b = DEFAULT_POINT_B
    @trail_r = DEFAULT_TRAIL_R
    @trail_g = DEFAULT_TRAIL_G
    @trail_b = DEFAULT_TRAIL_B
    
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
    
    # background 0
    text_mode SCREEN
    
    # Scale up by 200
    @factor = 200.0
  end
 
  def draw
    background 0
    stroke @point_r, @point_g, @point_b
    fill @point_r, @point_g, @point_b
    text "Kinect FR: #{@kinect.getDepthFPS}\nProcessing FR: #{frame_rate}\n[Q]uit",10,16
    # @trail_frame.slice!(0, @trail_frame.size)
    @trail_frame = []
    @is_recording = save_trail_frame?
    
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
        draw_point
        pop_matrix
        
        # We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
        y += skip
      end
      x += skip
    end
  
    if @is_recording
      draw_trail_frames
  
      # save trail frame
      save_trail_frame
    end
    
    # Rotate
    @a += @rotation_delta if @allow_rotation
  end
  
  def save_trail_frame?
    return @allow_trail && @trail_frames_size > 0
  end
  
  def save_trail_frame
    @trail_frames.delete_at 0 if @trail_frames.size >= @trail_frames_size
    @trail_frames << @trail_frame
  end
  
  def draw_point
    case @shape
    when "ellipse"
      ellipse 0, 0, @shape_width, @shape_height
    when "rect"
      rect 0, 0, @shape_width, @shape_height
    else 
      point 0, 0
    end
  end
  
  def translate_draw_point(x, y, z, cum_a)
    push_matrix
    translate x, y, z
    draw_point
    pop_matrix
  end
  
  def draw_trail_frames
    @trail_frames.each_with_index do |the_frame, i|
      # no_stroke
      d = max(1.0, @trail_frames.size)

      color_fade = 1.0*(i+1.0)/Float(d)
      alpha_fade = map(color_fade, 0.0, 1.0, 0.2*255, 255)

      # fill @trail_r, @trail_g, @trail_b, alpha_fade
      # stroke @trail_r, @trail_g, @trail_b, alpha_fade
      fill @trail_r*color_fade, @trail_g*color_fade, @trail_b*color_fade, alpha_fade
      stroke @trail_r*color_fade, @trail_g*color_fade, @trail_b*color_fade, alpha_fade
      
      push_matrix
      delta_a = the_frame.first[3] - @a
      rotateY delta_a
      the_frame.each do |pt|
        translate_draw_point *pt
      end
      pop_matrix
    end
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
    x = Float((x - @cx_d) * depth * @fx_d) * @factor
    y = Float((y - @cy_d) * depth * @fy_d) * @factor
    z = @factor - Float(depth) * @factor
    
    @trail_frame << [x, y, z, a] if @is_recording
    translate x, y, z
  end
  
  def setup_control
    control_panel do |c|
      c.title = "Point Cloud"
      c.slider(:point_skip, 1..100, @skip) do |v|
        @skip = Integer v
      end
      c.checkbox :allow_rotation, @allow_rotation
      c.slider(:rotation_delta, -2..2, @rotation_delta/ROTATION_SLIDER_SCALE) do |v|
        # value on slider scaled down by 1x10^-2
        @rotation_delta = Float v*ROTATION_SLIDER_SCALE
      end
      c.checkbox :allow_trail, @allow_trail
      c.slider(:trail_frames_size, 0..150, @trail_frames_size) do |v|
        @trail_frames_size = Integer v
        
        # trim trail 
        if (@trail_frames.size > @trail_frames_size)
          @trail_frames.slice! 0, @trail_frames.size - @trail_frames_size
        end
      end
      c.slider(:shape_width, 1..100, @shape_width)
      c.slider(:shape_height, 1..100, @shape_height)
      c.slider(:point_r, 0..255, @point_r)
      c.slider(:point_g, 0..255, @point_g)
      c.slider(:point_b, 0..255, @point_b)
      c.slider(:trail_r, 0..255, @trail_r)
      c.slider(:trail_g, 0..255, @trail_g)
      c.slider(:trail_b, 0..255, @trail_b)
      c.menu(:shape, ['ellipse', 'point', 'rect'], DEFAULT_SHAPE)
      c.button :clear_trail
      c.button :reset!
    end
  end
  
  def clear_trail
    @trail_frames.slice!(0, @trail_frames.size)
  end
  
  def reset!
    @skip = DEFAULT_SKIP
    @rotation_delta = DEFAULT_ROTATION_DELTA
    @trail_frames = []
    @shape = DEFAULT_SHAPE
    @shape_width = DEFAULT_SHAPE_WIDTH
    @shape_height = DEFAULT_SHAPE_HEIGHT
    @point_r = DEFAULT_POINT_R
    @point_g = DEFAULT_POINT_G
    @point_b = DEFAULT_POINT_B
    @trail_r = DEFAULT_TRAIL_R
    @trail_g = DEFAULT_TRAIL_G
    @trail_b = DEFAULT_TRAIL_B
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
