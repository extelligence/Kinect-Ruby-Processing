# Based on the pure Processing example by Daniel Shiffman
# Kinect Point Cloud example
# http://www.shiffman.net
# https://github.com/shiffman/libfreenect/tree/master/wrappers/java/processing
require 'ruby-processing'

class PointCloud < Processing::App
  load_library "openkinect"
  include_package 'org.openkinect'
  include_package 'org.openkinect.processing'

  # Kinect Library object
  attr_accessor :kinect
  
  attr_accessor :a

  # Size of kinect image
  attr_accessor :w, :h

  # We'll use a lookup table so that we don't have to repeat the math over and over
  attr_accessor :depth_lookup

  def setup
    @a = 0.0
    @w = 640
    @h = 480
    @depth_lookup = Array.new 2048
    
    @fx_d = 1.0 / 5.9421434211923247e+02
    @fy_d = 1.0 / 5.9104053696870778e+02
    @cx_d = 3.3930780975300314e+02
    @cy_d = 2.4273913761751615e+02

    size 800, 600, P3D
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
  end
 
  def draw
    background 0
    fill 255
    text_mode SCREEN
    text "Kinect FR: #{@kinect.getDepthFPS}\nProcessing FR: #{frame_rate}",10,16

    # Get the raw depth as array of integers
    depth = @kinect.getRawDepth
  
    # We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
    skip = 4;
  
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
        v = depth_to_world x, y, raw_depth

        stroke 255
        push_matrix

        # Scale up by 200
        factor = 200
        translate v.x*factor, v.y*factor, factor-v.z*factor

        # Draw a point
        point 0,0
        pop_matrix
        
        y += skip
      end
      x += skip
    end
  
    # Rotate
    @a += 0.015
  end
  
  # These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
  def raw_depth_to_meters(depth_value)
    if (depth_value < 2047)
      return 1.0 / Float(Float(depth_value) * -0.0030711016 + 3.3309495161)
    end
    return 0.0
  end
  
  def depth_to_world(x, y, depth_value)
    depth = @depth_lookup[depth_value]
    result = PVector.new
    result.x = Float((x - @cx_d) * depth * @fx_d)
    result.y = Float((y - @cy_d) * depth * @fy_d)
    result.z = Float(depth)
    return result
  end
  
  def stop
    @kinect.quit
    super.stop
  end

end
