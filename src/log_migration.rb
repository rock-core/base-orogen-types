#this conversions are used if the sample is older than time
time = Time.utc 2011,9,19
LogTools::Converter.register "converter for base type sonar scan", time,Orocos.registry do
    conversion "/base/samples/SonarScan","/base/samples/SonarBeam" do |dst,src|
        deep_cast(dst.beam,src.scanData)
        dst.time = src.time
        dst.bearing.rad = Math::PI-src.angle
        dst.bearing.rad = dst.bearing.rad - 2*Math::PI if dst.bearing.rad > Math::PI
        dst.sampling_interval = src.time_beetween_bins
        dst.beamwidth_vertical = 35/180* Math::PI 
        dst.beamwidth_horizontal = 3/180* Math::PI 
        dst.speed_of_sound = 1500
    end
end
time = Time.now
LogTools::Converter.register "converter for non base type MultilevelLaserScan", time, Orocos.registry do
    conversion "/velodyne_lidar/MultilevelLaserScan","/base/samples/DepthMap" do |dst,src|
        if not src.horizontal_scans.empty? then
            dst.horizontal_size = src.horizontal_scans.size
            dst.vertical_size = src.horizontal_scans.first.vertical_scans.size
            dst.vertical_interval << -10.666667/180.0* Math::PI 
            dst.vertical_interval << 30.666667/180.0* Math::PI 
            dst.distances = Array.new(dst.horizontal_size * dst.vertical_size, Float::INFINITY)
            dst.remissions = Array.new(dst.horizontal_size * dst.vertical_size, 0.0)
            dst.vertical_projection = 0
            dst.horizontal_projection = 0

            horizontal_index = 0            
            src.horizontal_scans.each do |horizontal_entry|
                vertical_index = 0
                horizontal_entry.vertical_scans.each do |scan|
                    index = vertical_index * dst.horizontal_size + horizontal_index
                    if(scan.range > 5)
                        dst.distances[index] = 0.001 * scan.range
                    end
                    dst.remissions[index] = scan.remission
                    vertical_index += 1
                end
                dst.timestamps << horizontal_entry.time
                dst.time = horizontal_entry.time
                dst.horizontal_interval << horizontal_entry.horizontal_angle.rad
                horizontal_index += 1
            end

        end
    end
end

LogTools::Converter.register "converter for DepthMap missing the time field", Time.now, Orocos.registry do
    conversion "/base/samples/DepthMap","/base/samples/DepthMap" do |dst,src|
        if dst.class.has_field?('time') and not src.class.has_field?('time') and not src.timestamps.empty? then
            dst.time = src.timestamps.last
        end
        deep_cast(dst,src,:self)
    end
end
