#this conversions are used if the sample is older than time
time = Time.utc 2011,9,19
LogTools::Converter.register "converter for base type sonar scan", time,Orocos.registry do
    conversion "/base/samples/SonarScan","/base/samples/SonarBeam" do |dst,src|
        deep_cast(dst.beam,src.scanData)
        dst.time = src.time
        dst.bearing = src.angle > Math::PI ? src.angle-2*Math::PI : src.angle
        dst.sampling_interval = src.time_beetween_bins
        dst.beamwidth_vertical = 35/180* Math::PI 
        dst.beamwidth_horizontal = 3/180* Math::PI 
        dst.speed_of_sound = 1500
    end
end
