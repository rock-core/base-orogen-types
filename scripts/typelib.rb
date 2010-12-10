# If we get a /base/Time, convert it to Ruby's Time class
Typelib.convert_to_ruby '/base/Time' do |value|
    microseconds = value.microseconds
    seconds = microseconds / 1_000_000
    Time.at(seconds, microseconds % 1_000_000)
end
# Tell Typelib that Time instances can be converted into /base/Time values
Typelib.convert_from_ruby Time, '/base/Time' do |value, typelib_type|
    result = typelib_type.new
    result.seconds      = value.tv_sec
    result.microseconds = value.tv_usec
    result
end


begin
    require 'eigen'
    Typelib.convert_to_ruby '/wrappers/Vector3' do |value|
        Eigen::Vector3.new(*value.data.to_a)
    end
    Typelib.convert_from_ruby Eigen::Vector3, '/wrappers/Vector3' do |value, type|
        type.new(:data => value.to_a)
    end
    Typelib.convert_to_ruby '/wrappers/Quaternion' do |value|
        Eigen::Quaternion.new(value.re, *value.im.to_a)
    end
    Typelib.convert_from_ruby Eigen::Quaternion, '/wrappers/Quaternion' do |value, type|
        data = value.to_a
        type.new(:re => data[0], :im => data[1, 3])
    end
rescue LoadError
    STDERR.puts "The Ruby Eigen library is not present, I am not providing extensions for the base geometry types"
end

