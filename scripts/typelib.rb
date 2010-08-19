# If we get a /base/Time, convert it to Ruby's Time class
Typelib.convert_to_ruby '/base/Time' do |value|
    Time.at(value.seconds, value.microseconds)
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
    Typelib.convert_to_ruby '/wrappers/Quaternion' do |value|
        Eigen::Quaternion.new(value.re, *value.im.to_a)
    end
rescue LoadError
    STDERR.puts "The Ruby Eigen library is not present, I am not providing extensions for the base geometry types"
end

