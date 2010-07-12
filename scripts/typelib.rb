Typelib.specialize_model '/base/Time' do
    def to_ruby(value)
        Typelib::DynamicWrapper(Time.at(value.seconds, value.microseconds), value)
    end
end
Typelib.convert_from_ruby Time, '/base/Time' do |value, typelib_type|
    result = typelib_type.new
    result.seconds      = value.tv_sec
    result.microseconds = value.tv_usec
    result
end


begin
    require 'eigen'
    Typelib.specialize_model '/wrappers/Vector3' do
        def to_ruby(value)
            Typelib::DynamicWrapper(Eigen::Vector3.new(*value.data.to_a), value)
        end
    end
    Typelib.specialize_model '/wrappers/Quaternion' do
        def to_ruby(value)
            Typelib::DynamicWrapper(Eigen::Quaternion.new(value.re, *value.im.to_a), value)
        end
    end
rescue LoadError
    STDERR.puts "The Ruby Eigen library is not present, I am not providing extensions for the base geometry types"
end

