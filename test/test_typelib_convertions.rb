require 'test/unit'
require 'typelib'
require 'orogen'

class TC_TypelibConvertions < Test::Unit::TestCase
    attr_reader :registry
    def setup
        @registry = Orocos.registry_of('base')
    end

    def test_time_to_ruby
        time_t = registry.get '/base/Time'
        value = time_t.new
        value.seconds = 10
        value.microseconds = 100

        time = Typelib.to_ruby(value)
        assert_kind_of(Time, time)
        assert_equal(10, time.seconds)
        assert_equal(100, time.microseconds)
        assert_equal(10, time.tv_sec)
        assert_equal(100, time.tv_usec)
    end
    def test_time_from_ruby
        time_t = registry.get '/base/Time'

        time = Time.at(10, 100)
        value = Typelib.from_ruby(time, time_t)
        assert_kind_of(time_t, value)
        assert_equal(10, value.seconds)
        assert_equal(100, value.microseconds)
    end

    def test_vector_to_ruby
        vector_t = registry.get '/wrappers/Vector3'
        value = vector_t.new
        value.data[0] = 10
        value.data[1] = 100
        value.data[2] = 1000

        vector = Typelib.to_ruby(value)
        assert_kind_of(Eigen::Vector3, vector)
        assert_equal([10, 100, 1000], vector.data.to_a)
        assert_equal(10, vector.x)
        assert_equal(100, vector.y)
        assert_equal(1000, vector.z)
    end

    def test_vector_from_ruby
        vector_t = registry.get '/wrappers/Vector3'

        vector = Eigen::Vector3.new(10, 100, 1000)
        value  = Typelib.from_ruby(vector, vector_t)
        assert_kind_of(vector_t, value)
        assert_equal([10, 100, 1000], value.data.to_a)
    end

    def test_quaternion_to_ruby
        quaternion_t = registry.get '/wrappers/Quaternion'
        value = quaternion_t.new
        value.re    = 1
        value.im[0] = 10
        value.im[1] = 100
        value.im[2] = 1000

        quaternion = Typelib.to_ruby(value)
        assert_kind_of(Eigen::Quaternion, quaternion)
        assert_equal(1, quaternion.re)
        assert_equal([10, 100, 1000], quaternion.im.to_a)
        assert_equal(1, quaternion.w)
        assert_equal(10, quaternion.x)
        assert_equal(100, quaternion.y)
        assert_equal(1000, quaternion.z)
    end

    def test_quaternion_from_ruby
        quaternion_t = registry.get '/wrappers/Quaternion'

        quaternion = Eigen::Quaternion.new(1, 10, 100, 1000)
        value  = Typelib.from_ruby(quaternion, quaternion_t)
        assert_kind_of(quaternion_t, value)
        assert_equal(1, value.re)
        assert_equal([10, 100, 1000], value.im.to_a)
    end

    def test_matrixx_to_ruby
        matrixxd_t = registry.get 'wrappers/MatrixXd'
        value = matrixxd_t.new

        value.rows = 2
        value.cols = 3
        value.data = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

        matrixxd = Typelib.to_ruby(value)
        assert_kind_of(Eigen::MatrixX,matrixxd)
        assert_equal(matrixxd.rows(),2)
        assert_equal(matrixxd.cols(),3)
        assert_equal([1.0,2.0,3.0,4.0,5.0,6.0],matrixxd.to_a)
    end
    
    def test_matrixx_from_ruby
        matrixxd_t = registry.get 'wrappers/MatrixXd'
        matrixxd = Eigen::MatrixX.new(2,3)
        matrixxd.from_a([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

        value = Typelib.from_ruby(matrixxd,matrixxd_t)

        assert_equal(value.rows,2)
        assert_equal(value.cols,3)
        assert_equal(value.data.to_a, [1.0,2.0,3.0,4.0,5.0,6.0])
        

    end
    
    def test_vectorx_to_ruby
        matrixxd_t = registry.get 'wrappers/VectorXd'
        value = matrixxd_t.new
        data = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        data.each { |v| value.push(v) }

        vectorxd = Typelib.to_ruby(value)
        assert_kind_of(Eigen::VectorX,vectorxd)
        assert_equal(vectorxd.size(),6)
        assert_equal([1.0,2.0,3.0,4.0,5.0,6.0],vectorxd.to_a)
    end
    
    def test_vectorx_from_ruby
        vectorxd_t = registry.get 'wrappers/VectorXd'
        vectorxd = Eigen::VectorX.new(6)
        vectorxd.from_a([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

        value = Typelib.from_ruby(vectorxd,vectorxd_t)

        assert_equal(value.size,6)
        assert_equal(value.to_a, [1.0,2.0,3.0,4.0,5.0,6.0])
    end

end

