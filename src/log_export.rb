require 'frame_helper_ruby'

module LogTools
    class Frame2Image 
        include ExporterBase

        @type_name = "/base/samples/frame/Frame"
        @suffix = ["png","jpg"]
        @doc = "exports base/sample/Frame as png or jpg"

        #called for each stream
        def initialize(stream)
            Orocos.load_typekit 'base'
        end

        def export(value,filename,options = Hash.new)
            FrameHelper.save_frame(filename,value)
        end
    end
end
