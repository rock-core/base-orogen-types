require 'orocos/log/converter'
require 'orocos'

Orocos.initialize
include Orocos::Log

#load all conversions for base types 
require_dir File.join(File.dirname(__FILE__),'immigration')

