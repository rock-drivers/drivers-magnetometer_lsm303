require 'ffi'

module FfiMag
    extend FFI::Library
    ffi_lib 'libmagnetometer_lsm303.so'
    attach_function :C_Create, [], :pointer
    attach_function :C_Destroy, [:pointer], :void
    attach_function :C_open, [:pointer,:string], :void
    attach_function :C_read, [:pointer], :void
    attach_function :C_getDevNo, [:pointer], :int
    attach_function :C_setReadTimeout, [:pointer,:int], :void
end

mag = FfiMag.C_Create
FfiMag.C_open(mag,"serial:///dev/ttyUSB0:57600")
FfiMag.C_setReadTimeout(mag,20)

loop do
    FfiMag.C_read(mag)
    devno = FfiMag.C_getDevNo(mag)
    puts devno
end




