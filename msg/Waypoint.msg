Header header
uint8 id   # id of drone for waypoint
float64 latitude    #degree
float64 longitude    #degree
float32 altitude    #WGS 84 reference ellipsoid fused by barometer
uint8 sample        #boolean to check if you need to sample at waypoint or not
float32 sampleTime  #seconds